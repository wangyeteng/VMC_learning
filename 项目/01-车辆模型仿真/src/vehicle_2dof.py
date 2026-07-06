"""
二自由度单轨模型 (Bicycle Model)

状态: [β, r]ᵀ = [质心侧偏角, 横摆角速度]
输入: δ = 前轮转角

参考:
  Rajamani "Vehicle Dynamics and Control" Ch.3
  《汽车理论》第5章

核心假设:
  1. Vx 恒定 (纵向与横向解耦)
  2. 线性轮胎 (小侧偏角假设)
  3. 忽略侧倾、俯仰、载荷转移
  4. 左右轮合并为等效单轮
"""

import numpy as np
from scipy.integrate import solve_ivp
from dataclasses import dataclass
from typing import Tuple, Callable, Optional


@dataclass
class VehicleParams:
    """整车参数 (典型B级轿车)"""
    m: float = 1500.0          # 整车质量 [kg]
    Iz: float = 2500.0         # 绕Z轴转动惯量 [kg·m²]
    lf: float = 1.2            # 质心到前轴距离 [m]
    lr: float = 1.6            # 质心到后轴距离 [m]
    L: float = None             # 轴距, 自动计算 = lf+lr
    Cf: float = -80000.0       # 前轴等效侧偏刚度 [N/rad] (负值!)
    Cr: float = -80000.0       # 后轴等效侧偏刚度 [N/rad] (负值!)
    Vx: float = 20.0           # 纵向速度 [m/s] (72km/h)

    def __post_init__(self):
        if self.L is None:
            self.L = self.lf + self.lr


class BicycleModel:
    """二自由度线性单轨模型

    状态空间方程:
      dx/dt = A @ x + B @ δ_sw

    其中:
      x = [β, r]ᵀ
      A = [[-(Cf+Cr)/(m·Vx),    -1 - (lf·Cf - lr·Cr)/(m·Vx²)],
           [-(lf·Cf-lr·Cr)/Iz,  -(lf²·Cf + lr²·Cr)/(Iz·Vx)]]

      B = [[Cf/(m·Vx)],
           [lf·Cf/Iz]]
    """

    def __init__(self, params: VehicleParams):
        self.p = params
        self._update_matrices()

    def _update_matrices(self):
        """(重)计算状态空间矩阵 (Vx变化时需调用)"""
        p = self.p
        m, Iz = p.m, p.Iz
        lf, lr = p.lf, p.lr
        Cf, Cr = p.Cf, p.Cr
        Vx = p.Vx

        if Vx < 0.1:
            raise ValueError(f"Vx={Vx} too small, matrices become singular")

        # 注意：Cf, Cr在符号约定下为负值
        # A矩阵元素
        a11 = -(Cf + Cr) / (m * Vx)
        a12 = -1.0 - (lf * Cf - lr * Cr) / (m * Vx**2)
        a21 = -(lf * Cf - lr * Cr) / Iz
        a22 = -(lf**2 * Cf + lr**2 * Cr) / (Iz * Vx)

        self.A = np.array([[a11, a12],
                           [a21, a22]])

        self.B = np.array([[Cf / (m * Vx)],
                           [lf * Cf / Iz]])

    def dynamics(self, t: float, x: np.ndarray, delta: float,
                 Mz_add: float = 0.0) -> np.ndarray:
        """连续时间动力学 ODE

        Args:
            t: 时间 [s]
            x: 状态 [β, r]ᵀ
            delta: 前轮转角 [rad]
            Mz_add: 附加横摆力矩 [N·m] (VMC注入)

        Returns:
            dx/dt: [dβ/dt, dr/dt]ᵀ
        """
        dx = self.A @ x + self.B.flatten() * delta

        # VMC附加横摆力矩的影响
        if abs(Mz_add) > 1e-6:
            dx[1] += Mz_add / self.p.Iz

        return dx

    def step(self, x: np.ndarray, delta: float, dt: float,
             Mz_add: float = 0.0) -> np.ndarray:
        """Euler积分一步 (用于离散时间仿真)

        Args:
            x: 当前状态 [β, r]ᵀ
            delta: 方向盘转角 [rad]
            dt: 时间步长 [s]
            Mz_add: 附加横摆力矩 [N·m]

        Returns:
            x_next: 下一步状态
        """
        dx = self.dynamics(0, x, delta, Mz_add)
        return x + dx * dt

    def set_speed(self, Vx: float):
        """更新纵向速度并重新计算系统矩阵"""
        self.p.Vx = Vx
        self._update_matrices()

    # ---- 稳态分析 ----

    def steady_state_gain(self) -> Tuple[float, float]:
        """稳态增益

        Returns:
            (β_ss/δ_ss, r_ss/δ_ss): 稳态单位前轮转角对应的质心侧偏角和横摆角速度
        """
        x_ss = -np.linalg.solve(self.A, self.B.flatten())
        return x_ss[0], x_ss[1]

    def understeer_gradient(self) -> float:
        """不足转向梯度 Kus

        Kus = (m/L) * (lr/Cf - lf/Cr)

        Returns:
            Kus: [rad/(m/s²)]
              > 0: 不足转向 (Understeer)
              = 0: 中性转向
              < 0: 过多转向 (Oversteer)
        """
        p = self.p
        return (p.m / p.L) * (p.lr / p.Cf - p.lf / p.Cr)

    def characteristic_speed(self) -> Optional[float]:
        """特征车速 (仅不足转向时有效)

        Vchar = sqrt(L / |Kus|)
        在特征车速下，横摆角速度增益达到最大值
        """
        Kus = self.understeer_gradient()
        if Kus > 0:
            return np.sqrt(self.p.L / Kus)
        return None

    def critical_speed(self) -> Optional[float]:
        """临界车速 (仅过多转向时)

        Vcrit = sqrt(L / |Kus|)
        超过此车速车辆失稳
        """
        Kus = self.understeer_gradient()
        if Kus < 0:
            return np.sqrt(self.p.L / abs(Kus))
        return None

    def eigenvalues(self) -> Tuple[complex, complex]:
        """计算系统特征值

        Returns:
            (λ1, λ2): 系统极点
            实部<0 → 稳定; 虚部≠0 → 振荡
        """
        return tuple(np.linalg.eigvals(self.A))

    def natural_frequency_and_damping(self) -> Tuple[float, float]:
        """固有频率 ωn 和 阻尼比 ζ

        Returns:
            (ωn, ζ)
        """
        λ1, λ2 = self.eigenvalues()
        # 对于共轭复极点: λ = -ζωn ± jωn√(1-ζ²)
        ωn = np.sqrt(np.real(λ1)**2 + np.imag(λ1)**2)
        ζ = -np.real(λ1) / ωn
        return ωn, ζ

    # ---- 时域仿真 ----

    def simulate(self, t_span: Tuple[float, float],
                 x0: np.ndarray,
                 delta_func: Callable[[float], float],
                 Mz_func: Callable[[float], float] = None,
                 dt: float = 0.001) -> dict:
        """时域仿真

        Args:
            t_span: (t_start, t_end)
            x0: 初始状态 [β0, r0]
            delta_func: δ = f(t), 前轮转角随时间变化
            Mz_func: Mz = f(t), 附加横摆力矩 (可选)
            dt: 仿真步长

        Returns:
            dict: {'t': 时间序列, 'beta': β序列, 'r': r序列,
                   'delta': δ序列, 'ay': 侧向加速度序列}
        """
        t = np.arange(t_span[0], t_span[1], dt)
        n = len(t)
        x = np.zeros((n, 2))
        x[0] = x0
        delta_hist = np.zeros(n)
        ay_hist = np.zeros(n)

        for i in range(n - 1):
            delta = delta_func(t[i])
            Mz = Mz_func(t[i]) if Mz_func else 0.0
            x[i+1] = self.step(x[i], delta, dt, Mz)
            delta_hist[i] = delta

            # 侧向加速度 ay = Vx*(β̇ + r)  (注意: 这里β̇来自动力学)
            beta, r = x[i]
            beta_dot = self.dynamics(t[i], x[i], delta, Mz)[0]
            ay_hist[i] = self.p.Vx * (beta_dot + r)

        delta_hist[-1] = delta_func(t[-1])

        return {
            't': t,
            'beta': x[:, 0],
            'r': x[:, 1],
            'delta': delta_hist,
            'ay': ay_hist,
            'Vx': self.p.Vx,
        }

    # ---- 汇总信息 ----

    def summary(self) -> str:
        """打印模型汇总信息"""
        Kus = self.understeer_gradient()
        Vchar = self.characteristic_speed()
        Vcrit = self.critical_speed()
        ωn, ζ = self.natural_frequency_and_damping()
        _, r_gain_ss = self.steady_state_gain()

        kind = "不足转向 (Understeer)" if Kus > 1e-6 else \
               "过多转向 (Oversteer)" if Kus < -1e-6 else "中性转向"

        lines = [
            f"=== 二自由度单轨模型 @ Vx={self.p.Vx:.1f} m/s ===",
            f"不足转向梯度 Kus = {Kus:.6f} rad/(m/s²) → {kind}",
            f"稳态横摆增益  r/δ = {r_gain_ss:.3f} (rad/s)/rad",
        ]
        if Vchar:
            lines.append(f"特征车速 Vchar = {Vchar*3.6:.1f} km/h")
        if Vcrit:
            lines.append(f"⚠ 临界车速 Vcrit = {Vcrit*3.6:.1f} km/h")
        lines.append(f"固有频率 ωn = {ωn:.2f} rad/s, 阻尼比 ζ = {ζ:.3f}")

        return "\n".join(lines)


# ---- 常用方向盘输入函数 ----

def step_input(amplitude: float = 0.05, onset_time: float = 0.5) -> Callable:
    """阶跃方向盘输入"""
    return lambda t: amplitude if t >= onset_time else 0.0


def sine_input(amplitude: float = 0.05, frequency: float = 0.5) -> Callable:
    """正弦方向盘输入 (用于频率响应测试)"""
    return lambda t: amplitude * np.sin(2 * np.pi * frequency * t)


def double_lane_change(t: float, Vx: float = 20.0) -> float:
    """双移线工况 — 简化的方向盘转角时间序列

    这只是一个近似，真实双移线应基于轨迹跟踪
    """
    # 双移线的简单位移方案
    if t < 0.5:
        return 0.0
    elif t < 0.5 + 0.7:
        return 0.03 * np.sin(np.pi * (t - 0.5) / 0.7)
    elif t < 0.5 + 0.7 + 0.3:
        return 0.0
    elif t < 0.5 + 0.7 + 0.3 + 0.7:
        return -0.03 * np.sin(np.pi * (t - 0.5 - 0.7 - 0.3) / 0.7)
    else:
        return 0.0


# ---- 演示代码 ----

if __name__ == '__main__':
    # 创建车辆模型
    params = VehicleParams(Vx=20.0)  # 72 km/h
    model = BicycleModel(params)

    # 打印模型信息
    print(model.summary())

    # 阶跃响应仿真
    result = model.simulate(
        t_span=(0, 5),
        x0=np.array([0.0, 0.0]),
        delta_func=step_input(amplitude=0.05, onset_time=1.0),
    )

    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    axes[0, 0].plot(result['t'], np.rad2deg(result['delta']))
    axes[0, 0].set_ylabel('前轮转角 δ [deg]'); axes[0, 0].grid(True)
    axes[0, 0].set_title('方向盘输入')

    axes[0, 1].plot(result['t'], np.rad2deg(result['beta']))
    axes[0, 1].set_ylabel('质心侧偏角 β [deg]'); axes[0, 1].grid(True)
    axes[0, 1].set_title('质心侧偏角响应')

    axes[1, 0].plot(result['t'], np.rad2deg(result['r']))
    axes[1, 0].set_ylabel('横摆角速度 r [deg/s]'); axes[1, 0].set_xlabel('时间 [s]')
    axes[1, 0].grid(True); axes[1, 0].set_title('横摆角速度响应')

    axes[1, 1].plot(result['t'], result['ay'] / 9.81)
    axes[1, 1].set_ylabel('侧向加速度 ay [g]'); axes[1, 1].set_xlabel('时间 [s]')
    axes[1, 1].grid(True); axes[1, 1].set_title('侧向加速度响应')

    plt.tight_layout()
    plt.show()
