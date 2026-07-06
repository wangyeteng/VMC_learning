"""
参考模型生成器 (Reference Model Generator)

VMC控制器需要参考模型来生成"理想的"车辆状态响应:
  - r_ref: 参考横摆角速度
  - β_ref: 参考质心侧偏角 (通常希望为0或很小)

参考模型基于二自由度线性模型，同时受轮胎附着极限约束。

核心公式:
  r_ref(s) / δ(s) = Gr / (τr·s + 1)  ← 一阶滞后近似
  β_ref(s) / δ(s) = Gβ / (τβ·s + 1)

  r_ss = Vx / (L + Kus·Vx²) · δ     ← 稳态增益
  |r_max| = μg / Vx                   ← 附着极限约束
  |β_max| = arctan(0.02·μg)          ← 经验上限

参考:
  Rajamani Ch.3
  《汽车理论》第5章
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class RefModelParams:
    """参考模型参数"""
    # 车辆基本参数
    m: float = 1500.0       # 质量 [kg]
    Iz: float = 2500.0      # 横摆转动惯量 [kg·m²]
    lf: float = 1.2         # 前轴到质心 [m]
    lr: float = 1.6         # 后轴到质心 [m]
    Cf: float = -80000.0    # 前轴侧偏刚度 [N/rad]
    Cr: float = -80000.0    # 后轴侧偏刚度 [N/rad]

    # 参考模型时间常数
    tau_r: float = 0.1      # 横摆角速度一阶滞后 [s]
    tau_beta: float = 0.1   # 质心侧偏角一阶滞后 [s]

    # 限制
    mu: float = 0.85        # 标称路面附着系数
    g: float = 9.81         # 重力加速度 [m/s²]


class ReferenceModel:
    """VMC参考模型

    功能:
      1. 根据方向盘转角计算参考横摆角速度和参考质心侧偏角
      2. 应用物理约束 (轮胎附着极限)
      3. 输出平滑的参考信号
    """

    def __init__(self, params: RefModelParams = None):
        self.p = params if params else RefModelParams()
        self._prev_r_ref = 0.0
        self._prev_beta_ref = 0.0

    @property
    def L(self) -> float:
        return self.p.lf + self.p.lr

    def understeer_gradient(self) -> float:
        """不足转向梯度"""
        p = self.p
        return (p.m / self.L) * (p.lr / p.Cf - p.lf / p.Cr)

    def steady_state_yaw_gain(self, Vx: float) -> float:
        """稳态横摆角速度增益 r_ss / δ"""
        Kus = self.understeer_gradient()
        # 防止分母为零或负
        denom = self.L + Kus * Vx**2
        if abs(denom) < 1e-6:
            return Vx / 1e-6  # 防止除零
        return Vx / denom

    def max_yaw_rate(self, Vx: float) -> float:
        """轮胎附着极限约束的横摆角速度上限

        |r_max| = μ·g / Vx

        这是轮胎物理极限——再大的方向盘转角也无法超过此值
        """
        if Vx < 0.1:
            return float('inf')
        return self.p.mu * self.p.g / Vx

    def max_sideslip(self) -> float:
        """经验质心侧偏角上限

        干燥路面: |β_max| ≈ 2~3°
        低附路面: |β_max| 更小
        """
        return np.arctan(0.02 * self.p.mu * self.p.g)

    def generate(self, delta: float, Vx: float, dt: float,
                 beta_ref_desired: float = 0.0) -> tuple:
        """生成参考状态

        Args:
            delta: 前轮转角 [rad]
            Vx: 纵向速度 [m/s]
            dt: 采样时间 [s]
            beta_ref_desired: 期望的质心侧偏角 (通常为0)

        Returns:
            (r_ref, beta_ref): 参考横摆角速度 [rad/s], 参考质心侧偏角 [rad]
        """
        # 1. 稳态横摆角速度
        r_ss = self.steady_state_yaw_gain(Vx) * delta

        # 2. 附着极限约束
        r_max = self.max_yaw_rate(Vx)
        r_limited = np.clip(r_ss, -r_max, r_max)

        # 3. 一阶滞后滤波 (模拟车辆动力学的响应延迟)
        alpha = dt / (self.p.tau_r + dt)
        r_ref = self._prev_r_ref + alpha * (r_limited - self._prev_r_ref)

        # 4. 质心侧偏角参考 (通常希望为0, 也受一阶滞后和极限约束)
        beta_max = self.max_sideslip()
        alpha_beta = dt / (self.p.tau_beta + dt)
        beta_ref = self._prev_beta_ref + alpha_beta * (beta_ref_desired - self._prev_beta_ref)
        beta_ref = np.clip(beta_ref, -beta_max, beta_max)

        # 存储状态用于下一时刻
        self._prev_r_ref = r_ref
        self._prev_beta_ref = beta_ref

        return r_ref, beta_ref

    def reset(self):
        """重置参考模型状态"""
        self._prev_r_ref = 0.0
        self._prev_beta_ref = 0.0


# ---- 典型车辆参数 ----

def sport_sedan_params() -> RefModelParams:
    """运动型轿车参考模型参数"""
    return RefModelParams(
        m=1600, Iz=2800, lf=1.3, lr=1.7,
        Cf=-90000, Cr=-85000,  # 更硬的侧偏刚度 → 更运动
        tau_r=0.08, tau_beta=0.08,
        mu=1.0
    )


def suv_params() -> RefModelParams:
    """SUV参考模型参数"""
    return RefModelParams(
        m=2200, Iz=4000, lf=1.4, lr=1.6,
        Cf=-75000, Cr=-70000,  # 更软的侧偏刚度 → 更舒适/保守
        tau_r=0.12, tau_beta=0.12,
        mu=0.85
    )


# ---- 测试 ----

if __name__ == '__main__':
    import matplotlib.pyplot as plt

    ref_model = ReferenceModel(sport_sedan_params())

    # 模拟正弦方向盘输入
    t = np.arange(0, 10, 0.01)
    Vx = 25.0  # 90 km/h

    r_ref_hist = []
    beta_ref_hist = []
    delta_hist = []

    for ti in t:
        delta = 0.05 * np.sin(2 * np.pi * 0.5 * ti)  # 0.5 Hz 正弦
        r_ref, beta_ref = ref_model.generate(delta, Vx, 0.01)
        r_ref_hist.append(r_ref)
        beta_ref_hist.append(np.rad2deg(beta_ref))
        delta_hist.append(np.rad2deg(delta))

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(t, delta_hist); axes[0].set_ylabel('δ [deg]'); axes[0].grid(True)
    axes[1].plot(t, np.rad2deg(r_ref_hist)); axes[1].set_ylabel('r_ref [deg/s]'); axes[1].grid(True)
    axes[2].plot(t, beta_ref_hist); axes[2].set_ylabel('β_ref [deg]'); axes[2].set_xlabel('Time [s]'); axes[2].grid(True)
    plt.suptitle('Reference Model Output @ 90 km/h')
    plt.tight_layout()
    plt.show()
