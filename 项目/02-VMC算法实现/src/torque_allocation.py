"""
扭矩分配模块 (Torque Allocation / Control Allocation)

将VMC上层控制器输出的目标力/力矩 [Fx_total, Mz_des] 分配到各轮扭矩。

三种方法:
  1. 规则式分配 (Rule-based): 基于前馈MAP → 简单、实时性好
  2. QP优化分配 (QP-based): 约束二次规划 → 最优、但需要求解器
  3. 伪逆分配 (Pseudo-inverse): 无约束最小二乘 → 快速但无法处理约束

典型的三电机构型:
  前轴: 一个电机驱动两前轮 (差速器分配)
  后轴: 左右各一个电机独立驱动
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class PlatformConfig:
    """底盘平台参数 (易三方三电机)"""
    # 几何
    tw_f: float = 1.6   # 前轮距 [m]
    tw_r: float = 1.6   # 后轮距 [m]
    r_w: float = 0.35   # 轮胎滚动半径 [m]

    # 电机限制
    T_max_front: float = 250.0   # 前轴电机最大扭矩 [Nm]
    T_max_rear: float = 200.0    # 后轴单电机最大扭矩 [Nm]
    T_min_regen: float = -100.0  # 最大回收制动扭矩 [Nm] (负值)

    # 电池功率限制
    P_bat_max: float = 300e3     # 最大放电功率 [W]
    P_bat_min: float = -150e3    # 最大充电功率 [W] (回收)

    # 制动
    T_brake_max: float = -2000.0  # 液压制动最大扭矩 [Nm]


class TorqueAllocator:
    """扭矩分配器

    输入: [Fx_des, Mz_des] (总纵向力需求和横摆力矩需求)
    输出: [T_fl, T_fr, T_rl, T_rr] (四轮扭矩指令)
    """

    def __init__(self, config: PlatformConfig):
        self.cfg = config
        self._prev_T = np.zeros(4)

    # ---- 控制效能矩阵 ----

    def control_effectiveness_matrix(self) -> np.ndarray:
        """构造控制效能矩阵 B (2×4)

        [Fx_total]   [1/rw,      1/rw,      1/rw,      1/rw     ] [T_fl]
        [        ] = [                                          ] [T_fr]
        [Mz_total]   [-tw_f/2rw, tw_f/2rw, -tw_r/2rw, tw_r/2rw] [T_rl]
                                                                [T_rr]

        Returns:
            B: 2×4 矩阵
        """
        cfg = self.cfg
        rw = cfg.r_w
        return np.array([
            [1/rw,              1/rw,              1/rw,              1/rw],
            [-cfg.tw_f/(2*rw),  cfg.tw_f/(2*rw),  -cfg.tw_r/(2*rw),  cfg.tw_r/(2*rw)],
        ])

    # ---- 方法1: 规则式分配 ----

    def allocate_rule_based(self, Fx_des: float, Mz_des: float,
                            axle_distribution: float = 0.5
                            ) -> np.ndarray:
        """规则式扭矩分配

        简单策略:
          1. 前后轴按指定比例分配总驱动力
          2. 横摆力矩完全由后轴差动产生 (三电机优势!)
          3. 如果后轴差动不足，再考虑前轴制动辅助

        Args:
            Fx_des: 总纵向力需求 [N], 正值=加速
            Mz_des: 横摆力矩需求 [N·m], 正值=左转方向(逆时针)
            axle_distribution: 前轴驱动力占比 (0=全后驱, 0.5=50:50)

        Returns:
            T: [T_fl, T_fr, T_rl, T_rr] [Nm]
        """
        cfg = self.cfg
        rw = cfg.r_w

        # 前后轴驱动力分配
        Fx_front = Fx_des * axle_distribution
        Fx_rear = Fx_des * (1 - axle_distribution)

        # 后轴差动产生横摆力矩
        # Mz = (T_rr - T_rl) * tw_r / (2 * rw)
        # ⇒ delta_T_rear = Mz * 2 * rw / tw_r
        delta_T_rear = Mz_des * 2 * rw / cfg.tw_r

        # 后轴扭矩
        T_rl = (Fx_rear / 2) * rw - delta_T_rear / 2
        T_rr = (Fx_rear / 2) * rw + delta_T_rear / 2

        # 前轴扭矩 (均分)
        T_fl = (Fx_front / 2) * rw
        T_fr = (Fx_front / 2) * rw

        T = np.array([T_fl, T_fr, T_rl, T_rr])

        # 应用饱和
        return self._saturate_torques(T)

    # ---- 方法2: QP优化分配 (需要cvxpy/qpsolvers) ----

    def allocate_qp(self, Fx_des: float, Mz_des: float,
                    Fy_est: np.ndarray = None, Fz_est: np.ndarray = None,
                    mu_est: float = 0.85
                    ) -> np.ndarray:
        """基于QP的最优扭矩分配

        min  ||B·u - v_des||²_W  +  λ·||u - u_prev||²  +  ε·||u||²
        s.t. u_min ≤ u ≤ u_max (执行器约束)
             轮胎摩擦椭圆约束 (可选)

        Args:
            Fx_des, Mz_des: 控制目标
            Fy_est: 估计的各轮侧向力 [Fy_fl, Fy_fr, Fy_rl, Fy_rr]
            Fz_est: 估计的各轮垂直载荷
            mu_est: 估计的路面附着系数

        Returns:
            T: 最优四轮扭矩
        """
        # 注意: 需要 cvxpy 或 qpsolvers
        # 这里提供一个简化的QP实现，不依赖外部求解器
        try:
            import cvxpy as cp
            return self._allocate_qp_cvxpy(Fx_des, Mz_des, Fy_est, Fz_est, mu_est)
        except ImportError:
            # Fallback to pseudo-inverse if cvxpy not available
            print("Warning: cvxpy not installed, using pseudo-inverse allocation")
            return self.allocate_pinv(Fx_des, Mz_des)

    def _allocate_qp_cvxpy(self, Fx_des: float, Mz_des: float,
                           Fy_est: np.ndarray, Fz_est: np.ndarray,
                           mu_est: float) -> np.ndarray:
        """使用cvxpy求解QP"""
        import cvxpy as cp

        cfg = self.cfg
        B = self.control_effectiveness_matrix()
        v_des = np.array([Fx_des, Mz_des])

        # 决策变量
        u = cp.Variable(4)

        # 目标函数
        W = np.diag([1.0, 0.1])  # Fx权重高, Mz权重低
        lambda_smooth = 0.01
        lambda_eff = 0.001

        cost = (cp.quad_form(B @ u - v_des, cp.psd_wrap(W)) +
                lambda_smooth * cp.sum_squares(u - self._prev_T) +
                lambda_eff * cp.sum_squares(u))

        # 约束
        constraints = [
            u >= np.array([cfg.T_min_regen] * 4),  # 回收下限
            u <= np.array([cfg.T_max_front, cfg.T_max_front,
                          cfg.T_max_rear, cfg.T_max_rear]),  # 驱动上限
        ]

        # 轮胎摩擦椭圆约束 (如有侧向力估计)
        if Fy_est is not None and Fz_est is not None:
            for i in range(4):
                # Fx_i = u_i / rw, 约束: |Fx_i| ≤ sqrt((μFz_i)² - Fy_i²)
                Fy_avail = np.sqrt(max(0, (mu_est * Fz_est[i])**2 - Fy_est[i]**2))
                constraints.append(u[i] / cfg.r_w <= Fy_avail)
                constraints.append(u[i] / cfg.r_w >= -Fy_avail)

        # 求解
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP, warm_start=True)

        if prob.status in ['optimal', 'optimal_inaccurate']:
            T_opt = u.value
            self._prev_T = T_opt
            return T_opt
        else:
            # QP求解失败，退回规则式分配
            return self.allocate_rule_based(Fx_des, Mz_des)

    # ---- 方法3: 伪逆分配 ----

    def allocate_pinv(self, Fx_des: float, Mz_des: float) -> np.ndarray:
        """伪逆分配 (无约束最小二乘)

        u = B⁺ · v_des,  B⁺ = Bᵀ(BBᵀ)⁻¹
        """
        B = self.control_effectiveness_matrix()
        v_des = np.array([Fx_des, Mz_des])

        # Moore-Penrose伪逆
        B_pinv = B.T @ np.linalg.inv(B @ B.T)
        T = B_pinv @ v_des

        return self._saturate_torques(T)

    # ---- 辅助方法 ----

    def _saturate_torques(self, T: np.ndarray) -> np.ndarray:
        """扭矩饱和处理"""
        cfg = self.cfg
        T_min = np.array([cfg.T_min_regen, cfg.T_min_regen,
                         cfg.T_min_regen, cfg.T_min_regen])
        T_max = np.array([cfg.T_max_front, cfg.T_max_front,
                         cfg.T_max_rear, cfg.T_max_rear])
        return np.clip(T, T_min, T_max)

    def torque_to_force_moment(self, T: np.ndarray) -> Tuple[float, float]:
        """将扭矩向量转换为力和力矩 (用于验证)"""
        B = self.control_effectiveness_matrix()
        v = B @ T
        return v[0], v[1]


# ---- 三电机特有的分配 ----

def tri_motor_allocation(Fx_des: float, Mz_des: float,
                         axle_distribution: float = 0.4) -> np.ndarray:
    """三电机专用扭矩分配

    前轴: 1个电机 → 差速器分配 → 左右前轮均分
    后轴: 2个独立电机 → 可差动

    这是易三方平台的核心差异化算法!
    """
    cfg = PlatformConfig()  # 使用默认参数
    allocator = TorqueAllocator(cfg)

    # 横摆力矩全部由后轴承担 (三电机的设计思路)
    return allocator.allocate_rule_based(Fx_des, Mz_des, axle_distribution)


# ---- 测试 ----

if __name__ == '__main__':
    cfg = PlatformConfig()
    alloc = TorqueAllocator(cfg)

    # 测试场景: 弯道加速
    Fx_des = 2000.0   # 需要2000N驱动力
    Mz_des = 500.0    # 需要500N·m左转横摆力矩

    T_rule = alloc.allocate_rule_based(Fx_des, Mz_des)
    T_pinv = alloc.allocate_pinv(Fx_des, Mz_des)

    print(f"需求: Fx={Fx_des:.0f}N, Mz={Mz_des:.0f}N·m")
    print(f"规则分配: T={np.round(T_rule, 1)} Nm")
    print(f"  验证 → Fx={alloc.torque_to_force_moment(T_rule)[0]:.0f}N, "
          f"Mz={alloc.torque_to_force_moment(T_rule)[1]:.0f}N·m")
    print(f"伪逆分配: T={np.round(T_pinv, 1)} Nm")
    print(f"  验证 → Fx={alloc.torque_to_force_moment(T_pinv)[0]:.0f}N, "
          f"Mz={alloc.torque_to_force_moment(T_pinv)[1]:.0f}N·m")

    # 分析三电机构型的扭矩分配特点
    delta_T_rear = T_rule[3] - T_rule[2]
    print(f"\n后轴扭矩差: {delta_T_rear:.1f} Nm (横摆力矩来源)")
    print(f"后轴总驱动力 = {(T_rule[2]+T_rule[3])/cfg.r_w:.0f} N")
