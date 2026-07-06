"""
轮胎模型实现 — Magic Formula (Pacejka 2002)

参考:
  - Pacejka, "Tyre and Vehicle Dynamics", 3rd Ed, 2012
  - https://www.mathworks.com/help/vdynblks/ref/magicformulatiremodel.html

Magic Formula 的核心公式:
  y(x) = D * sin(C * arctan(B*x - E*(B*x - arctan(B*x))))
  Y(X) = y(x) + Sv
  x = X + Sh

  参数含义:
  B - 刚度因子  C - 形状因子  D - 峰值因子
  E - 曲率因子  Sh - 水平偏移  Sv - 垂直偏移
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple


@dataclass
class MFCoefficients:
    """Magic Formula 系数 (纯工况)

    参考值来源: Pacejka 2002, 乘用车轮胎典型值
    """
    # 纯纵滑工况
    Bx: float = 12.0     # 纵向刚度因子
    Cx: float = 1.65     # 纵向形状因子
    Dx: float = 1.0      # 纵向峰值因子 (乘以 μ*Fz)
    Ex: float = -0.5     # 纵向曲率因子
    Shx: float = 0.0     # 水平偏移
    Svx: float = 0.0     # 垂直偏移

    # 纯侧偏工况
    By: float = 8.0      # 侧向刚度因子
    Cy: float = 1.3      # 侧向形状因子
    Dy: float = 1.0      # 侧向峰值因子 (乘以 μ*Fz)
    Ey: float = -1.0     # 侧向曲率因子
    Shy: float = 0.0     # 水平偏移
    Svy: float = 0.0     # 垂直偏移

    # 回正力矩 (可选)
    Bt: float = 10.0
    Ct: float = 1.0
    Dt: float = 1.0
    Et: float = -0.5


class MagicFormulaTire:
    """Pacejka Magic Formula 轮胎模型

    支持:
      - 纯纵滑工况 (Fx vs κ)
      - 纯侧偏工况 (Fy vs α)
      - 联合工况 (Fx, Fy vs κ, α) — 基于摩擦椭圆耦合
    """

    def __init__(self, coeffs: MFCoefficients = None,
                 mu: float = 1.0, Fz_nominal: float = 4000.0):
        """
        Args:
            coeffs: Magic Formula 系数
            mu: 路面附着系数 (1.0 = 干燥沥青, 0.3 = 雪, 0.1 = 冰)
            Fz_nominal: 标称垂直载荷 [N]
        """
        self.coeffs = coeffs if coeffs else MFCoefficients()
        self.mu = mu
        self.Fz_nominal = Fz_nominal

    # ---- 纯工况 ----

    def pure_longitudinal(self, kappa: float, Fz: float) -> float:
        """纯纵滑工况: 计算 Fx

        Args:
            kappa: 纵向滑移率, 范围 [-1, 1]
                   正值=驱动滑移, 负值=制动滑移
            Fz: 垂直载荷 [N]

        Returns:
            Fx: 纵向力 [N], 正值=驱动力
        """
        c = self.coeffs
        D = self.mu * c.Dx * Fz
        C = c.Cx
        B = c.Bx * Fz / (C * D) if D > 0 else 0
        E = c.Ex

        x = kappa + c.Shx
        Fx0 = D * np.sin(C * np.arctan(B * x - E * (B * x - np.arctan(B * x))))
        return Fx0 + c.Svx

    def pure_lateral(self, alpha: float, Fz: float, gamma: float = 0.0) -> float:
        """纯侧偏工况: 计算 Fy

        Args:
            alpha: 侧偏角 [rad], 通常范围 [-0.3, 0.3] rad ≈ [-17°, 17°]
            Fz: 垂直载荷 [N]
            gamma: 外倾角 [rad] (暂忽略，保留接口)

        Returns:
            Fy: 侧向力 [N], 正值=指向轮胎左侧 (SAE坐标系)
        """
        c = self.coeffs
        D = self.mu * c.Dy * Fz
        C = c.Cy
        B = c.By * Fz / (C * D) if D > 0 else 0
        E = c.Ey

        x = alpha + c.Shy
        Fy0 = D * np.sin(C * np.arctan(B * x - E * (B * x - np.arctan(B * x))))
        return Fy0 + c.Svy

    # ---- 联合工况 (Combined Slip) ----

    def combined_slip(self, kappa: float, alpha: float, Fz: float
                      ) -> Tuple[float, float]:
        """联合工况: 基于摩擦椭圆耦合计算 Fx, Fy

        采用简化的摩擦椭圆法:
          1. 先计算纯工况下的Fx0, Fy0
          2. 用摩擦椭圆缩放: (Fx/Fx0)² + (Fy/Fy0)² ≤ 1

        Args:
            kappa: 纵向滑移率
            alpha: 侧偏角 [rad]
            Fz: 垂直载荷 [N]

        Returns:
            (Fx, Fy): 纵向力和侧向力 [N]
        """
        Fx0 = self.pure_longitudinal(kappa, Fz)
        Fy0 = self.pure_lateral(alpha, Fz)

        # 摩擦椭圆缩放
        mu_Fz = self.mu * Fz

        if abs(Fx0) < 1e-3 and abs(Fy0) < 1e-3:
            return 0.0, 0.0

        # 计算"理想"合力 (忽略耦合时的合力)
        F_total = np.sqrt(Fx0**2 + Fy0**2)

        if F_total <= mu_Fz:
            # 合力未超限, 不缩放
            return Fx0, Fy0
        else:
            # 合力超限, 按比例缩放以满足摩擦椭圆
            scale = mu_Fz / F_total
            return Fx0 * scale, Fy0 * scale

    # ---- 侧偏刚度 ----

    def cornering_stiffness(self, Fz: float) -> float:
        """计算侧偏刚度 Cα = ∂Fy/∂α |_{α=0}

        在小侧偏角下 (α→0), 可以用数值微分或解析近似

        Args:
            Fz: 垂直载荷 [N]

        Returns:
            Cα: 侧偏刚度 [N/rad], 通常为负值（侧向力方向与侧偏角相反）
        """
        dalpha = 0.001  # 1e-3 rad, 用于数值微分
        Fy_plus = self.pure_lateral(dalpha, Fz)
        Fy_minus = self.pure_lateral(-dalpha, Fz)
        return (Fy_plus - Fy_minus) / (2 * dalpha)

    # ---- 摩擦椭圆边界 ----

    def friction_ellipse_boundary(self, Fx: float, Fz: float) -> float:
        """给定纵向力Fx，返回可用的最大侧向力Fy_max

        基于摩擦椭圆: Fy_max = sqrt((μFz)² - Fx²)

        Args:
            Fx: 当前纵向力 [N]
            Fz: 垂直载荷 [N]

        Returns:
            Fy_max: 可用最大侧向力 [N]
        """
        mu_Fz = self.mu * Fz
        if abs(Fx) >= mu_Fz:
            return 0.0
        return np.sqrt(mu_Fz**2 - Fx**2)


# ---- 方便的函数 ----

def demo_tire_curves():
    """演示：绘制典型的轮胎力曲线"""
    import matplotlib.pyplot as plt

    tire = MagicFormulaTire(mu=1.0, Fz_nominal=4000)

    # 不同载荷下的侧向力曲线
    alpha_range = np.linspace(-0.3, 0.3, 200)  # ±17°
    Fz_values = [2000, 4000, 6000, 8000]  # N

    fig, axes = plt.subplots(1, 3, figsize=(15, 4))

    # 1. 纯侧偏: Fy vs α
    ax = axes[0]
    for Fz in Fz_values:
        Fy = np.array([tire.pure_lateral(a, Fz) for a in alpha_range])
        ax.plot(np.rad2deg(alpha_range), Fy/1000,
                label=f'Fz={Fz/1000:.0f}kN')
    ax.set_xlabel('侧偏角 α [deg]'); ax.set_ylabel('侧向力 Fy [kN]')
    ax.set_title('纯侧偏工况: Fy vs α'); ax.legend(); ax.grid(True)

    # 2. 纯纵滑: Fx vs κ
    ax = axes[1]
    kappa_range = np.linspace(-1, 1, 200)
    for Fz in Fz_values:
        Fx = np.array([tire.pure_longitudinal(k, Fz) for k in kappa_range])
        ax.plot(kappa_range, Fx/1000, label=f'Fz={Fz/1000:.0f}kN')
    ax.set_xlabel('滑移率 κ'); ax.set_ylabel('纵向力 Fx [kN]')
    ax.set_title('纯纵滑工况: Fx vs κ'); ax.legend(); ax.grid(True)

    # 3. 摩擦椭圆
    ax = axes[2]
    for Fz in Fz_values:
        Fx_vec = np.linspace(-tire.mu*Fz, tire.mu*Fz, 100)
        Fy_max = np.array([tire.friction_ellipse_boundary(Fx, Fz) for Fx in Fx_vec])
        ax.plot(Fx_vec/1000, Fy_max/1000, label=f'Fz={Fz/1000:.0f}kN')
        ax.plot(Fx_vec/1000, -Fy_max/1000, '--', color=ax.lines[-1].get_color())
    ax.set_xlabel('纵向力 Fx [kN]'); ax.set_ylabel('侧向力 Fy [kN]')
    ax.set_title('摩擦椭圆 (Friction Ellipse)'); ax.legend()
    ax.set_aspect('equal'); ax.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    demo_tire_curves()
