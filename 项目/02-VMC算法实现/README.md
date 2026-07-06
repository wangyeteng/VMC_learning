# 项目2: VMC算法实现

## 目标
在车辆模型基础上，逐步实现VMC的核心算法模块。

## 项目结构
```
02-VMC算法实现/
├── README.md
├── src/
│   ├── __init__.py
│   ├── reference_model.py   # 参考模型 (生成目标 β_ref, r_ref)
│   ├── torque_allocation.py  # 扭矩分配 (QP/规则)
│   ├── dyc_controller.py     # DYC滑模控制器
│   ├── mpc_controller.py     # MPC控制器 (进阶)
│   ├── state_estimator.py    # EKF/UKF状态估计器
│   └── simulator.py          # 闭环仿真框架
├── notebooks/
│   ├── 01_参考模型与理想响应.ipynb
│   ├── 02_扭矩分配算法对比.ipynb
│   ├── 03_DYC控制闭环仿真.ipynb
│   └── 04_MPC_VMC完整仿真.ipynb
└── tests/
    ├── test_torque_allocation.py
    └── test_dyc.py
```

## 技术栈
- NumPy, SciPy, Matplotlib (基础)
- cvxpy / qpsolvers (QP求解)
- control (系统分析)

## 里程碑
1. Week 5-6: 扭矩分配模块 + 参考模型
2. Week 7-8: DYC控制器 + 闭环仿真
3. Week 13-14: EKF/UKF状态估计器
4. Week 15-16: MPC控制器
