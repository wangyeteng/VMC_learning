# 项目1: 车辆动力学仿真平台

## 目标
从零搭建一个Python车辆动力学仿真框架，作为后续所有VMC算法的测试平台。

## 项目结构
```
01-车辆模型仿真/
├── README.md
├── requirements.txt
├── src/
│   ├── __init__.py
│   ├── tire.py          # 轮胎模型 (Magic Formula)
│   ├── vehicle_2dof.py  # 二自由度单轨模型
│   ├── vehicle_7dof.py  # 七自由度整车模型
│   ├── kinematics.py    # 坐标系转换与运动学
│   ├── reference.py     # 参考模型生成器
│   └── viz.py           # 可视化工具
├── notebooks/
│   ├── 01_轮胎力曲线.ipynb
│   ├── 02_二自由度稳态响应.ipynb
│   ├── 03_阶跃响应与频率响应.ipynb
│   └── 04_双移线仿真.ipynb
└── tests/
    └── test_vehicle_model.py
```

## 技术路线
- Python 3.9+
- NumPy (数值计算)
- SciPy (积分求解、优化)
- Matplotlib (可视化)
- control (控制系统分析: 频率响应、特征值)
- cvxpy / qpsolvers (优化求解，Phase 2引入)

## 里程碑
1. Week 1: 轮胎模型实现 + 力曲线绘制
2. Week 2: 2-DOF模型实现 + 稳态/瞬态分析
3. Week 3: 转向阶跃/正弦输入仿真
4. Week 4: 7-DOF模型 + 双移线/蛇形工况

## 开始
```bash
cd 项目/01-车辆模型仿真
pip install -r requirements.txt
jupyter notebook notebooks/01_轮胎力曲线.ipynb
```
