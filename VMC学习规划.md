# 智能电动汽车大脑-小脑架构与自研VMC 学习规划

> **制定日期**: 2026-07-05
> **学习者**: 易三方平台主管 | 流体力学博士 | 数学基础扎实
> **目标**: 全面掌握大脑-小脑架构与VMC技术，主导自研VMC技术路线
> **每周投入**: 5-10小时 | **总周期**: 约20周（5个月）

---

## 一、学习目标体系

### 终极目标
主导比亚迪易三方平台自研VMC的技术路线规划与核心算法开发。

### 分目标

| 层级 | 目标 | 衡量标准 |
|------|------|----------|
| **架构层** | 深刻理解大脑-小脑分层逻辑，能独立设计VMC在整车EEA中的定位方案 | 能画出完整的VMC系统架构图并论述各模块边界 |
| **算法层** | 掌握VMC核心算法：扭矩分配、横摆力矩控制、状态估计、MPC | 能从数学推导到代码实现全链路打通 |
| **工程层** | 理解VMC的量产标定、故障安全、传感器融合等工程问题 | 能评估现有方案的工程可行性并提出改进方案 |
| **战略层** | 对比行业方案，形成自研技术路线的判断力 | 能写出VMC技术路线论证报告 |

---

## 二、学习路线图（5阶段 × 4周）

```
阶段1: 车辆动力学基础 (Week 1-4)
  ├── 轮胎力学与轮胎模型 (Magic Formula, UniTire)
  ├── 车辆坐标系与运动学方程
  ├── 二自由度单轨模型 (Bicycle Model)
  └── 🔧 项目: Python搭建车辆2-DOF仿真模型

阶段2: VMC核心理论与算法 (Week 5-8)
  ├── VMC定义、架构与接口
  ├── 扭矩矢量控制 (Torque Vectoring)
  ├── 直接横摆力矩控制 (DYC)
  ├── 车辆稳定性判据 (β-β̇相图, 稳定域)
  └── 🔧 项目: 实现基础扭矩分配 + DYC控制器

阶段3: 大脑-小脑架构深度剖析 (Week 9-12)
  ├── 整车电子电气架构演进 (Distributed → Domain → Central)
  ├── 大脑 (智能驾驶域) 与小脑 (底盘/运动域) 的职责边界
  ├── 跨域通信 (SOME/IP, DDS, TSN) 与实时性要求
  ├── 行业方案横向对比 (Tesla, 比亚迪璇玑, Bosch, Huawei, NIO)
  └── 🔧 项目: 大脑-小脑架构设计文档 + 方案对比报告

阶段4: 高级控制与状态估计 (Week 13-16)
  ├── 卡尔曼滤波体系 (KF → EKF → UKF) 在车辆状态估计中的应用
  ├── 模型预测控制 (MPC) 基础理论
  ├── 线性时变MPC用于轨迹跟踪与稳定性控制
  ├── 质心侧偏角估计、轮胎力估计
  └── 🔧 项目: 实现EKF状态估计器 + MPC控制器

阶段5: 工程实践与集成 (Week 17-20)
  ├── 传感器融合 (IMU, 轮速, 方向盘转角, GPS/RTK)
  ├── VMC功能安全 (ISO 26262) 与故障降级策略
  ├── 标定方法与实车测试流程
  ├── 三电机分布式驱动的特殊问题 (易三方平台)
  └── 🔧 项目: 完整VMC仿真平台 + 三电机特有场景分析
```

---

## 三、每周详细计划

### 阶段1: 车辆动力学基础 (Week 1-4)

#### Week 1: 轮胎力学入门
- **理论** (3h): 轮胎坐标系 (SAE/ISO)、滑移率定义、轮胎力产生机理、摩擦椭圆
- **数学** (2h): Magic Formula (Pacejka) 公式推导与参数含义
- **实践** (2h): Python绘制轮胎力曲线（纯纵滑、纯侧偏、联合工况）
- **阅读**: 《Vehicle Dynamics and Control》(Rajamani) Ch.1-2 | 《汽车理论》(余志生) Ch.1-2

#### Week 2: 车辆运动学与坐标系
- **理论** (3h): 车辆坐标系定义、运动学约束、阿克曼转向几何
- **数学** (2h): 运动学方程推导 (位置、速度、加速度在车身坐标系下的表达)
- **实践** (2h): Python实现车辆运动学可视化
- **阅读**: 《车辆动力学及控制》(Rajamani 中译本) Ch.2 | 《汽车动力学》(Mitschke) Ch.1-3

#### Week 3: 二自由度单轨模型
- **理论** (3h): 侧偏角、横摆角速度、质心侧偏角定义与物理意义
- **数学** (2h): 二自由度状态空间方程推导、特征值分析、频率响应
- **实践** (2h): Python实现Bicycle Model，分析不足转向梯度
- **阅读**: Rajamani Ch.3 | 《汽车理论》Ch.5

#### Week 4: 动力学模型拓展 + 仿真平台搭建
- **理论** (2h): 从2-DOF到7-DOF/14-DOF车辆模型概览
- **数学** (2h): 考虑载荷转移的简化模型、侧倾运动
- **实践** (3h): 🔧 完整搭建Python车辆动力学仿真框架
- **输出**: 车辆模型Python库 (可复用于后续项目)

---

### 阶段2: VMC核心理论与算法 (Week 5-8)

#### Week 5: VMC概念与系统架构
- **理论** (3h): VMC的定义演变 (ESP → IVD → VMC → VDC → 智能VMC)
- **系统** (2h): VMC在整车控制器网络中的位置、信号流、接口定义
- **实践** (2h): 整理主流OEM的VMC架构图对比
- **阅读**: Bosch VDC系统资料 | 《车辆动力学控制系统》(喻凡)

#### Week 6: 扭矩矢量控制 (Torque Vectoring)
- **理论** (3h): 差动驱动的横摆力矩产生机理、前后轴扭矩分配策略
- **数学** (2h): 最优扭矩分配问题——含约束的二次规划 (QP)
- **实践** (2h): Python实现基于QP的扭矩分配算法
- **阅读**: "Torque Vectoring for Electric Vehicles" (De Novellis et al.)

#### Week 7: 直接横摆力矩控制 (DYC)
- **理论** (3h): DYC原理、参考模型生成 (Reference Model)、滑模控制基础
- **数学** (2h): 滑模控制器设计 (滑模面选取、趋近律设计)
- **实践** (2h): Simulink搭建DYC控制器 + 联合仿真验证
- **阅读**: "Direct Yaw Moment Control" (Shino & Nagai) | 《滑模控制理论》(刘金琨)

#### Week 8: 车辆稳定性判据
- **理论** (3h): β-β̇相平面分析、稳定域边界、李雅普诺夫稳定性
- **数学** (2h): 相平面绘制与稳定域计算方法
- **实践** (2h): 🔧 集成扭矩分配 + DYC + 稳定性监控的VMC仿真
- **输出**: 基础VMC控制器 Python/Simulink实现

---

### 阶段3: 大脑-小脑架构深度剖析 (Week 9-12)

#### Week 9: 整车EEA演进
- **理论** (4h): 分布式→功能域→跨域融合→中央计算的演进路径
- **关键概念**: ECU合并、域控制器、区域控制器 (Zonal)、SOA
- **实践** (3h): 绘制EEA演进时间线，标注关键车型
- **阅读**: Bosch EEA路线图 | 《智能网联汽车电子电气架构》(清华)

#### Week 10: 大脑 (AD域) 与小脑 (Chassis域) 的职责划分
- **理论** (4h): 大脑职责 (感知、规划、决策) vs 小脑职责 (执行、稳定、安全)
- **接口设计** (3h): 大脑→小脑的指令接口 (轨迹/目标横摆/扭矩请求)、小脑→大脑的状态反馈
- **关键问题**: 控制权限分配、延迟容忍、功能降级策略
- **阅读**: Tesla AI Day 资料 | Apollo VMC架构文档

#### Week 11: 跨域通信与实时性
- **理论** (3h): SOME/IP vs DDS vs TSN 协议特点与适用场景
- **VMC需求** (2h): VMC对通信延迟的要求 (<10ms? <1ms?)、确定性保证
- **实践** (2h): 整理主流协议的VMC适用性对比表
- **阅读**: AUTOSAR AP/CP 相关文档 | DDS标准简介

#### Week 12: 行业方案横向对比
- **调研** (5h): 深入调研6家方案
  - **Tesla**: FSD + 底盘域控制器 (CyberCab/V3架构)
  - **比亚迪**: 璇玑架构 (DiPilot + iTAC/云辇 + VMC)
  - **Bosch**: Vehicle Dynamic Control 2.0 + 线控底盘
  - **华为**: iDVP + 途灵底盘 + xMotion
  - **蔚来**: NIO ICC (智能底盘域控制器)
  - **采埃孚**: cubiX 底盘控制平台
- **输出**: 🔧 大脑-小脑架构方案对比报告 (含架构图、接口、优劣分析)

---

### 阶段4: 高级控制与状态估计 (Week 13-16)

#### Week 13: 卡尔曼滤波基础
- **数学** (4h): 从贝叶斯滤波到KF的推导、协方差传播、卡尔曼增益
- **实践** (3h): Python实现KF → EKF，用于车辆纵向速度估计
- **阅读**: 《最优状态估计》(Simon) Ch.1-5 | 《State Estimation for Robotics》(Barfoot)

#### Week 14: 车辆状态估计
- **理论** (3h): 质心侧偏角估计 (直接积分法 vs 运动学法 vs 模型法)
- **算法** (2h): EKF/UKF用于质心侧偏角 + 横摆角速度 + 轮胎力联合估计
- **实践** (2h): Python实现EKF/UKF车辆状态估计器
- **阅读**: "Vehicle Sideslip Angle Estimation" (Piyabongkarn)

#### Week 15: MPC基础理论
- **数学** (4h): 最优控制回顾 → 滚动时域优化 → 约束处理 → 线性MPC
- **关键概念**: 预测时域、控制时域、终端代价、递归可行性
- **实践** (3h): Python/Simulink实现简单线性MPC控制器
- **阅读**: 《Model Predictive Control》(Rawlings) Ch.1-3 | 《模型预测控制》(陈虹)

#### Week 16: MPC用于车辆控制
- **理论** (3h): 线性时变MPC (LTV-MPC) 用于轨迹跟踪
- **VMC应用** (2h): MPC框架下的集成控制 (转向+驱动+制动)
- **实践** (2h): 🔧 实现MPC-based VMC控制器，与阶段2控制器对比
- **输出**: MPC-VMC控制器 + 性能对比分析

---

### 阶段5: 工程实践与集成 (Week 17-20)

#### Week 17: 传感器融合
- **理论** (3h): IMU、轮速传感器、方向盘转角传感器的数据特性与误差模型
- **算法** (2h): 多传感器融合策略 (互补滤波、联邦滤波)
- **实践** (2h): Python实现传感器数据预处理 + 融合流水线

#### Week 18: 功能安全与故障处理
- **理论** (3h): ISO 26262 对VMC的要求、ASIL等级分配
- **设计** (2h): VMC故障模式分析 (FMEA)、降级策略设计
- **实践** (2h): 在VMC仿真中加入故障注入与降级逻辑

#### Week 19: 标定与测试
- **理论** (3h): VMC标定参数 (轮胎参数、车辆惯量、质心高度等) 的获取方法
- **流程** (2h): VIL (Vehicle-in-the-Loop) / HIL (Hardware-in-the-Loop) 测试方法
- **实践** (2h): 设计VMC标定与测试用例矩阵

#### Week 20: 三电机分布式驱动的特殊问题
- **理论** (3h): 三电机平台的冗余性、非对称扭矩分配、失效模式
- **创新** (3h): 易三方平台的特殊场景——坦克转向、蟹行、单电机失效补偿
- **输出**: 🔧 完整VMC仿真平台 + 三电机专属功能分析报告

---

## 四、推荐学习资料体系

### 必读教材（系统学习）

| 序号 | 书名 | 作者 | 关键章节 | 优先级 |
|------|------|------|----------|--------|
| 1 | 《Vehicle Dynamics and Control》 | Rajamani | Ch.1-6 | ⭐⭐⭐⭐⭐ |
| 2 | 《汽车理论》 | 余志生 | Ch.1,2,4,5 | ⭐⭐⭐⭐⭐ |
| 3 | 《车辆动力学及控制》(中译本) | Rajamani | 全书 | ⭐⭐⭐⭐⭐ |
| 4 | 《汽车动力学》(第5版) | Mitschke/陈荫三译 | Ch.1-4 | ⭐⭐⭐⭐ |
| 5 | 《Model Predictive Control》 | Rawlings/Mayne | Ch.1-3 | ⭐⭐⭐⭐ |
| 6 | 《模型预测控制》 | 陈虹 | 全书 | ⭐⭐⭐⭐ |

### 关键论文（按主题）

```
[轮胎模型]
  Pacejka H.B. "The Magic Formula Tyre Model" (2002)
  Guo K.H. "UniTire: Unified Tire Model" (2016)

[VMC基础]
  Shino M., Nagai M. "Yaw-moment control of electric vehicle 
    for improving handling and stability" (2001)
  De Novellis L. et al. "Torque Vectoring for Electric Vehicles: 
    A State-of-the-Art Review" (2015)

[状态估计]
  Piyabongkarn D. et al. "Development and Experimental Evaluation 
    of a Slip Angle Estimator for Vehicle Stability Control" (2009)
  Hrgetic M. et al. "Vehicle Sideslip Angle EKF Estimator based on
    Nonlinear Vehicle Dynamics Model" (2014)

[MPC与VMC]
  Falcone P. et al. "Predictive Active Steering Control for 
    Autonomous Vehicle Systems" (2007)
  Beal C.E., Gerdes J.C. "Model Predictive Control for Vehicle 
    Stabilization at the Limits of Handling" (2013)

[大脑-小脑/EEA]
  Bandur V. et al. "Making the Case for Centralized Automotive E/E 
    Architectures" (2021)
  Burkacky O. et al. "Rethinking Car Software and Electronics 
    Architecture" (McKinsey, 2018)

[三电机/分布式驱动]
  Zhao Y. et al. "Coordinated Control of Distributed Drive Electric 
    Vehicles" (2020)
```

### 行业报告与白皮书

- 各主机厂VMC/底盘域技术发布会材料
- 博世、大陆、采埃孚底盘控制白皮书
- 中国汽研/中汽中心 VMC测试评价标准研究

---

## 五、项目体系

### 项目1: 车辆动力学仿真平台 (Python)
**位置**: `项目/01-车辆模型仿真/`
**里程碑**:
- [x] Sprint 1: 轮胎模型 (Magic Formula)
- [x] Sprint 2: 2-DOF Bicycle Model
- [x] Sprint 3: 7-DOF 整车模型
- [x] Sprint 4: 可视化与仿真框架

### 项目2: VMC算法库 (Python + Simulink)
**位置**: `项目/02-VMC算法实现/`
**里程碑**:
- [x] Sprint 1: 参考模型生成器
- [x] Sprint 2: 扭矩分配 (QP/规则)
- [x] Sprint 3: DYC控制器 (滑模/MPC)
- [x] Sprint 4: 状态估计器 (EKF/UKF)

### 项目3: 大脑-小脑架构设计文档
**位置**: `项目/03-架构分析/`
**里程碑**:
- [x] Sprint 1: EEA演进梳理
- [x] Sprint 2: 行业方案架构图收集
- [x] Sprint 3: 自研VMC架构设计V1
- [x] Sprint 4: 接口定义与通信协议设计

### 项目4: VMC集成仿真平台
**位置**: `项目/04-集成仿真/`
**里程碑**:
- [x] Sprint 1: 场景定义 (双移线、蛇形、定圆、对接路面)
- [x] Sprint 2: 控制器集成与对比框架
- [x] Sprint 3: 三电机特殊场景仿真
- [x] Sprint 4: 性能指标计算与可视化

---

## 六、每周学习节奏建议

```
周一/二 (1-2h): 理论学习 + 阅读教材
周三/四 (1-2h): 数学推导 + 算法理解
周末 (3-6h):  实践项目 + 论文精读 + 知识库整理
```

> **流体力学博士的数学优势提醒**:
> - 状态空间、特征值分析、频率响应对你来说会很自然
> - 最优控制 (LQR) 可类比流体的变分原理
> - 卡尔曼滤波本质是贝叶斯推断——流体力学中的数据同化有类似思想
> - 重点攻克的是**车辆动力学特有的物理直觉**（不足转向、侧偏刚度、摩擦椭圆等）
> - MPC中的约束处理是你的新内容，但优化理论基础好的话上手很快

---

## 七、检查点 (Checkpoints)

| 时间 | 检查内容 | 交付物 |
|------|----------|--------|
| Week 4 | 能否独立推导2-DOF模型并解释不足转向梯度？ | Python车辆模型库 |
| Week 8 | 能否解释扭矩分配QP问题的约束含义？ | 基础VMC控制器 |
| Week 12 | 能否画出自研VMC架构图并说明各模块边界？ | 大脑-小脑架构报告 |
| Week 16 | 能否比较EKF与MPC在VMC中的角色差异？ | EKF+MPC实现 |
| Week 20 | 能否向领导汇报自研VMC技术路线？ | 完整仿真平台+技术路线报告 |
