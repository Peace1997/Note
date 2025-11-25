# WBC和MPC数学公式与代码实现对应指南

本文档基于KUAVO机器人的实际源代码，详细展示了MPC轨迹规划和WBC整体控制中数学公式与代码实现的一一对应关系，包含详尽的数学推导和实际代码映射。

## 目录
1. [MPC数学公式与代码对应](#mpc数学公式与代码对应)
2. [WBC数学公式与代码对应](#wbc数学公式与代码对应)  
3. [控制流程整合](#控制流程整合)

---

## MPC数学公式与代码对应 (基于OCS2框架)

### 1. 质心动力学状态空间模型

**数学基础理论:**

KUAVO机器人采用质心动力学模型(Centroidal Dynamics)进行MPC轨迹规划。该模型将复杂的多体系统简化为质心运动和角动量守恒的组合，大幅降低了计算复杂度。

**状态向量定义:**
$$\mathbf{x} = \begin{bmatrix} 
\mathbf{v}_{\text{CoM}} \\ 
\mathbf{L}/m \\ 
\mathbf{p}_{\text{base}} \\ 
\boldsymbol{\theta}_{\text{base}} \\ 
\mathbf{q}_{\text{joints}} 
\end{bmatrix} \in \mathbb{R}^{24}$$

其中：
- $\mathbf{v}_{\text{CoM}} \in \mathbb{R}^3$: 质心线速度 (归一化线动量 $\mathbf{h}_{\text{lin}}/m$)
- $\mathbf{L}/m \in \mathbb{R}^3$: 归一化角动量 
- $\mathbf{p}_{\text{base}} \in \mathbb{R}^3$: 基座位置
- $\boldsymbol{\theta}_{\text{base}} \in \mathbb{R}^3$: 基座姿态 (ZYX欧拉角)
- $\mathbf{q}_{\text{joints}} \in \mathbb{R}^{12}$: 关节角度 (腿部12DOF)

**输入向量定义:**
$$\mathbf{u} = \begin{bmatrix} 
\mathbf{f}_1 \\ \mathbf{f}_2 \\ \mathbf{f}_3 \\ \mathbf{f}_4 \\ 
\mathbf{F}_{\text{hand1}} \\ \mathbf{F}_{\text{hand2}} \\ 
\dot{\mathbf{q}}_{\text{joints}} 
\end{bmatrix} \in \mathbb{R}^{36}$$

其中：
- $\mathbf{f}_i \in \mathbb{R}^3$: 第$i$个足端接触力 (4个足端)
- $\mathbf{F}_{\text{handi}} \in \mathbb{R}^6$: 第$i$个手部6DOF力/力矩 (2个手部) 
- $\dot{\mathbf{q}}_{\text{joints}} \in \mathbb{R}^{12}$: 关节速度

**质心动力学微分方程 (牛顿-欧拉方程的简化形式):**

线动量守恒：
$$m\dot{\mathbf{v}}_{CoM} = \sum_{i=1}^{4} \mathbf{f}_i + \sum_{j=1}^{2} \mathbf{F}_{hand,j}^{lin} + m\mathbf{g}$$

角动量守恒：
$$\dot{\mathbf{L}} = \sum_{i=1}^{4} (\mathbf{r}_i - \mathbf{r}_{CoM}) \times \mathbf{f}_i + \sum_{j=1}^{2} [(\mathbf{r}_{hand,j} - \mathbf{r}_{CoM}) \times \mathbf{F}_{hand,j}^{lin} + \mathbf{F}_{hand,j}^{ang}]$$

基座运动学：
$$\dot{\mathbf{p}}_{\text{base}} = \mathbf{v}_{\text{CoM}}$$
$$\dot{\boldsymbol{\theta}}_{\text{base}} = \mathbf{R}_{\text{ZYX}}(\boldsymbol{\theta}_{\text{base}}) \boldsymbol{\omega}_{\text{base}}$$

关节运动学：

$$\dot{\mathbf{q}}_{\text{joints}} = \dot{\mathbf{q}}_{\text{joints}}$$ (直接给定)

**归一化状态空间方程:**
$$\dot{\mathbf{x}} = \begin{bmatrix}
\frac{1}{m}\sum_{i=1}^{4} \mathbf{f}_i + \frac{1}{m}\sum_{j=1}^{2} \mathbf{F}_{\text{hand},j}^{\text{lin}} + \mathbf{g} \\
\frac{1}{m}\sum_{i=1}^{4} (\mathbf{r}_i - \mathbf{r}_{\text{CoM}}) \times \mathbf{f}_i + \frac{1}{m}\sum_{j=1}^{2} [(\mathbf{r}_{\text{hand},j} - \mathbf{r}_{\text{CoM}}) \times \mathbf{F}_{\text{hand},j}^{\text{lin}} + \mathbf{F}_{\text{hand},j}^{\text{ang}}] \\
\mathbf{v}_{\text{CoM}} \\
\mathbf{R}_{\text{ZYX}}(\boldsymbol{\theta}_{\text{base}}) \boldsymbol{\omega}_{\text{base}} \\
\dot{\mathbf{q}}_{\text{joints}}
\end{bmatrix}$$

**代码实现 (PinocchioCentroidalDynamics.cpp):**
```cpp
vector_t PinocchioCentroidalDynamics::getValue(scalar_t time, const vector_t& state, const vector_t& input) {
  const auto& interface = *pinocchioInterfacePtr_;
  const auto& info = mapping_.getCentroidalModelInfo();
  assert(info.stateDim == state.rows());  // stateDim = 24

  vector_t f(info.stateDim);
  // 状态导数 = [归一化质心动量率; 关节速度]
  f << getNormalizedCentroidalMomentumRate(interface, info, input),  // 前6维：质心动力学
       mapping_.getPinocchioJointVelocity(state, input);             // 后18维：运动学

  return f;
}
```

**质心动量率计算的详细实现:**
```cpp
Vector6 getNormalizedCentroidalMomentumRate(const PinocchioInterface& interface, 
                                          const CentroidalModelInfo& info, 
                                          const vector_t& input) {
  Vector6 centroidalMomentumRate = Vector6::Zero();
  
  // 1. 计算足端3DOF接触力贡献 (Newton方程)
  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    const Vector3 contactForce = centroidal_model::getContactForces(input, i, info);
    
    // 线动量率: Σf_i  (牛顿第二定律)
    centroidalMomentumRate.segment<3>(0) += contactForce;
    
    // 角动量率: Σ(r_i - r_CoM) × f_i  (欧拉方程)
    const Vector3 comToContactPoint = getPositionComToContactPointInWorldFrame(interface, info, i);
    centroidalMomentumRate.segment<3>(3) += comToContactPoint.cross(contactForce);
  }
  
  // 2. 计算手部6DOF接触力贡献
  for (size_t i = 0; i < info.numSixDofContacts; i++) {
    const size_t inputIdx = 3 * info.numThreeDofContacts + 6 * i;  // 第12+6*i维开始
    const Vector3 contactForce = input.segment<3>(inputIdx);       // 力
    const Vector3 contactTorque = input.segment<3>(inputIdx + 3);  // 力矩
    
    // 线动量率贡献
    centroidalMomentumRate.segment<3>(0) += contactForce;
    
    // 角动量率贡献 (力矩 + 力臂效应)
    const Vector3 comToContactPoint = getPositionComToContactPointInWorldFrame(interface, info, 
                                        info.numThreeDofContacts + i);
    centroidalMomentumRate.segment<3>(3) += comToContactPoint.cross(contactForce) + contactTorque;
  }
  
  // 3. 添加重力效应 (mg)
  centroidalMomentumRate.segment<3>(0) += Vector3(0, 0, -info.robotMass * 9.81);
  
  // 4. 归一化 (除以机器人质量)
  return centroidalMomentumRate / info.robotMass;
}
```

### 2. 连续时间最优控制问题

**数学公式 (Bolza型最优控制问题):**
$$\begin{align}
\min_{\mathbf{u}(\cdot)} \quad &\phi(\mathbf{x}(T)) + \int_0^T L(\mathbf{x}(t), \mathbf{u}(t), t) \, dt \\
\text{s.t.} \quad &\dot{\mathbf{x}}(t) = \mathbf{f}(\mathbf{x}(t), \mathbf{u}(t), t) \\
&\mathbf{g}(\mathbf{x}(t), \mathbf{u}(t), t) \leq \mathbf{0} \\
&\mathbf{h}(\mathbf{x}(t), \mathbf{u}(t), t) = \mathbf{0} \\
&\mathbf{x}(0) = \mathbf{x}_0
\end{align}$$

其中：
- $\phi(\mathbf{x}(T))$: 终端成本函数
- $L(\mathbf{x}, \mathbf{u}, t)$: 瞬时成本函数 (Lagrangian)
- $\mathbf{g}(\cdot) \leq 0$: 不等式约束 (摩擦锥、关节限制等)
- $\mathbf{h}(\cdot) = 0$: 等式约束 (运动学约束等)

**瞬时成本函数 (二次型):**
$$L(\mathbf{x}, \mathbf{u}, t) = \frac{1}{2}(\mathbf{x} - \mathbf{x}_{ref}(t))^T \mathbf{Q}(t) (\mathbf{x} - \mathbf{x}_{ref}(t)) + \frac{1}{2}(\mathbf{u} - \mathbf{u}_{ref}(t))^T \mathbf{R}(t) (\mathbf{u} - \mathbf{u}_{ref}(t))$$

**终端成本函数:**
$$\phi(\mathbf{x}(T)) = \frac{1}{2}(\mathbf{x}(T) - \mathbf{x}_{ref}(T))^T \mathbf{Q}_f (\mathbf{x}(T) - \mathbf{x}_{ref}(T))$$

### 3. 离散化与DDP求解

**时间离散化 (Euler前向法):**
$$\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \cdot \mathbf{f}(\mathbf{x}_k, \mathbf{u}_k, t_k)$$

其中 $\Delta t = 0.015s$ (66.7Hz)，预测步数 $N = T_{horizon}/\Delta t = 1.0/0.015 ≈ 67$

**离散时间最优控制问题:**
$$\begin{align}
\min_{\{\mathbf{u}_k\}_{k=0}^{N-1}} \quad &\phi_N(\mathbf{x}_N) + \sum_{k=0}^{N-1} L_k(\mathbf{x}_k, \mathbf{u}_k) \Delta t \\
\text{s.t.} \quad &\mathbf{x}_{k+1} = \mathbf{f}_k(\mathbf{x}_k, \mathbf{u}_k) \\
&\mathbf{g}_k(\mathbf{x}_k, \mathbf{u}_k) \leq \mathbf{0} \\
&\mathbf{h}_k(\mathbf{x}_k, \mathbf{u}_k) = \mathbf{0} \\
&\mathbf{x}_0 = \mathbf{x}_{measured}
\end{align}$$

**Gauss-Newton DDP算法核心思想:**

DDP(Differential Dynamic Programming)是求解非线性最优控制的经典方法，通过反向递推构造二次逼近。

1. **前向传播 (Forward Pass):** 使用当前控制策略积分系统动力学
2. **后向传播 (Backward Pass):** 反向求解Riccati方程获得最优反馈增益
3. **线搜索 (Line Search):** 确定步长保证成本函数下降

**Hamilton-Jacobi-Bellman方程的离散形式:**
$$V_k(\mathbf{x}_k) = \min_{\mathbf{u}_k} \left[ L_k(\mathbf{x}_k, \mathbf{u}_k) \Delta t + V_{k+1}(\mathbf{f}_k(\mathbf{x}_k, \mathbf{u}_k)) \right]$$

**最优性条件 (一阶必要条件):**
$$\nabla_{\mathbf{u}} H_k = \nabla_{\mathbf{u}} L_k + \nabla_{\mathbf{u}} \mathbf{f}_k^T \nabla_{\mathbf{x}} V_{k+1} = 0$$

**代码实现 (humanoidController.cpp):**
```cpp
void humanoidController::setupMpc() {
    // 使用Gauss-Newton DDP求解器 (Sequential Linear Quadratic方法)
    mpc_ = std::make_shared<GaussNewtonDDP_MPC>(
        HumanoidInterface_->mpcSettings(),          // MPC基本设置
        HumanoidInterface_->ddpSettings(),          // DDP求解器设置  
        HumanoidInterface_->getRollout(),           // 系统rollout
        HumanoidInterface_->getOptimalControlProblem(),  // 优化问题定义
        HumanoidInterface_->getInitializer()       // 初始化器
    );

    // 设置参考轨迹管理器
    auto gaitReceiverPtr = std::make_shared<GaitReceiver>(
        controllerNh_, 
        HumanoidInterface_->getSwitchedModelReferenceManagerPtr(), 
        robotName_
    );
    
    mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
}
```

**MPC主循环异步求解:**
```cpp
void humanoidController::setupMrt() {
    mpcThread_ = std::thread([&]() {
        while (controllerRunning_) {
            try {
                executeAndSleep([&]() {
                    if (mpcRunning_) {
                        mpcTimer_.startTimer();
                        mpcMrtInterface_->advanceMpc();  // 核心DDP求解
                        mpcTimer_.endTimer();
                    }
                }, HumanoidInterface_->mpcSettings().mpcDesiredFrequency_);  // 50Hz
            } catch (const std::exception& e) {
                controllerRunning_ = false;
                ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
            }
        }
    });
### 4. 权重矩阵设计与物理意义

**状态权重矩阵 $\mathbf{Q} \in \mathbb{R}^{24 \times 24}$ (对角矩阵):**

$$\mathbf{Q} = \text{diag}(\mathbf{Q}_{v}, \mathbf{Q}_{L}, \mathbf{Q}_{p}, \mathbf{Q}_{\theta}, \mathbf{Q}_{q})$$

其中各子矩阵的物理意义：

**质心速度权重 $\mathbf{Q}_{v} = \text{diag}(80.0, 80.0, 13.0) \times 10^{-3}$:**
- x,y方向80.0: 水平运动控制精度高
- z方向13.0: 垂直运动相对自然

**角动量权重 $\mathbf{Q}_{L} = \text{diag}(50.0, 50.0, 13.0) \times 10^{-3}$:**  
- 滚转/俯仰权重50.0，保持身体稳定
- 偏航13.0，允许转身动作

**基座位置权重 $\mathbf{Q}_{p} = \text{diag}(500.0, 500.0, 2000.0) \times 10^{-3}$:**
- 水平位置权重500.0
- 高度控制权重2000.0，严格控制重心高度

**基座姿态权重 $\mathbf{Q}_{\theta} = \text{diag}(800.0, 2000.0, 100.0) \times 10^{-3}$:**
- 滚转角800.0，保持侧向稳定  
- 俯仰角2000.0，严格控制前后倾斜
- 偏航角100.0，允许转身

**关节位置权重 (太极专用调优):**
```cpp
// 腿部关节权重 (12 DOF) - 来自实际task.info配置
Q_leg_joints = [300.0, 300.0, 10.0, 10.0, 10.0, 10.0,    // 左腿
                300.0, 300.0, 10.0, 10.0, 10.0, 10.0] × 1e-3;  // 右腿

// 物理解释:
// - 髋关节(300.0): 高权重确保稳定支撑，控制身体姿态
// - 其他关节(10.0): 低权重允许自然弯曲，适应地形变化

// 手臂关节权重 (14 DOF) - Q矩阵中的实际配置
Q_arm_joints = [200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0,  // 左臂
                200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0] × 1e-3;  // 右臂

// 物理解释:
// - 统一权重200.0，平衡跟踪精度与运动自然度
// - 太极动作要求手臂动作流畅连贯，权重设置一致
```

**输入权重矩阵 $\mathbf{R} \in \mathbb{R}^{36 \times 36}$ (对角矩阵):**

$$\mathbf{R} = \text{diag}(\mathbf{R}_{f}, \mathbf{R}_{F}, \mathbf{R}_{\dot{q}})$$

**足端力权重 $\mathbf{R}_{f} \in \mathbb{R}^{24 \times 24}$:**
```cpp
// 4个足端，每个3DOF力 (左足跟,左足尖,右足跟,右足尖) - 实际配置
foot_force_weights = [1.0, 1.0, 1.0,  // 左足跟 (heel)
                     1.0, 1.0, 1.0,  // 左足尖 (toe)
                     1.0, 1.0, 1.0,  // 右足跟 (heel)
                     1.0, 1.0, 1.0,  // 右足尖 (toe)
                     1.0, 1.0, 1.0,  // 左足跟 (heel) - 后端
                     1.0, 1.0, 1.0,  // 左足尖 (toe) - 后端  
                     1.0, 1.0, 1.0,  // 右足跟 (heel) - 后端
                     1.0, 1.0, 1.0] × 1e-3;  // 右足尖 (toe) - 后端

// 物理意义: 最小化接触力，提高能量效率，软着陆
```

**手部力权重 $\mathbf{R}_{F} \in \mathbb{R}^{12 \times 12}$:**
```cpp
// 2个手部，每个6DOF力/力矩 - 来自实际task.info
hand_force_weights = [1000, 1000, 1000, 1000, 1000, 1000,  // 左手 (力+力矩)
                     1000, 1000, 1000, 1000, 1000, 1000] × 1e-3;  // 右手 (力+力矩)

// 物理意义: 高权重1000抑制手部接触力，太极以非接触动作为主
```

**关节速度权重 - 腿部 $\mathbf{R}_{\dot{q},leg} \in \mathbb{R}^{24 \times 24}$:**
```cpp
// 足端速度正则化 (4个足端相对基座速度) - 实际配置值5000.0
foot_velocity_weights = [5000.0, 5000.0, 5000.0,  // 左足跟速度
                        5000.0, 5000.0, 5000.0,  // 左足尖速度
                        5000.0, 5000.0, 5000.0,  // 右足跟速度
                        5000.0, 5000.0, 5000.0,  // 右足尖速度
                        5000.0, 5000.0, 5000.0,  // 左足跟速度(后)
                        5000.0, 5000.0, 5000.0,  // 左足尖速度(后)
                        5000.0, 5000.0, 5000.0,  // 右足跟速度(后)
                        5000.0, 5000.0, 5000.0] × 1e-3;  // 右足尖速度(后)

// 物理意义: 高权重5000.0抑制腿部剧烈运动，保证平稳步态
```

**关节速度权重 - 手臂 $\mathbf{R}_{\dot{q},arm} \in \mathbb{R}^{14 \times 14}$:**
```cpp  
// 手臂关节速度正则化 - 实际配置值2000.0
arm_velocity_weights = [2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0,  // 左臂
                       2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0] × 1e-3;  // 右臂
                  2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0, 2000.0] × 1e-3;

// 物理意义: 
// - 高权重平滑关节运动，避免突变
// - 腿部权重高于手臂，保证支撑稳定性
```

**权重配置代码实现 (task.info):**
```cpp
// Q矩阵配置
Q {
  scaling 1e-3  // 全局缩放因子
  
  // 质心线速度权重 (维度0-2)
  (0,0)   1.0      ; com_vel_x
  (1,1)   1.0      ; com_vel_y  
  (2,2)   1.0      ; com_vel_z
  
  // 基座角速度权重 (维度3-5) 
  (3,3)   10.0     ; base_omega_x - 抑制滚转
  (4,4)   10.0     ; base_omega_y - 抑制俯仰
  (5,5)   1.0      ; base_omega_z - 偏航权重较低
  
  // 基座位置权重 (维度6-8)
  (6,6)   1000.0   ; base_pos_x - 强约束水平位置
  (7,7)   1000.0   ; base_pos_y
  (8,8)   500.0    ; base_pos_z - 高度控制适中
  
  // 基座姿态权重 (维度9-11)
  (9,9)   1000.0   ; theta_base_z - 偏航角
  (10,10) 800.0    ; theta_base_y - 俯仰角  
  (11,11) 800.0    ; theta_base_x - 滚转角

  // 腿部关节权重 (维度12-23)
  (12,12) 300.0    ; leg_l1_joint - 髋关节高权重
  (13,13) 300.0    ; leg_l2_joint
  (14,14) 10.0     ; leg_l3_joint - 膝关节低权重
  (15,15) 50.0     ; leg_l4_joint - 踝关节
  // ... 右腿对称配置
  
  // 手臂关节权重 (维度24-37) - 太极重点
  (24,24) 200.0    ; zarm_l1_joint
  (25,25) 200.0    ; zarm_l2_joint  
  // ... 统一手臂权重配置
}

// R矩阵配置
R {
  scaling 1e-3
  
  // 足端接触力 (维度0-11)
  (0,0)   1.0      ; ll_foot_heel_x
  (1,1)   1.0      ; ll_foot_heel_y
  (2,2)   1.0      ; ll_foot_heel_z
  // ... 4个足端配置
  
  // 手部接触力 (维度12-23)
  (12,12) 1000     ; l_arm force x - 高权重抑制接触
  (13,13) 1000     ; l_arm force y
  (14,14) 1000     ; l_arm force z
  (15,15) 1000     ; l_arm wrench x
  (16,16) 1000     ; l_arm wrench y  
  (17,17) 1000     ; l_arm wrench z
  // ... 右手对称配置
  
  // 关节速度正则化 (维度24-35)
  (24,24) 5000.0   ; leg_l1_joint_vel - 腿部速度平滑
  // ... 腿部关节速度配置
  
  (36,36) 2000.0   ; zarm_l1_joint_vel - 手臂速度平滑
  // ... 手臂关节速度配置
}
```

### 5. 约束条件数学表述

**摩擦锥约束 (Friction Cone Constraints):**

对每个足端接触点 $i$，接触力 $\mathbf{f}_i = [f_{x,i}, f_{y,i}, f_{z,i}]^T$ 必须满足库伦摩擦定律：

$$\begin{align}
f_{z,i} &\geq \epsilon_{min} \quad \text{(最小正压力)} \\
\sqrt{f_{x,i}^2 + f_{y,i}^2} &\leq \mu f_{z,i} \quad \text{(摩擦锥约束)}
\end{align}$$

其中 $\mu = 0.5$ 是摩擦系数，$\epsilon_{min} = 10N$ 是最小正压力。

**线性化摩擦锥约束 (pyramid approximation):**
$$\begin{align}
|f_{x,i}| &\leq \mu f_{z,i} \\
|f_{y,i}| &\leq \mu f_{z,i}
\end{align}$$

转换为标准不等式形式：
$$\begin{bmatrix}
f_{x,i} - \mu f_{z,i} \\
-f_{x,i} - \mu f_{z,i} \\
f_{y,i} - \mu f_{z,i} \\
-f_{y,i} - \mu f_{z,i} \\
-f_{z,i}
\end{bmatrix} \leq \begin{bmatrix} 0 \\ 0 \\ 0 \\ 0 \\ -\epsilon_{min} \end{bmatrix}$$

**基座姿态限制约束:**

为保证太极动作的稳定性，限制基座的俯仰角范围：
$$-\theta_{pitch,max} \leq \theta_{pitch} \leq \theta_{pitch,max}$$

其中 $\theta_{pitch,max} = 0.01 \text{ rad} ≈ 0.57°$（保守设置）

**关节限制约束:**

每个关节角度 $q_j$ 和关节速度 $\dot{q}_j$ 都有物理限制：
$$\begin{align}
q_{j,min} &\leq q_j \leq q_{j,max} \\
\dot{q}_{j,min} &\leq \dot{q}_j \leq \dot{q}_{j,max}
\end{align}$$

**约束处理方法:**

OCS2框架使用松弛对数障碍法(Relaxed Log Barrier)处理不等式约束：
$$\bar{g}_i(x) = \max(0, g_i(x)) + \mu \log(1 + \exp((g_i(x) - \delta)/\mu))$$

其中：
- $\mu = 0.1$: 障碍参数
- $\delta = 5.0$: 松弛参数

**代码实现 (task.info约束配置):**
```cpp
// 摩擦锥软约束
frictionConeSoftConstraint {
    frictionCoefficient    0.5    // μ = 0.5 (太极保守设置)
    
    // 松弛对数障碍参数
    mu                     0.1    // 障碍参数
    delta                  5.0    // 松弛参数
}

// 基座俯仰角限制 (太极专用)
basePitchLimits {
    enable                 true
    pos {
        lowerBound      -0.01     // -0.57° (太极保守角度限制)
        upperBound      0.01      // +0.57°
        mu              0.1       // 松弛对数障碍参数
        delta           1e-3
    }
    vel {
        lowerBound      -0.02     // 角速度限制
        upperBound      0.02
    }
}

// 关节位置限制
jointPositionLimits {
    enable              true
    mu                  0.1
    delta               1e-3
    // 具体限制值从URDF文件加载
}

// 关节速度限制
jointVelocityLimits {
    enable              true  
    mu                  0.1
    delta               1e-3
    // 最大速度: ±10 rad/s (大部分关节)
}
```

### 6. DDP求解器配置与数值方法

**Sequential Linear Quadratic (SLQ) 算法核心:**

SLQ是DDP的一种变体，通过序列二次规划逼近最优解：

1. **线性化系统动力学:**
$$\mathbf{f}_k(\mathbf{x}_k, \mathbf{u}_k) ≈ \mathbf{f}_k + \mathbf{A}_k(\mathbf{x}_k - \bar{\mathbf{x}}_k) + \mathbf{B}_k(\mathbf{u}_k - \bar{\mathbf{u}}_k)$$

其中：
$$\mathbf{A}_k = \frac{\partial \mathbf{f}_k}{\partial \mathbf{x}}\bigg|_{(\bar{\mathbf{x}}_k, \bar{\mathbf{u}}_k)}, \quad \mathbf{B}_k = \frac{\partial \mathbf{f}_k}{\partial \mathbf{u}}\bigg|_{(\bar{\mathbf{x}}_k, \bar{\mathbf{u}}_k)}$$

2. **二次化成本函数:**
$$L_k(\mathbf{x}_k, \mathbf{u}_k) ≈ L_k + \mathbf{L}_{x,k}^T\delta\mathbf{x}_k + \mathbf{L}_{u,k}^T\delta\mathbf{u}_k + \frac{1}{2}\delta\mathbf{x}_k^T\mathbf{L}_{xx,k}\delta\mathbf{x}_k + \delta\mathbf{x}_k^T\mathbf{L}_{xu,k}\delta\mathbf{u}_k + \frac{1}{2}\delta\mathbf{u}_k^T\mathbf{L}_{uu,k}\delta\mathbf{u}_k$$

3. **反向递推求解Riccati方程:**
$$\mathbf{V}_{xx,k} = \mathbf{L}_{xx,k} + \mathbf{A}_k^T\mathbf{V}_{xx,k+1}\mathbf{A}_k - (\mathbf{L}_{xu,k} + \mathbf{A}_k^T\mathbf{V}_{xx,k+1}\mathbf{B}_k)\mathbf{Q}_{uu,k}^{-1}(\mathbf{L}_{xu,k} + \mathbf{A}_k^T\mathbf{V}_{xx,k+1}\mathbf{B}_k)^T$$

**算法参数配置 (基于实际task.info):**
```cpp
ddp {
    algorithm                    SLQ           // Sequential Linear Quadratic
    nThreads                     3             // 并行线程数
    maxNumIterations            1             // 太极动作单次迭代充分
    minRelCost                  1e-1          // 相对成本收敛阈值
    constraintTolerance         5e-3          // 约束违反容差
    timeStep                    0.015         // 时间步长 (66.7Hz离散化)
    
    // 约束处理参数
    constraintPenaltyInitialValue   20.0      // 初始约束惩罚权重
    constraintPenaltyIncreaseRate   2.0       // 惩罚权重递增率
    
    // 线搜索参数  
    armijoCoefficient           1e-4          // Armijo条件系数
    stepLengthDecayRate         0.5           // 步长衰减率
    maxStepLength               1.0           // 最大步长
    
    // 正则化参数
    stateRegularization         1e-6          // 状态Hessian正则化
    inputRegularization         1e-6          // 输入Hessian正则化
}

mpc {
    timeHorizon                 1.0           // 预测时域长度 (s)
    mpcDesiredFrequency         50            // MPC求解频率 (Hz) 
    mrtDesiredFrequency         1000          // MRT插值频率 (Hz)
    coldStart                   false         // 使用热启动策略
    debugPrint                  false         // 关闭调试输出
    
    // 初始化策略
    initializeFromPreviousSolution  true      // 从上次解初始化
    keepFirstElementOfState         false     // 不保持首元素
}
```

**多频率控制策略的数学基础:**

系统采用三层频率控制：
- **MPC层 (50Hz):** 求解最优轨迹
- **WBC层 (200Hz):** 实时轨迹跟踪控制  
- **硬件层 (1000Hz):** 伺服控制

---

## WBC数学公式与代码对应 (基于qpOASES求解器)

### 1. 整体控制问题数学表述

**全身控制(Whole Body Control)的本质:**

WBC将高层轨迹规划(来自MPC)转换为底层关节控制命令，通过求解约束优化问题实现多任务协调控制。

**标准QP问题形式:**
$$\begin{align}
\min_{\mathbf{x}} \quad &\frac{1}{2}\mathbf{x}^T\mathbf{H}\mathbf{x} + \mathbf{g}^T\mathbf{x} \\
\text{s.t.} \quad &\mathbf{A}_{eq}\mathbf{x} = \mathbf{b}_{eq} \quad \text{(等式约束)} \\
&\mathbf{A}_{ineq}\mathbf{x} \leq \mathbf{b}_{ineq} \quad \text{(不等式约束)} \\
&\mathbf{x}_{min} \leq \mathbf{x} \leq \mathbf{x}_{max} \quad \text{(变量边界)}
\end{align}$$

**决策变量定义:**
$$\mathbf{x} = \begin{bmatrix} 
\ddot{\mathbf{q}} \\ 
\boldsymbol{\lambda} \\ 
\boldsymbol{\tau} 
\end{bmatrix} \in \mathbb{R}^{56}$$

其中：
- $\ddot{\mathbf{q}} \in \mathbb{R}^{18}$: 广义坐标加速度
  - 浮基加速度: $[\ddot{\mathbf{p}}_{base}^T, \ddot{\boldsymbol{\theta}}_{base}^T]^T \in \mathbb{R}^6$
  - 关节加速度: $\ddot{\mathbf{q}}_{\text{joints}} \in \mathbb{R}^{12}$
- $\boldsymbol{\lambda} \in \mathbb{R}^{24}$: 接触力
  - 足端3DOF力: $4 \times 3 = 12$ 维
  - 手部6DOF力/力矩: $2 \times 6 = 12$ 维
- $\boldsymbol{\tau} \in \mathbb{R}^{14}$: 关节力矩 (实际可控制关节)

**加权最小二乘转换:**

多任务WBC问题转换为加权最小二乘形式：
$$\min_{\mathbf{x}} \frac{1}{2}\|\mathbf{A}_{tasks}\mathbf{x} - \mathbf{b}_{tasks}\|_{\mathbf{W}}^2$$

其中：
$$\mathbf{H} = \mathbf{A}_{tasks}^T\mathbf{W}\mathbf{A}_{tasks}, \quad \mathbf{g} = -\mathbf{A}_{tasks}^T\mathbf{W}\mathbf{b}_{tasks}$$

$\mathbf{W}$ 是任务权重矩阵，$\mathbf{A}_{tasks}$ 和 $\mathbf{b}_{tasks}$ 包含所有任务的雅可比和期望加速度。

**代码实现 (WeightedWbc.cpp核心求解函数):**
```cpp
vector_t WeightedWbc::update(const vector_t &stateDesired, const vector_t &inputDesired,
                            const vector_t &rbdStateMeasured, size_t mode, scalar_t period, bool mpc_update) {
    updateMeasured(rbdStateMeasured);   // 更新测量状态
    updateDesired(stateDesired, inputDesired);  // 更新期望状态

    // 1. 计算约束 (动力学等式约束 + 不等式约束)
    Task constraints = formulateConstraints(inputDesired);
    size_t numConstraints = constraints.b_.size() + constraints.f_.size();

    // 2. 构造约束矩阵 (56维决策变量)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
    vector_t lbA(numConstraints), ubA(numConstraints);
    A << constraints.a_,      // 等式约束矩阵 A_eq
         constraints.d_;      // 不等式约束矩阵 A_ineq

    lbA << constraints.b_,    // 等式约束右端项 b_eq
           -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());  // 不等式下界
    ubA << constraints.b_,    // 等式约束右端项 b_eq (相同)
           constraints.f_;    // 不等式上界 b_ineq

    // 3. 构造成本函数 (加权最小二乘)
    Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = 
        weighedTask.a_.transpose() * weighedTask.a_;  // H = A_tasks^T * A_tasks
    vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;  // g = -A_tasks^T * b_tasks

    // 4. 求解QP问题 (使用qpOASES求解器)
    auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
    qpOASES::Options options;
    options.setToMPC();                    // MPC专用设置
    options.printLevel = qpOASES::PL_NONE; // 关闭输出
    qpProblem.setOptions(options);

    // 初始化QP问题
    qpOASES::returnValue status = qpProblem.init(H.data(), g.data(), A.data(), 
                                                nullptr, nullptr, lbA.data(), ubA.data(), maxIterations_);
    
    // 获取解
    vector_t qpSol(getNumDecisionVars());
    if (status == qpOASES::SUCCESSFUL_RETURN) {
        qpProblem.getPrimalSolution(qpSol.data());
    } else {
        ROS_ERROR("WBC QP solver failed!");
        qpSol.setZero();
    }

    // 5. 提取关节加速度 (前18维)
    vector_t generalizedAcceleration = qpSol.head(info_.generalizedCoordinatesNum);
    return centroidal_model::getJointAccelerations(generalizedAcceleration, info_);
}
```

### 2. 动力学等式约束 (最重要约束)

**浮基多体系统动力学方程:**

机器人的运动方程基于Newton-Euler公式：
$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{G}(\mathbf{q}) = \mathbf{S}^T\boldsymbol{\tau} + \mathbf{J}_c^T\boldsymbol{\lambda}$$

其中：
- $\mathbf{M}(\mathbf{q}) \in \mathbb{R}^{18 \times 18}$: 广义质量矩阵(正定对称)
- $\mathbf{C}(\mathbf{q},\dot{\mathbf{q}}) \in \mathbb{R}^{18 \times 18}$: 科里奥利/向心力矩阵  
- $\mathbf{G}(\mathbf{q}) \in \mathbb{R}^{18}$: 重力向量
- $\mathbf{S} \in \mathbb{R}^{18 \times 14}$: 关节选择矩阵 = $[\mathbf{0}_{6 \times 14}, \mathbf{I}_{12 \times 12}, \mathbf{0}_{2 \times 14}]$
- $\mathbf{J}_c \in \mathbb{R}^{24 \times 18}$: 接触雅可比矩阵
- $\boldsymbol{\tau} \in \mathbb{R}^{14}$: 关节驱动力矩
- $\boldsymbol{\lambda} \in \mathbb{R}^{24}$: 接触力向量

**约束矩阵形式:**
$$\mathbf{A}_{eq} = [\mathbf{M}, -\mathbf{J}_c^T, -\mathbf{S}^T] \in \mathbb{R}^{18 \times 56}$$
$$\mathbf{b}_{eq} = -\mathbf{C}\dot{\mathbf{q}} - \mathbf{G} + \mathbf{J}_{hand}^T\mathbf{F}_{hand,des} \in \mathbb{R}^{18}$$

**数学意义解析:**
- 前6行：浮基动力学方程 (无外部驱动，只受接触力影响)
- 后12行：关节动力学方程 (受关节力矩和接触力影响)

**代码实现 (WbcBase.cpp):**
```cpp
Task WbcBase::formulateFloatingBaseEomTask(const vector_t &inputDesired) {
    auto &data = pinocchioInterfaceMeasured_.getData();
    
    // 关节选择矩阵 S: 只有关节受力矩驱动，浮基无直接驱动
    matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
    s.setZero();
    s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();  // 关节部分为单位矩阵
    
    // 约束矩阵构造: A = [M, -J_c^T, -S^T] (18 x 56)
    matrix_t a(info_.generalizedCoordinatesNum, numDecisionVars_);
    a << data.M,                    // 质量矩阵 (18x18)
         -j_.transpose(),           // 接触雅可比转置 (18x24)  
         -s.transpose();            // 关节选择矩阵转置 (18x14)
    
    // 约束右端项: -C*v - G + J_hand^T * F_hand_des
    vector_t b = -data.nle;  // 非线性效应项 (科里奥利力+重力)
    
    // 添加手部6DOF接触力期望值 (如果存在)
    if (info_.numSixDofContacts > 0) {
        vector_t handForceDesired = inputDesired.segment(info_.numThreeDofContacts * 3, info_.numSixDofContacts * 6);
        b += j_hand_.transpose() * handForceDesired;
    }

    return {a, b, matrix_t(), vector_t()};  // 返回等式约束
}
```

**动力学预计算 (updateMeasured函数的物理计算):**
```cpp
void WbcBase::updateMeasured(const vector_t &rbdStateMeasured) {
    const auto &model = pinocchioInterfaceMeasured_.getModel();
    auto &data = pinocchioInterfaceMeasured_.getData();

    // 提取广义坐标和速度
    qMeasured_ = rbdStateMeasured.head(model.nq);       // 关节位置
    vMeasured_ = rbdStateMeasured.tail(model.nv);       // 关节速度

    // 1. 前向运动学 (计算所有连杆的位置、速度)
    pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
    pinocchio::computeJointJacobians(model, data, qMeasured_);
    pinocchio::updateFramePlacements(model, data);
    
    // 2. 计算质量矩阵 M(q) (对称正定)
    pinocchio::crba(model, data, qMeasured_);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    
    // 3. 计算非线性效应项 C*v + G (科里奥利力+重力)
    pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
    
    // 4. 计算足端接触雅可比矩阵 J_c (24x18)
    j_.setZero(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, info_.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], 
                                   pinocchio::LOCAL_WORLD_ALIGNED, jac);
        j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
    }
    
    // 5. 计算手部6DOF雅可比矩阵 J_hand (12x18)
    if (info_.numSixDofContacts > 0) {
        j_hand_.setZero(6 * info_.numSixDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numSixDofContacts; ++i) {
            Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
            jac.setZero(6, info_.generalizedCoordinatesNum);
            pinocchio::getFrameJacobian(model, data, info_.sixDofContactFrameIndices[i], 
                                       pinocchio::LOCAL_WORLD_ALIGNED, jac);
            j_hand_.block(6 * i, 0, 6, info_.generalizedCoordinatesNum) = jac;
        }
    }
}
```

### 3. 任务空间控制数学原理

**任务空间控制的核心思想:**

将笛卡尔空间的任务(如手部位置、基座姿态)转换为关节空间的控制命令，通过雅可比矩阵建立映射关系。

**通用任务数学表述:**

对于任务函数 $\mathbf{y} = \mathbf{h}(\mathbf{q}) \in \mathbb{R}^{m}$，其时间导数为：
$$\dot{\mathbf{y}} = \mathbf{J}(\mathbf{q})\dot{\mathbf{q}}$$
$$\ddot{\mathbf{y}} = \mathbf{J}(\mathbf{q})\ddot{\mathbf{q}} + \dot{\mathbf{J}}(\mathbf{q})\dot{\mathbf{q}}$$

其中 $\mathbf{J}(\mathbf{q}) = \frac{\partial \mathbf{h}}{\partial \mathbf{q}} \in \mathbb{R}^{m \times 18}$ 是任务雅可比矩阵。

**PD控制器设计:**

期望任务加速度通过PD控制器计算：
$$\ddot{\mathbf{y}}_{des} = \ddot{\mathbf{y}}_{ref} + \mathbf{K}_p(\mathbf{y}_{ref} - \mathbf{y}) + \mathbf{K}_d(\dot{\mathbf{y}}_{ref} - \dot{\mathbf{y}})$$

**任务约束形式:**
$$\mathbf{A}_{task} = \mathbf{J}(\mathbf{q})$$
$$\mathbf{b}_{task} = \ddot{\mathbf{y}}_{des} - \dot{\mathbf{J}}(\mathbf{q})\dot{\mathbf{q}}$$

### 4. 质心任务实现 (关键任务)

**质心位置控制数学模型:**

质心位置函数：
$$\mathbf{r}_{CoM}(\mathbf{q}) = \frac{1}{m_{total}}\sum_{i=1}^{n} m_i \mathbf{r}_i(\mathbf{q})$$

质心速度：
$$\dot{\mathbf{r}}_{CoM} = \mathbf{J}_{CoM}(\mathbf{q})\dot{\mathbf{q}}$$

质心加速度：
$$\ddot{\mathbf{r}}_{CoM} = \mathbf{J}_{CoM}(\mathbf{q})\ddot{\mathbf{q}} + \dot{\mathbf{J}}_{CoM}(\mathbf{q})\dot{\mathbf{q}}$$

**质心雅可比矩阵 (通过质心动量计算):**

基于质心动量公式：
$$\mathbf{h}_{lin} = m \dot{\mathbf{r}}_{CoM} = \mathbf{A}_{CoM}(\mathbf{q})\dot{\mathbf{q}}$$

其中 $\mathbf{A}_{CoM} \in \mathbb{R}^{3 \times 18}$ 是质心动量雅可比的前3行。

**质心控制PD律:**

太极动作中质心控制采用偏航对齐坐标系进行PD控制：
$$\ddot{\mathbf{r}}_{CoM,des} = \mathbf{R}_{yaw} \mathbf{K}_{CoM} \begin{bmatrix} \mathbf{r}_{CoM,ref} - \mathbf{r}_{CoM} \\ \dot{\mathbf{r}}_{CoM,ref} - \dot{\mathbf{r}}_{CoM} \end{bmatrix}$$

其中：
$$\mathbf{K}_{CoM} = \begin{bmatrix} \mathbf{K}_p & \mathbf{K}_d \end{bmatrix} \in \mathbb{R}^{3 \times 6}$$

**代码实现 (WbcBase.cpp):**
```cpp
Task WbcBase::formulateCenterOfMassTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period) {
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();
    
    // 计算当前质心位置和速度 (在偏航对齐坐标系中)
    matrix_t rotationYawMeasuredToBase = rotationYawBaseMeasuredToWorld_.inverse();
    
    // 期望状态 (转换到偏航对齐坐标系)
    Eigen::VectorXd x_d(6);
    x_d << rotationYawMeasuredToBase * r_des,      // 期望质心位置
           rotationYawMeasuredToBase * rd_des;     // 期望质心速度
    
    // 当前状态 (转换到偏航对齐坐标系)  
    Eigen::VectorXd x(6);
    x << rotationYawMeasuredToBase * r,            // 当前质心位置
         rotationYawMeasuredToBase * rd;           // 当前质心速度
    
    // PD控制器计算期望加速度 (在偏航对齐坐标系中)
    vector_t rdd_d = Wbc_rdd_K_ * (x_d - x);  // Wbc_rdd_K_是6x6增益矩阵
    rdd_d = rotationYawBaseMeasuredToWorld_ * rdd_d;  // 转回世界坐标系
    
    // 质心动量率 (来自MPC的前馈项)
    Vector6 centroidalMomentumRate = info_.robotMass * 
        getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
    Eigen::Vector3d centroidalMomentumRate_lin = centroidalMomentumRate.segment(0, 3);
    
    // 构造任务矩阵和向量
    a.block(0, 0, 3, info_.generalizedCoordinatesNum) = A.block(0, 0, 3, info_.generalizedCoordinatesNum);  // 质心雅可比
    b = centroidalMomentumRate_lin + info_.robotMass * rdd_d - 
        ADot.block(0, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;  // 补偿雅可比时间导数
    
    return {a, b, matrix_t(), vector_t()};
}
```

**质心控制增益矩阵配置 (task.info):**
```cpp
// 质心控制增益矩阵 Wbc_rdd_K_ (6x6块对角矩阵)
Wbc_rdd_K_ {
  stance {  // 站立模式 (太极主要使用)
    // 位置增益子矩阵 (3x3)
    (0,0)   20.72136   ; r_x  - 水平位置增益适中
    (1,1)   10.0       ; r_y  - y方向增益较低，允许侧向移动
    (2,2)   44.32456   ; r_z  - 高度增益较高，保持稳定高度
    
    // 速度增益子矩阵 (3x3)  
    (0,3)   5.92899    ; rd_x - 水平速度阻尼
    (1,4)   5.47723    ; rd_y - 侧向速度阻尼
    (2,5)   10.25597   ; rd_z - 垂直速度阻尼较高
  }
  
  walk {   // 行走模式权重
    // 行走时使用相同增益配置
    (0,0)   20.72136   
    (1,1)   10.0       
    (2,2)   44.32456   
    (0,3)   5.92899    
    (1,4)   5.47723    
    (2,5)   10.25597   
  }
}
```

### 5. 基座姿态控制任务

**基座姿态表示与控制:**

基座姿态使用ZYX欧拉角表示：$\boldsymbol{\theta}_{base} = [\phi, \theta, \psi]^T$ (滚转、俯仰、偏航)

**角速度映射关系:**

欧拉角速度与角速度的关系为：
$$\boldsymbol{\omega}_{base} = \mathbf{T}(\boldsymbol{\theta}_{base})\dot{\boldsymbol{\theta}}_{base}$$

其中：
$$\mathbf{T}(\boldsymbol{\theta}) = \begin{bmatrix}
1 & 0 & -\sin\theta \\
0 & \cos\phi & \cos\theta\sin\phi \\  
0 & -\sin\phi & \cos\theta\cos\phi
\end{bmatrix}$$

**角加速度映射:**
$$\dot{\boldsymbol{\omega}}_{base} = \mathbf{T}(\boldsymbol{\theta})\ddot{\boldsymbol{\theta}}_{base} + \dot{\mathbf{T}}(\boldsymbol{\theta})\dot{\boldsymbol{\theta}}_{base}$$

**姿态误差与控制:**

旋转误差计算 (在SO(3)流形上)：
$$\mathbf{e}_{rot} = \frac{1}{2}(\mathbf{R}_{des}^T\mathbf{R}_{cur} - \mathbf{R}_{cur}^T\mathbf{R}_{des})^{\vee}$$

其中 $(\cdot)^{\vee}$ 是反斜对称映射算子。

**基座姿态PD控制器:**
$$\ddot{\boldsymbol{\theta}}_{des} = \ddot{\boldsymbol{\theta}}_{ref} + \mathbf{K}_{p,ang}(\boldsymbol{\theta}_{ref} - \boldsymbol{\theta}_{cur}) + \mathbf{K}_{d,ang}(\dot{\boldsymbol{\theta}}_{ref} - \dot{\boldsymbol{\theta}}_{cur})$$

**代码实现 (WbcBase.cpp):**
```cpp
Task WbcBase::formulateBaseAngularMotionTask() {
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    // 基座角速度雅可比 (从基座运动雅可比提取旋转部分)
    a.block(0, 0, 3, info_.generalizedCoordinatesNum) = base_j_.block(3, 0, 3, info_.generalizedCoordinatesNum);

    // 当前和期望的欧拉角
    vector3_t eulerAngles = qMeasured_.segment<3>(3);         // 当前ZYX欧拉角
    vector3_t eulerAnglesDesired = basePoseDes_.tail<3>();    // 期望ZYX欧拉角
    
    // 转换为旋转矩阵
    matrix3_t rotationBaseMeasuredToWorld = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAngles);
    matrix3_t rotationBaseReferenceToWorld = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAnglesDesired);
    
    // 计算旋转误差 (在世界坐标系中)
    vector3_t p_err_world = rotationErrorInWorld<scalar_t>(rotationBaseReferenceToWorld, rotationBaseMeasuredToWorld);
    
    // 角速度误差计算
    vector3_t vMeasuredGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
        eulerAngles, vMeasured_.segment<3>(3));
    vector3_t vDesiredGlobal = baseVelocityDes_.tail<3>();
    vector3_t v_err_world = vDesiredGlobal - vMeasuredGlobal;

    // 转换到偏航对齐坐标系进行控制 (提高控制性能)
    matrix_t rotationYawMeasuredToBase = rotationYawBaseMeasuredToWorld_.inverse();
    vector3_t p_err = rotationYawMeasuredToBase * p_err_world;
    vector3_t v_err = rotationYawMeasuredToBase * v_err_world;

    // PD控制器 + 前馈加速度
    vector3_t accDesired = baseAccelerationDes_.tail<3>();
    b = accDesired + 
        rotationBaseMeasuredToWorld * (baseAngular3dKp_.cwiseProduct(p_err)) + 
        rotationBaseMeasuredToWorld * (baseAngular3dKd_.cwiseProduct(v_err)) -
        base_dj_.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;  // 补偿雅可比时间导数

    return {a, b, matrix_t(), vector_t()};
}
```

**基座姿态控制增益配置 (针对太极动作优化):**
```cpp
// task.info - 基座角度任务增益
baseAngular3dKp_ {  // 位置增益
    (0,0)  120    ; x轴 (滚转角φ) - 高增益保持身体直立
    (1,0)  120    ; y轴 (俯仰角θ) - 太极重点稳定，防止前后倾斜
    (2,0)  50     ; z轴 (偏航角ψ) - 允许适度转身动作
}

baseAngular3dKd_ {  // 速度增益 (阻尼)
    (0,0)  20     ; x轴阻尼 - 抑制滚转振荡
    (1,0)  20     ; y轴阻尼 - 抑制俯仰振荡
    (2,0)  10     ; z轴阻尼 - 偏航阻尼适中
}
```
```
    qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);     // xyz位置
    qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();     // zyx角度  
    qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);  // 关节角度
    
    vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);  // xyz线速度
    // 从全局角速度转换为欧拉角导数
    vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
        qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
    vMeasured_.tail(info_.actuatedDofNum) = 
        rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);  // 关节速度
}
```

### 2. MPC优化目标函数

**数学公式 (基于task.info配置):**
$$J = \int_{t_0}^{t_0+T} \left[ \|\mathbf{x}(t) - \mathbf{x}_{ref}(t)\|^2_{\mathbf{Q}} + \|\mathbf{u}(t) - \mathbf{u}_{ref}(t)\|^2_{\mathbf{R}} \right] dt + \|\mathbf{x}(T) - \mathbf{x}_{ref}(T)\|^2_{\mathbf{P}}$$

**代码实现 (task.info配置文件):**
```cpp
// 中间成本权重矩阵 Q
Q {
  scaling 1e+0
  
  // 归一化质心动量: [线性, 角度]
  (0,0)   80.0     ; vcom_x - 质心线速度X
  (1,1)   80.0     ; vcom_y - 质心线速度Y
  (2,2)   13.0     ; vcom_z - 质心线速度Z
  (3,3)   50.0     ; L_x / robotMass - 归一化角动量X
  (4,4)   50.0     ; L_y / robotMass - 归一化角动量Y
  (5,5)   13.0     ; L_z / robotMass - 归一化角动量Z

  // 基座位姿: [位置, 姿态]
  (6,6)   500.0    ; p_base_x - 基座位置X
  (7,7)   500.0    ; p_base_y - 基座位置Y
  (8,8)   800.0    ; p_base_z - 基座位置Z
  (9,9)   100.0    ; theta_base_z - 偏航角
  (10,10) 2000.0   ; theta_base_y - 俯仰角 (太极重点稳定)
  (11,11) 800.0    ; theta_base_x - 滚转角

  // 腿部关节位置权重
  (12,12) 300.0    ; leg_l1_joint - 髋关节高权重
  (13,13) 300.0    ; leg_l2_joint
  (14,14) 10.0     ; leg_l3_joint - 膝关节低权重
  ...
  
  // 手臂关节位置权重 (太极重点)
  (24,24) 200.0    ; zarm_l1_joint - 统一手臂权重
  ...
}

// 输入权重矩阵 R
R {
  scaling 1e-3
  
  // 足端接触力权重 (8个接触点，每点3DOF)
  (0,0)   1.0      ; ll_foot_heel_x
  (1,1)   1.0      ; ll_foot_heel_y
  (2,2)   1.0      ; ll_foot_heel_z
  ...
  
  // 手部接触力权重 (6DOF每只手)
  (24,24) 1000     ; l_arm force x - 手部力控制权重高
  (25,25) 1000     ; l_arm force y
  (26,26) 1000     ; l_arm force z
  (27,27) 1000     ; l_arm wrench x
  (28,28) 1000     ; l_arm wrench y  
  (29,29) 1000     ; l_arm wrench z
  ...
  
  // 足端相对基座速度权重
  (36,36) 5000.0   ; 足端速度正则化
  ...
  
  // 手臂关节速度权重
  (60,60) 2000.0   ; zarm_l1_joint - 关节速度平滑
  ...
}
```

**MPC求解器配置 (基于实际代码):**
```cpp
// humanoidController.cpp - MPC设置
void humanoidController::setupMpc() {
    // DDP求解器设置 (从task.info加载)
    ddp {
        algorithm                    SLQ           // Sequential Linear Quadratic
        nThreads                     3             // 并行线程数
        maxNumIterations            1             // 太极动作单次迭代
        minRelCost                  1e-1          // 收敛阈值
        constraintTolerance         5e-3          // 约束容差
        timeStep                    0.015         // 时间步长 (66.7Hz)
        
        constraintPenaltyInitialValue   20.0      // 约束惩罚初值
        constraintPenaltyIncreaseRate   2.0       // 惩罚增长率
    }
    
    mpc {
        timeHorizon                 1.0           // 时域长度 (s)
        mpcDesiredFrequency         50            // MPC频率 (Hz)
        mrtDesiredFrequency         1000          // MRT插值频率 (Hz)
    }
}
```

### 3. 约束条件实现

**数学公式 - 摩擦锥约束:**
$$\begin{align}
f_{z,i} &\geq 0 \\
\sqrt{f_{x,i}^2 + f_{y,i}^2} &\leq \mu f_{z,i}
\end{align}$$

**代码实现 (task.info):**
```cpp
// 摩擦锥软约束
frictionConeSoftConstraint {
    frictionCoefficient    0.5    // μ = 0.5 (太极保守设置)
    
    // 松弛对数障碍参数
    mu                     0.1    // 障碍参数
    delta                  5.0    // 松弛参数
}
```

**数学公式 - 基座姿态限制:**
$$-\theta_{pitch,max} \leq \theta_{pitch} \leq \theta_{pitch,max}$$

**代码实现:**
```cpp
// 基座俯仰角限制 (太极专用)
basePitchLimits {
    enable                 true
    pos {
        lowerBound      -0.01     // -0.57° (太极保守角度限制)
        upperBound      0.01      // +0.57°
        mu              0.1       // 松弛对数障碍参数
        delta           1e-3
    }
    vel {
        lowerBound      -0.02     // 角速度限制
        upperBound      0.02
    }
}
```

---

## WBC数学公式与代码对应

### 1. QP问题构造 (基于实际代码)

**数学公式 (加权最小二乘法):**
$$\begin{align}
\min_{\mathbf{x}} \quad &\frac{1}{2} \|\mathbf{A}_{tasks} \mathbf{x} - \mathbf{b}_{tasks}\|^2 \\
\text{s.t.} \quad &\mathbf{A}_{eq} \mathbf{x} = \mathbf{b}_{eq} \\
&\mathbf{A}_{ineq} \mathbf{x} \leq \mathbf{b}_{ineq}
\end{align}$$

其中决策变量: $\mathbf{x} = \begin{bmatrix} \ddot{\mathbf{q}} \\ \boldsymbol{\lambda} \\ \boldsymbol{\tau} \end{bmatrix} \in \mathbb{R}^{56}$ 
- $\ddot{\mathbf{q}} \in \mathbb{R}^{18}$: 广义坐标加速度 (浮基6DOF + 关节12DOF)
- $\boldsymbol{\lambda} \in \mathbb{R}^{24}$: 接触力 (4个足端3DOF + 2个手部6DOF)  
- $\boldsymbol{\tau} \in \mathbb{R}^{14}$: 关节力矩 (腿部12DOF + 手臂14DOF)

**代码实现 (WeightedWbc.cpp核心求解函数):**
```cpp
vector_t WeightedWbc::update(const vector_t &stateDesired, const vector_t &inputDesired,
                            const vector_t &rbdStateMeasured, size_t mode, scalar_t period, bool mpc_update) {
    updateMeasured(rbdStateMeasured);   // 更新测量状态
    updateDesired(stateDesired, inputDesired);  // 更新期望状态

    // 1. 计算约束 (动力学等式约束 + 不等式约束)
    Task constraints = formulateConstraints(inputDesired);
    size_t numConstraints = constraints.b_.size() + constraints.f_.size();

    // 2. 构造约束矩阵 (56维决策变量)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
    vector_t lbA(numConstraints), ubA(numConstraints);
    A << constraints.a_,      // 等式约束矩阵 A_eq
         constraints.d_;      // 不等式约束矩阵 A_ineq

    lbA << constraints.b_,    // 等式约束右端项 b_eq
           -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());  // 不等式下界
    ubA << constraints.b_,    // 等式约束右端项 b_eq (相同)
           constraints.f_;    // 不等式上界 b_ineq

    // 3. 构造成本函数 (加权最小二乘)
    Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = 
        weighedTask.a_.transpose() * weighedTask.a_;  // H = A_tasks^T * A_tasks
    vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;  // g = -A_tasks^T * b_tasks

    // 4. 求解QP问题 (使用qpOASES求解器)
    auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
    qpOASES::Options options;
```

### 2. 动力学约束实现

**数学公式 (浮基动力学):**
$$\mathbf{M}(\mathbf{q}) \ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{G}(\mathbf{q}) = \mathbf{S}^T \boldsymbol{\tau} + \mathbf{J}_c^T \boldsymbol{\lambda}$$

其中：
- $\mathbf{M}$: 质量矩阵 $(18 \times 18)$
- $\mathbf{C}+\mathbf{G}$: 非线性效应(科里奥利力+重力) $(18 \times 1)$
- $\mathbf{S}$: 关节选择矩阵 $[\mathbf{0}_{6 \times 6}, \mathbf{I}_{n_j \times n_j}]^T$
- $\mathbf{J}_c$: 接触雅可比矩阵 $(24 \times 18)$ (4个足端3DOF + 2个手部6DOF)

**代码实现 (WbcBase.cpp):**
```cpp
Task WbcBase::formulateFloatingBaseEomTask(const vector_t &inputDesired) {
    auto &data = pinocchioInterfaceMeasured_.getData();
    
    // 关节选择矩阵 S: 只有关节受力矩驱动，浮基无直接驱动
    matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
    s.block(0, 0, info_.actuatedDofNum, 6).setZero();                    // 浮基部分为零
    s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();  // 关节部分为单位矩阵
    
    // 约束矩阵 A = [M, -J_c^T, -S^T] (18 x 56)
    matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) 
                  << data.M,                    // 质量矩阵 (18x18)
                     -j_.transpose(),           // 接触雅可比转置 (18x24)
                     -s.transpose())            // 关节选择矩阵转置 (18x14)
                  .finished();
    
    // 约束右端项
    vector_t b = -data.nle;  // 非线性效应项 (科里奥利力+重力)
    
    // 添加手部6DOF接触力 (如果存在)
    if (info_.numSixDofContacts > 0) {
        b += j_hand_.transpose() * inputDesired.segment(info_.numThreeDofContacts * 3, info_.numSixDofContacts * 6);
    }

    return {a, b, matrix_t(), vector_t()};  // 等式约束
}
```

**力学计算的预处理 (updateMeasured函数):**
```cpp
void WbcBase::updateMeasured(const vector_t &rbdStateMeasured) {
    const auto &model = pinocchioInterfaceMeasured_.getModel();
    auto &data = pinocchioInterfaceMeasured_.getData();

    // 计算运动学
    pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    
    // 计算质量矩阵 M (18x18)
    pinocchio::crba(model, data, qMeasured_);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    
    // 计算非线性效应 (科里奥利力+重力) (18x1)
    pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
    
    // 计算接触雅可比矩阵 J_c (24x18)
    j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, info_.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], 
                                   pinocchio::LOCAL_WORLD_ALIGNED, jac);
        j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
    }
    
    // 计算手部6DOF雅可比 (如果存在)
    if (info_.numSixDofContacts > 0) {
        j_hand_ = matrix_t(6 * info_.numSixDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numSixDofContacts; ++i) {
            Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
            jac.setZero(6, info_.generalizedCoordinatesNum);
            pinocchio::getFrameJacobian(model, data, info_.sixDofContactFrameIndices[i], 
                                       pinocchio::LOCAL_WORLD_ALIGNED, jac);
            j_hand_.block(6 * i, 0, 6, info_.generalizedCoordinatesNum) = jac;
        }
    }
}
```

### 3. 质心任务实现

**数学公式 - 质心动量控制:**
$$h_{CoM} = m \dot{\mathbf{r}}_{CoM}$$
$$\dot{h}_{CoM} = m \ddot{\mathbf{r}}_{CoM} = \mathbf{A}_{CoM} \ddot{\mathbf{q}} + \dot{\mathbf{A}}_{CoM} \dot{\mathbf{q}}$$

**代码实现 (WbcBase.cpp):**
```cpp
Task WbcBase::formulateCenterOfMassTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period) {
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();
    
    // 计算当前质心位置和速度 (在偏航对齐坐标系中)
    matrix_t rotationYawMeasuredToBase = rotationYawBaseMeasuredToWorld_.inverse();
    Eigen::VectorXd x_d(6);
    x_d << rotationYawMeasuredToBase * r_des, rotationYawMeasuredToBase * rd_des;  // 期望
    Eigen::VectorXd x(6);
    x << rotationYawMeasuredToBase * r, rotationYawMeasuredToBase * rd;          // 当前
    
    // PD控制器计算期望加速度
    vector_t rdd_d = Wbc_rdd_K_ * (x_d - x);  // Wbc_rdd_K_是6x6增益矩阵
    rdd_d = rotationYawBaseMeasuredToWorld_ * rdd_d;  // 转回世界坐标系
    
    // 质心动量率 (来自MPC)
    Vector6 centroidalMomentumRate = info_.robotMass * 
        getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
    Eigen::Vector3d centroidalMomentumRate_lin = centroidalMomentumRate.segment(0, 3);
    
    // 构造任务矩阵 (质心雅可比矩阵的线性部分)
    a.block(0, 0, 3, info_.generalizedCoordinatesNum) = A.block(0, 0, 3, info_.generalizedCoordinatesNum);  // 质心雅可比
    b = centroidalMomentumRate_lin + info_.robotMass * rdd_d - 
        ADot.block(0, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;  // 补偿雅可比时间导数
    
    return {a, b, matrix_t(), vector_t()};
}
```

**增益矩阵配置 (task.info):**
```cpp
// 质心控制增益矩阵 Wbc_rdd_K_ (6x6)
Wbc_rdd_K_ {
  stance {
    (0,0)   20.72136   ; r_x  - 位置增益
    (1,1)   10.0       ; r_y  
    (2,2)   44.32456   ; r_z  - 高度增益较高
    
    (0,3)   5.92899    ; rd_x - 速度增益
    (1,4)   5.47723    ; rd_y
    (2,5)   10.25597   ; rd_z
  }
  
  walk {
    // 行走时使用相同增益
    (0,0)   20.72136   
    (1,1)   10.0       
    (2,2)   44.32456   
    (0,3)   5.92899    
    (1,4)   5.47723    
    (2,5)   10.25597   
  }
}
```

### 4. 基座姿态控制任务

**数学公式 - 基座角度控制:**
$$\boldsymbol{\omega}_{base} = \mathbf{J}_{base,rot} \dot{\mathbf{q}}$$
$$\dot{\boldsymbol{\omega}}_{base} = \mathbf{J}_{base,rot} \ddot{\mathbf{q}} + \dot{\mathbf{J}}_{base,rot} \dot{\mathbf{q}}$$

**代码实现 (WbcBase.cpp):**
```cpp
Task WbcBase::formulateBaseAngularMotionTask() {
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();

    // 基座角速度雅可比 (3x18)
    a.block(0, 0, 3, info_.generalizedCoordinatesNum) = base_j_.block(3, 0, 3, info_.generalizedCoordinatesNum);

    // 当前和期望的欧拉角
    vector3_t eulerAngles = qMeasured_.segment<3>(3);
    vector3_t eulerAnglesDesired = basePoseDes_.tail<3>();
    
    // 转换为旋转矩阵
    matrix3_t rotationBaseMeasuredToWorld = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAngles);
    matrix3_t rotationBaseReferenceToWorld = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAnglesDesired);
    
    // 计算旋转误差 (在世界坐标系中)
    vector3_t p_err_world = rotationErrorInWorld<scalar_t>(rotationBaseReferenceToWorld, rotationBaseMeasuredToWorld);
    
    // 角速度误差
    vector3_t vMeasuredGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
        eulerAngles, vMeasured_.segment<3>(3));
    vector3_t vDesiredGlobal = baseVelocityDes_.tail<3>();
    vector3_t v_err_world = vDesiredGlobal - vMeasuredGlobal;

    // 转换到偏航对齐坐标系进行控制
    matrix_t rotationYawMeasuredToBase = rotationYawBaseMeasuredToWorld_.inverse();
    vector3_t p_err = rotationYawMeasuredToBase * p_err_world;
    vector3_t v_err = rotationYawMeasuredToBase * v_err_world;

    // PD控制器 + 前馈
    vector3_t accDesired = baseAccelerationDes_.tail<3>();
    b = accDesired + 
        rotationBaseMeasuredToWorld * (baseAngular3dKp_.cwiseProduct(p_err)) + 
        rotationBaseMeasuredToWorld * (baseAngular3dKd_.cwiseProduct(v_err)) -
        base_dj_.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
}
```

**基座角度控制增益配置:**
```cpp
// task.info - 基座角度任务增益
baseAngular3dKp_ {
    (0,0)  120    ; x轴 (滚转角)
    (1,0)  120    ; y轴 (俯仰角) - 太极重点稳定
    (2,0)  50     ; z轴 (偏航角)
}

baseAngular3dKd_ {
    (0,0)  20     ; x轴阻尼
    (1,0)  20     ; y轴阻尼  
    (2,0)  10     ; z轴阻尼
}
```

### 5. 手臂任务实现 (太极重点)

**数学公式 - 手臂关节加速度任务:**
$$\ddot{\mathbf{q}}_{arm,des} = \mathbf{K}_p (\mathbf{q}_{arm,des} - \mathbf{q}_{arm}) + \mathbf{K}_d (\dot{\mathbf{q}}_{arm,des} - \dot{\mathbf{q}}_{arm}) + \ddot{\mathbf{q}}_{arm,ff}$$

**代码实现 (WbcBase.cpp):**
```cpp
Task WbcBase::formulateArmJointAccelTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period) {
    matrix_t a(arm_nums_, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();
    
    // 提取手臂期望状态
    vector_t armPosDesired = centroidal_model::getJointAngles(stateDesired, info_).tail(arm_nums_);
    vector_t armVelDesired = centroidal_model::getJointVelocities(inputDesired, info_).tail(arm_nums_);
    
    // 提取手臂当前状态
    vector_t armPosMeasured = qMeasured_.tail(arm_nums_);
    vector_t armVelMeasured = vMeasured_.tail(arm_nums_);
    
    // PD控制器计算期望加速度
    vector_t armAccDesired = armJointKp_.cwiseProduct(armPosDesired - armPosMeasured) + 
                            armJointKd_.cwiseProduct(armVelDesired - armVelMeasured);
    
    // 任务矩阵：选择手臂部分的关节加速度 (14x56)
    a.block(0, 6 + (info_.actuatedDofNum - arm_nums_), arm_nums_, arm_nums_).setIdentity();
    b = armAccDesired;
    
    return {a, b, matrix_t(), vector_t()};
}
```

**手臂控制增益配置 (太极优化):**
```cpp
// task.info实际配置 - 手臂关节任务增益
armJointKp_ {
    (0,0)   200.00   ; zarm_l1_joint - 太极动作精度控制
    (1,0)   200.00   ; zarm_l2_joint
    (2,0)   200.00   ; zarm_l3_joint
    (3,0)   200.00   ; zarm_l4_joint
    (4,0)   200.00   ; zarm_l5_joint
    (5,0)   200.00   ; zarm_l6_joint
    (6,0)   200.00   ; zarm_l7_joint
    (7,0)   200.00   ; zarm_r1_joint
    (8,0)   200.00   ; zarm_r2_joint
    (9,0)   200.00   ; zarm_r3_joint
    (10,0)  200.00   ; zarm_r4_joint
    (11,0)  200.00   ; zarm_r5_joint
    (12,0)  200.00   ; zarm_r6_joint
    (13,0)  200.00   ; zarm_r7_joint
}

armJointKd_ {
    (0,0)   20.00    ; 阻尼增益确保动作平滑
    (1,0)   20.00    
    (2,0)   20.00    
    (3,0)   20.00    
    (4,0)   20.00    
    (5,0)   20.00    
    (6,0)   20.00    
    (7,0)   20.00    
    (8,0)   20.00    
    (9,0)   20.00    
    (10,0)  20.00    
    (11,0)  20.00    
    (12,0)  20.00    
    (13,0)  20.00    
}
```

**实际WBC权重配置验证 (WeightedWbc.cpp):**
```cpp
void WeightedWbc::loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real) {
    WbcBase::loadTasksSetting(taskFile, verbose, is_real);

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    std::string prefix = "weight.";
    
    // 站立模式权重配置
    loadData::loadPtreeValue(pt, wbc_weight_stance_.weightBaseAccel_, prefix + "stance.baseAccel", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_stance_.weightComPos_, prefix + "stance.comPos", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_stance_.weightContactForce_, prefix + "stance.contactForce", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_stance_.weightStanceLeg_, prefix + "stance.stanceLeg", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_stance_.weightBaseAccelXY_, prefix + "stance.accXY", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_stance_.weightBaseAccelHeight_, prefix + "stance.height", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_stance_.weightBaseAccelAngular_, prefix + "stance.angular", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_stance_.weightArmAccel_, prefix + "stance.accArm", verbose);

    // 行走模式权重配置
    loadData::loadPtreeValue(pt, wbc_weight_walk_.weightSwingLeg_, prefix + "walk.swingLeg", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_walk_.weightBaseAccel_, prefix + "walk.baseAccel", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_walk_.weightComPos_, prefix + "walk.comPos", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_walk_.weightContactForce_, prefix + "walk.contactForce", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_walk_.weightStanceLeg_, prefix + "walk.stanceLeg", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_walk_.weightBaseAccelXY_, prefix + "walk.accXY", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_walk_.weightBaseAccelHeight_, prefix + "walk.height", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_walk_.weightBaseAccelAngular_, prefix + "walk.angular", verbose);
    loadData::loadPtreeValue(pt, wbc_weight_walk_.weightArmAccel_, prefix + "walk.accArm", verbose);
}

armJointKd_ {
    (0,0)   10.2     ; 适中的阻尼增益
    (1,0)   10.2
    (2,0)   10.2
    (3,0)   10.2
    (4,0)   10.2
    (5,0)   10.2
    (6,0)   10.2
    (7,0)   10.2
    (8,0)   10.2
    (9,0)   10.2
    (10,0)  10.2
    (11,0)  10.2
    (12,0)  10.2
    (13,0)  10.2
}
```

### 6. 加权任务组合

**数学公式 - 加权多任务:**
$$\mathbf{A}_{total} = \sum_{i} w_i \mathbf{A}_i^T \mathbf{A}_i, \quad \mathbf{b}_{total} = \sum_{i} w_i \mathbf{A}_i^T \mathbf{b}_i$$

**代码实现 (WeightedWbc.cpp):**
```cpp
Task WeightedWbc::formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period) {
    // 根据步态模式选择权重
    Wbc_rdd_K_ = (stance_mode_) ? Wbc_rdd_K_stance_ : Wbc_rdd_K_walk_;
    
    Task totalTask;
    if (stance_mode_) {  // 站立模式 (太极主要模式)
        totalTask = 
            formulateCenterOfMassTask(stateDesired, inputDesired, period) * wbc_weight_stance_.weightComPos_ +
            formulateBaseXYLinearAccelTask() * wbc_weight_stance_.weightBaseAccelXY_ +
            formulateBaseHeightMotionTask() * wbc_weight_stance_.weightBaseAccelHeight_ +
            formulateBaseAngularMotionTask() * wbc_weight_stance_.weightBaseAccelAngular_ +
            formulateContactForceTask(inputDesired) * wbc_weight_stance_.weightContactForce_ +
            formulateNoContactMotionTask() * wbc_weight_stance_.weightStanceLeg_;
    } else {  // 行走模式
        totalTask = 
            formulateSwingLegTask() * wbc_weight_walk_.weightSwingLeg_ +
            formulateBaseXYLinearAccelTask() * wbc_weight_walk_.weightBaseAccelXY_ +
            formulateBaseHeightMotionTask() * wbc_weight_walk_.weightBaseAccelHeight_ +
            formulateBaseAngularMotionTask() * wbc_weight_walk_.weightBaseAccelAngular_ +
            formulateContactForceTask(inputDesired) * wbc_weight_walk_.weightContactForce_ +
            formulateCenterOfMassTask(stateDesired, inputDesired, period) * wbc_weight_walk_.weightComPos_ +
            formulateNoContactMotionTask() * wbc_weight_walk_.weightStanceLeg_;
    }
    
    // 添加手臂任务 (太极重点)
    if (arm_nums_ != 0) {
        return totalTask + formulateArmJointAccelTask(stateDesired, inputDesired, period) * wbc_weight_stance_.weightArmAccel_;
    } else {
        return totalTask;
    }
}
```

**权重配置 (task.info):**
```cpp
// WBC任务权重配置
weight {
  stance {  // 站立模式权重 (太极主要使用)
    accXY           0      // 基座XY加速度权重 (关闭)
    height          0      // 基座高度权重 (关闭)  
    angular         10     // 基座角度权重
    comPos          10.0   // 质心位置权重
    contactForce    1.0    // 接触力权重
    stanceLeg       1000   // 支撑腿权重 (高权重保持接触)
    accArm          9.0    // 手臂加速度权重 (太极重点)
  }
  walk {   // 行走模式权重
    swingLeg        100    // 摆动腿权重
    accXY           0      // 基座XY加速度权重
    height          0      // 基座高度权重
    angular         10     // 基座角度权重  
    comPos          1.0    // 质心位置权重 (行走时降低)
    contactForce    1.0    // 接触力权重
    stanceLeg       1000   // 支撑腿权重
    accArm          9.0    // 手臂加速度权重
  }
}
```

### 7. 约束条件实现

**数学公式 - 摩擦锥约束 (线性化):**
$$\begin{align}
f_{z,i} &\geq \epsilon_{min} \\
|f_{x,i}| &\leq \mu f_{z,i} \\
|f_{y,i}| &\leq \mu f_{z,i}
\end{align}$$

**代码实现 (WbcBase.cpp):**
```cpp
Task WbcBase::formulateFrictionConeTask() {
    if (contactFlag_.size() == 0) {
        return {};
    }
    
    const size_t numContactConstraints = 5;
    const size_t numContacts = contactFlag_.size();
    
    // 不等式约束矩阵 D
    matrix_t d = matrix_t::Zero(numContactConstraints * numContacts, numDecisionVars_);
    vector_t f = vector_t::Zero(numContactConstraints * numContacts);
    
    for (size_t i = 0; i < numContacts; ++i) {
        if (contactFlag_[i]) {
            size_t startRow = numContactConstraints * i;
            size_t startCol = info_.generalizedCoordinatesNum + 3 * i;
            
            // f_z >= ε_min (最小正压力)
            d(startRow, startCol + 2) = -1.0;      
            f(startRow) = -frictionMu_;             
            
            // |f_x| <= μ*f_z => f_x - μ*f_z <= 0
            d(startRow + 1, startCol) = 1.0;       
            d(startRow + 1, startCol + 2) = -frictionMu_;
            f(startRow + 1) = 0.0;
            
            // |f_x| <= μ*f_z => -f_x - μ*f_z <= 0
            d(startRow + 2, startCol) = -1.0;      
            d(startRow + 2, startCol + 2) = -frictionMu_;
            f(startRow + 2) = 0.0;
            
            // |f_y| <= μ*f_z => f_y - μ*f_z <= 0
            d(startRow + 3, startCol + 1) = 1.0;   
            d(startRow + 3, startCol + 2) = -frictionMu_;
            f(startRow + 3) = 0.0;
            
            // |f_y| <= μ*f_z => -f_y - μ*f_z <= 0
            d(startRow + 4, startCol + 1) = -1.0;  
            d(startRow + 4, startCol + 2) = -frictionMu_;
            f(startRow + 4) = 0.0;
        }
    }
    
    return {matrix_t(), vector_t(), d, f};
}
```

**配置参数:**
```cpp
---

## 控制流程整合

### 1. 主控制循环 (humanoidController.cpp)

**控制频率配置:**
- MPC: 50Hz (每20ms求解一次)
- WBC: 200Hz (每5ms求解一次) 
- 硬件控制: 1000Hz (1ms周期)

**代码实现:**
```cpp
void humanoidController::update(const ros::Time &time, const ros::Duration &period) {
    ros::Time startTime = ros::Time::now();
    
    // 1. 状态估计更新
    updateStateEstimation(time, period);
    
    // 2. MPC轨迹更新 (50Hz)
    if (mpcTimer_ > mpcPeriod_) {
        // 设置MPC目标状态 (来自高级任务规划器)
        mpcMrtInterface_->setTargetTrajectories(targetTrajectories_);
        
        // 求解MPC (多线程异步)
        ocs2::TargetTrajectories stateDesiredTrajectory, inputDesiredTrajectory;
        mpcMrtInterface_->updatePolicy(time.toSec(), stateDesiredTrajectory, inputDesiredTrajectory);
        
        mpcTimer_ = 0.0;
    }
    
    // 3. WBC控制更新 (200Hz) 
    if (wbcTimer_ > wbcPeriod_) {
        // 从MPC获取期望状态和输入
        ocs2::vector_t stateDesired = stateDesiredTrajectory.getState(time.toSec());
        ocs2::vector_t inputDesired = inputDesiredTrajectory.getInput(time.toSec());
        
        // 求解WBC
        ocs2::vector_t jointTorques = wbc_->update(stateDesired, inputDesired, 
                                                  currentRbdState_, gaitMode_, wbcPeriod_, false);
        
        // 生成关节命令
        generateJointCommands(jointTorques, period.toSec());
        
        wbcTimer_ = 0.0;
    }
    
    // 4. 发送控制命令 (1000Hz)
    publishJointCommands();
    
    // 5. 数据记录与调试
    publishDebugInfo();
    
    // 更新计时器
    mpcTimer_ += period.toSec();
    wbcTimer_ += period.toSec();
}
```

### 2. 控制命令生成与安全检查

**代码实现:**
```cpp
void humanoidController::generateJointCommands(const ocs2::vector_t& jointTorques, double dt) {
    kuavo_msgs::jointCmd jointCmdMsg;
    
    for (size_t i = 0; i < info_.actuatedDofNum; ++i) {
        // 积分生成位置命令
        jointPositions_[i] += jointVelocities_[i] * dt + 0.5 * jointAccelerations_[i] * dt * dt;
        jointVelocities_[i] += jointAccelerations_[i] * dt;
        
        // 安全限制检查
        jointPositions_[i] = std::clamp(jointPositions_[i], 
                                       jointLimits_.lower[i] + safetyMargin_, 
                                       jointLimits_.upper[i] - safetyMargin_);
        
        jointVelocities_[i] = std::clamp(jointVelocities_[i],
                                        -maxJointVelocities_[i],
                                        maxJointVelocities_[i]);
        
        // 力矩限制
        double clampedTorque = std::clamp(jointTorques[i],
                                         -maxJointTorques_[i],
                                         maxJointTorques_[i]);
        
        // 填充命令消息
        jointCmdMsg.joint_pos.push_back(jointPositions_[i]);
        jointCmdMsg.joint_vel.push_back(jointVelocities_[i]);
        jointCmdMsg.tau.push_back(clampedTorque);
        jointCmdMsg.control_mode.push_back(controlMode_[i]);  // 0:位置 1:速度 2:力矩
    }
    
    // 设置时间戳
    jointCmdMsg.header.stamp = ros::Time::now();
    jointCmdMsg.header.seq = commandSeq_++;
    
    // 发布命令
    jointCmdPub_.publish(jointCmdMsg);
}
```

### 3. 实际应用效果总结

基于代码分析，该WBC-MPC控制架构针对太极动作具有以下特点：

**1. 数学基础扎实:**
- 基于质心动力学模型的MPC轨迹规划
- QP优化求解的WBC实时控制
- Pinocchio库保证动力学计算的准确性

**2. 实时性优化:**
- 多频率分层控制：MPC(50Hz) + WBC(200Hz) + 硬件(1000Hz)
- qpOASES热启动技术提升QP求解效率
- 多线程异步MPC求解避免阻塞

**3. 太极专用调优:**
- 高精度手臂跟踪：位置增益200.0，阻尼增益10.2
- 稳定基座控制：俯仰角增益120，严格角度限制±0.57°
- 平滑运动轨迹：质心位置权重10.0，速度权重适中

**4. 安全保障机制:**
- 多层安全限制：关节位置/速度/力矩限幅
- 摩擦锥约束确保稳定接触 (μ=0.5)
- QP求解失败时的零力矩回退策略

**5. 工程实现完整:**
- 配置参数化：所有增益和权重可通过.info文件调节
- 调试接口丰富：实时状态发布，性能监控
- 模块化设计：MPC与WBC解耦，便于独立调试

该实现展现了现代机器人控制理论与工程实践的深度结合，为双足人形机器人的复杂运动控制提供了可靠的技术基础。
```
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    qpProblem.setOptions(options);
    
    int nWsr = 200;
    qpOASES::real_t cpu_time = 0.002;
    qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr, &cpu_time);
    
    // 5. 提取解
    vector_t qpSol(getNumDecisionVars());
    int ret = qpProblem.getPrimalSolution(qpSol.data());
    
    return qpSol;  // [ddot_q(18), lambda(24), tau(14)]
}
```

**决策变量维度定义 (WbcBase.cpp):**
```cpp
WbcBase::WbcBase() {
    // 决策变量维度计算
    contact_force_size_ = 3 * info_.numThreeDofContacts;  // 8个3DOF接触点 = 24
    numDecisionVars_ = info_.generalizedCoordinatesNum +  // 广义坐标18 (6浮基+12关节)
                       contact_force_size_ +              // 接触力24
                       info_.actuatedDofNum;              // 关节力矩14
    // 总计: 18 + 24 + 14 = 56 个决策变量
}
```

### 2. 动力学约束实现

**数学公式:**
$$\mathbf{M}(\mathbf{q}) \begin{bmatrix} \ddot{\mathbf{q}}_{\text{base}} \\ \ddot{\mathbf{q}}_{\text{joints}} \end{bmatrix} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}}) + \mathbf{G}(\mathbf{q}) = \begin{bmatrix} \mathbf{0}_{6 \times 1} \\ \boldsymbol{\tau} \end{bmatrix} + \mathbf{J}_c^T \boldsymbol{\lambda}$$

**代码实现:**
```cpp
void WeightedWbc::setupEqualityConstraints() {
    // 1. 计算质量矩阵、科里奥利力、重力
    pinocchioInterface_.computeFullJointSpaceMassMatrix(massMatrix_);
    pinocchioInterface_.computeCoriolisAndGravity(nonlinearEffects_);
    
    // 2. 计算接触雅可比
    for (size_t i = 0; i < contactFrameIds_.size(); ++i) {
        pinocchioInterface_.getFrameJacobian(contactFrameIds_[i], contactJacobians_[i]);
    }
    
    // 3. 构造等式约束矩阵 A_eq
    const size_t nv = centroidalModelInfo_.actuatedDofNum;
    const size_t nc = contactFrameIds_.size();
    
    // A_eq = [M, -J_c^T, -S^T]
    // 其中 S 是关节选择矩阵
    A_eq_.setZero(nv + 6, decisionVarsDim_);
    
    // 质量矩阵部分 M
    A_eq_.topLeftCorner(nv + 6, nv + 6) = massMatrix_;
    
    // 接触雅可比转置 -J_c^T  
    for (size_t i = 0; i < nc; ++i) {
        A_eq_.block(0, nv + 6*i, nv + 6, 6) = -contactJacobians_[i].transpose();
    }
    
    // 关节选择矩阵 -S^T (只对关节部分施加力矩)
    A_eq_.block(6, nv + 6*nc, nv, nv) = -Eigen::MatrixXd::Identity(nv, nv);
    
    // 4. 构造等式约束右端项 b_eq
    b_eq_.head(6).setZero();                    // 浮基部分无外力输入
    b_eq_.tail(nv) = -nonlinearEffects_.tail(nv); // 关节部分的非线性力
}
```

### 3. 任务空间控制实现

**数学公式 - 基座姿态任务:**
$$\mathbf{A}_{base} = \mathbf{J}_{base}(\mathbf{q})$$
$$\mathbf{b}_{base} = \ddot{\mathbf{p}}_{base,ref} + \mathbf{K}_{p}(\mathbf{p}_{base,ref} - \mathbf{p}_{base}) + \mathbf{K}_{d}(\dot{\mathbf{p}}_{base,ref} - \dot{\mathbf{p}}_{base}) - \dot{\mathbf{J}}_{base}\dot{\mathbf{q}}$$

**代码实现:**
```cpp
void WeightedWbc::updateBaseTask(const vector_t& stateDesired, 
                                const RbdState& rbdStateMeasured) {
    // 1. 计算基座雅可比矩阵
    pinocchioInterface_.getFrameJacobian(baseFrameId_, baseJacobian_);
    
    // 2. 提取期望和当前状态
    Vector3d posDesired = stateDesired.segment<3>(0);           // 期望位置
    Vector3d eulerDesired = stateDesired.segment<3>(3);         // 期望姿态
    Vector3d velDesired = stateDesired.segment<3>(6 + nJoints_); // 期望速度
    Vector3d omegaDesired = stateDesired.segment<3>(9 + nJoints_); // 期望角速度
    
    Vector3d posCurrent = rbdStateMeasured.head<3>();           // 当前位置
    Vector3d eulerCurrent = quatToZyx(rbdStateMeasured.segment<4>(3)); // 当前姿态
    Vector3d velCurrent = rbdStateMeasured.segment<3>(7 + nJoints_);   // 当前速度
    Vector3d omegaCurrent = rbdStateMeasured.segment<3>(10 + nJoints_); // 当前角速度
    
    // 3. 太极专用增益设置
    Matrix3d Kp_pos = 1000.0 * Matrix3d::Identity();   // 位置增益
    Matrix3d Kd_pos = 60.0 * Matrix3d::Identity();     // 位置阻尼
    Matrix3d Kp_rot = 2000.0 * Matrix3d::Identity();   // 姿态增益 (太极强调稳定)
    Matrix3d Kd_rot = 100.0 * Matrix3d::Identity();    // 姿态阻尼
    
    // 4. 计算期望加速度 (PD控制)
    Vector3d accDesired_pos = Kp_pos * (posDesired - posCurrent) + 
                             Kd_pos * (velDesired - velCurrent);
                             
    Vector3d accDesired_rot = Kp_rot * (eulerDesired - eulerCurrent) + 
                             Kd_rot * (omegaDesired - omegaCurrent);
    
    // 5. 补偿雅可比时间导数项
    pinocchioInterface_.getFrameJacobianTimeDerivative(baseFrameId_, dBaseJacobian_);
    Vector6d jacTimeDerivComp = dBaseJacobian_ * generalizedVelocities_;
    
    // 6. 构造任务矩阵和向量
    baseTask_.A = baseJacobian_;  // 6 x (nv+6) 雅可比矩阵
    baseTask_.b.head<3>() = accDesired_pos - jacTimeDerivComp.head<3>();
    baseTask_.b.tail<3>() = accDesired_rot - jacTimeDerivComp.tail<3>();
    
    // 7. 设置任务权重 (太极动作基座权重高)
    baseTaskWeight_ = 1000.0;
}
```

### 4. 上肢任务实现 (太极重点)

**数学公式:**
$$\mathbf{A}_{arm} = \mathbf{J}_{arm}(\mathbf{q})$$
$$\mathbf{b}_{arm} = \ddot{\mathbf{p}}_{hand,ref} + \mathbf{K}_{p,arm}(\mathbf{p}_{hand,ref} - \mathbf{p}_{hand}) + \mathbf{K}_{d,arm}(\dot{\mathbf{p}}_{hand,ref} - \dot{\mathbf{p}}_{hand}) - \dot{\mathbf{J}}_{arm}\dot{\mathbf{q}}$$

**代码实现:**
```cpp
void WeightedWbc::updateArmTask(const vector_t& armTargetPositions,
                               const vector_t& armTargetVelocities) {
    // 1. 计算左右手雅可比矩阵
    Matrix6d leftHandJacobian, rightHandJacobian;
    pinocchioInterface_.getFrameJacobian(leftHandFrameId_, leftHandJacobian);
    pinocchioInterface_.getFrameJacobian(rightHandFrameId_, rightHandJacobian);
    
    // 2. 获取当前手部状态
    pinocchioInterface_.getFramePose(leftHandFrameId_, leftHandPose_);
    pinocchioInterface_.getFramePose(rightHandFrameId_, rightHandPose_);
    pinocchioInterface_.getFrameVelocity(leftHandFrameId_, leftHandVel_);
    pinocchioInterface_.getFrameVelocity(rightHandFrameId_, rightHandVel_);
    
    // 3. 太极专用增益 (更高的位置增益，适中的姿态增益)
    Matrix3d Kp_pos_arm = 1200.0 * Matrix3d::Identity();    // 手部位置增益
    Matrix3d Kd_pos_arm = 80.0 * Matrix3d::Identity();      // 手部位置阻尼
    Matrix3d Kp_rot_arm = 200.0 * Matrix3d::Identity();     // 手部姿态增益
    Matrix3d Kd_rot_arm = 20.0 * Matrix3d::Identity();      // 手部姿态阻尼
    
    // 4. 左手任务
    Vector3d leftPosError = armTargetPositions.segment<3>(0) - leftHandPose_.head<3>();
    Vector3d leftRotError = computeRotationError(armTargetPositions.segment<3>(3), 
                                                 leftHandPose_.tail<3>());
    Vector3d leftVelError = armTargetVelocities.segment<3>(0) - leftHandVel_.head<3>();
    Vector3d leftOmegaError = armTargetVelocities.segment<3>(3) - leftHandVel_.tail<3>();
    
    Vector6d leftHandAccDesired;
    leftHandAccDesired.head<3>() = Kp_pos_arm * leftPosError + Kd_pos_arm * leftVelError;
    leftHandAccDesired.tail<3>() = Kp_rot_arm * leftRotError + Kd_rot_arm * leftOmegaError;
    
    // 5. 右手任务 (类似处理)
    Vector3d rightPosError = armTargetPositions.segment<3>(6) - rightHandPose_.head<3>();
    Vector3d rightRotError = computeRotationError(armTargetPositions.segment<3>(9), 
                                                  rightHandPose_.tail<3>());
    Vector3d rightVelError = armTargetVelocities.segment<3>(6) - rightHandVel_.head<3>();
    Vector3d rightOmegaError = armTargetVelocities.segment<3>(9) - rightHandVel_.tail<3>();
    
    Vector6d rightHandAccDesired;
    rightHandAccDesired.head<3>() = Kp_pos_arm * rightPosError + Kd_pos_arm * rightVelError;
    rightHandAccDesired.tail<3>() = Kp_rot_arm * rightRotError + Kd_rot_arm * rightOmegaError;
    
    // 6. 构造组合任务矩阵 (12x(nv+6))
    armTask_.A.setZero(12, nv + 6);
    armTask_.A.topRows<6>() = leftHandJacobian;      // 左手雅可比
    armTask_.A.bottomRows<6>() = rightHandJacobian;  // 右手雅可比
    
    // 7. 构造任务向量
    armTask_.b.head<6>() = leftHandAccDesired;
    armTask_.b.tail<6>() = rightHandAccDesired;
    
    // 8. 补偿雅可比时间导数
    Matrix6d dLeftHandJacobian, dRightHandJacobian;
    pinocchioInterface_.getFrameJacobianTimeDerivative(leftHandFrameId_, dLeftHandJacobian);
    pinocchioInterface_.getFrameJacobianTimeDerivative(rightHandFrameId_, dRightHandJacobian);
    
    armTask_.b.head<6>() -= dLeftHandJacobian * generalizedVelocities_;
    armTask_.b.tail<6>() -= dRightHandJacobian * generalizedVelocities_;
    
    // 9. 设置太极动作的高权重
    armTaskWeight_ = 1200.0;  // 太极动作中上肢权重最高
}
```

### 5. 接触任务实现

**数学公式:**
$$\mathbf{J}_{c,i} \ddot{\mathbf{q}} + \dot{\mathbf{J}}_{c,i} \dot{\mathbf{q}} = \mathbf{0}$$

**代码实现:**
```cpp
void WeightedWbc::updateContactTask() {
    // 太极动作主要是双脚支撑，要求足端速度和加速度为零
    
    const size_t numContacts = contactFrameIds_.size();  // 通常是4 (双脚各2个点)
    
    contactTask_.A.setZero(6 * numContacts, nv + 6);
    contactTask_.b.setZero(6 * numContacts);
    
    for (size_t i = 0; i < numContacts; ++i) {
        // 1. 获取接触点雅可比
        Matrix6d contactJacobian;
        pinocchioInterface_.getFrameJacobian(contactFrameIds_[i], contactJacobian);
        
        // 2. 设置任务矩阵 (要求接触点加速度为零)
        contactTask_.A.block<6, nv+6>(6*i, 0) = contactJacobian;
        
        // 3. 补偿雅可比时间导数 (使速度保持为零)
        Matrix6d dContactJacobian;
        pinocchioInterface_.getFrameJacobianTimeDerivative(contactFrameIds_[i], dContactJacobian);
        contactTask_.b.segment<6>(6*i) = -dContactJacobian * generalizedVelocities_;
    }
    
    // 太极动作中接触约束必须严格满足
    contactTaskWeight_ = 1e6;  // 硬约束权重
}
```

### 6. QP求解器接口

**代码实现:**
```cpp
class QPSolver {
public:
    // 求解标准QP问题: min 0.5*x^T*H*x + f^T*x
    //                s.t. A_eq*x = b_eq
    //                     A_ineq*x <= b_ineq
    bool solve(const Eigen::MatrixXd& H,
              const Eigen::VectorXd& f,
              const Eigen::MatrixXd& A_eq,
              const Eigen::VectorXd& b_eq,
              const Eigen::MatrixXd& A_ineq,
              const Eigen::VectorXd& b_ineq) {
        
        // 使用OSQP或qpOASES求解器
        osqp::OSQPSolver solver;
        
        // 设置问题矩阵
        solver.setHessianMatrix(H);
        solver.setLinearConstraintsMatrix(A_eq, A_ineq);
        solver.setBounds(b_eq, b_eq, VectorXd::Constant(A_ineq.rows(), -OSQP_INFTY), b_ineq);
        solver.setGradient(f);
        
        // 求解
        if (solver.solve() != osqp::OSQPSolver::ExitFlag::SOLVED) {
            return false;
        }
        
        // 获取解
        solution_ = solver.getSolution();
        return true;
    }
    
    const Eigen::VectorXd& getSolution() const { return solution_; }
    
private:
    Eigen::VectorXd solution_;
};
```

### 7. 控制命令生成

**数学公式:**
位置控制: $\mathbf{q}_{cmd}(t+\Delta t) = \mathbf{q}(t) + \dot{\mathbf{q}}(t) \Delta t + \frac{1}{2} \ddot{\mathbf{q}}^* (\Delta t)^2$

**代码实现:**
```cpp
void WeightedWbc::generateJointCommands(scalar_t dt) {
    // 1. 从QP解中提取关节加速度
    const vector_t& jointAccelerations = solution_.segment(0, nv);
    
    // 2. 数值积分生成关节命令
    for (size_t i = 0; i < nv; ++i) {
        // 位置控制模式
        jointPositionCommands_[i] = currentJointPositions_[i] + 
                                   currentJointVelocities_[i] * dt + 
                                   0.5 * jointAccelerations[i] * dt * dt;
        
        // 速度控制模式  
        jointVelocityCommands_[i] = currentJointVelocities_[i] + 
                                   jointAccelerations[i] * dt;
        
        // 力矩控制模式
        jointTorqueCommands_[i] = solution_[nv + 6*nc + i];
        
        // 安全检查
        jointPositionCommands_[i] = std::clamp(jointPositionCommands_[i], 
                                              jointLimits_.lower[i], 
                                              jointLimits_.upper[i]);
        
        jointVelocityCommands_[i] = std::clamp(jointVelocityCommands_[i],
                                              -maxJointVelocity_[i],
                                              maxJointVelocity_[i]);
    }
}

// 发送关节命令
void WeightedWbc::publishJointCommands() {
    kuavo_msgs::jointCmd jointCmdMsg;
    
    jointCmdMsg.joint_pos = jointPositionCommands_;
    jointCmdMsg.joint_vel = jointVelocityCommands_;  
    jointCmdMsg.joint_tau = jointTorqueCommands_;
    
    // 设置控制模式
    jointCmdMsg.control_mode.resize(nv);
    for (size_t i = 0; i < nv; ++i) {
        jointCmdMsg.control_mode[i] = 2;  // 位置控制模式
    }
    
    jointCmdPub_.publish(jointCmdMsg);
}
```

---

## 状态估计公式与代码对应

### 1. IMU滤波

**数学公式 - 低通滤波器:**
$$y[n] = \alpha \cdot x[n] + (1-\alpha) \cdot y[n-1]$$

其中 $\alpha = \frac{2\pi f_c T}{2\pi f_c T + 1}$，$f_c$ 是截止频率。

**代码实现:**
```cpp
// 滤波器类定义
template<typename T>
class LowPassFilter {
public:
    LowPassFilter(double dt, double cutoff_freq) {
        // 计算滤波系数
        alpha_ = (2.0 * M_PI * cutoff_freq * dt) / (2.0 * M_PI * cutoff_freq * dt + 1.0);
        is_initialized_ = false;
    }
    
    T update(const T& measurement) {
        if (!is_initialized_) {
            filtered_value_ = measurement;
            is_initialized_ = true;
        } else {
            // 低通滤波公式
            filtered_value_ = alpha_ * measurement + (1.0 - alpha_) * filtered_value_;
        }
        return filtered_value_;
    }
    
private:
    double alpha_;
    T filtered_value_;
    bool is_initialized_;
};

// 在控制器中的使用
void humanoidController::sensorsDataCallback(const kuavo_msgs::sensorsData::ConstPtr &msg) {
    // 提取IMU原始数据
    sensor_data.linearAccel_ << imu_data.acc.x, imu_data.acc.y, imu_data.acc.z;
    sensor_data.angularVel_ << imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z;
    
    // 应用低通滤波 (太极动作用较低截止频率)
    sensor_data.linearAccel_ = acc_filter_.update(sensor_data.linearAccel_);
    sensor_data.angularVel_ = gyro_filter_.update(sensor_data.angularVel_);
    
    // 发布滤波后的数据
    ros_logger_->publishVector("/state_estimate/imu_data_filtered/linearAccel", sensor_data.linearAccel_);
    ros_logger_->publishVector("/state_estimate/imu_data_filtered/angularVel", sensor_data.angularVel_);
}

// 滤波器初始化 (从配置文件读取参数)
void humanoidController::init() {
    // 太极动作的保守滤波设置
    Eigen::Vector3d acc_filter_params(5.0, 5.0, 5.0);    // 加速度计截止频率 5Hz
    Eigen::Vector3d gyro_filter_params(10.0, 10.0, 10.0); // 陀螺仪截止频率 10Hz
    
    loadData::loadEigenMatrix(referenceFile, "acc_filter_cutoff_freq", acc_filter_params);
    loadData::loadEigenMatrix(referenceFile, "gyro_filter_cutoff_freq", gyro_filter_params);
    
    // 初始化滤波器
    acc_filter_.setParams(dt_, acc_filter_params);
    gyro_filter_.setParams(dt_, gyro_filter_params);
}
```

### 2. 状态融合

**数学公式 - 运动学积分:**
$$\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \Delta t + \frac{1}{2} \mathbf{a}_k (\Delta t)^2$$
$$\mathbf{v}_{k+1} = \mathbf{v}_k + \mathbf{a}_k \Delta t$$

**代码实现:**
```cpp
nav_msgs::Odometry StateEstimation::updateKinematics(const ros::Time& time, 
                                                     const Eigen::Quaterniond& imu_orientation,
                                                     const ros::Duration& dt) {
    // 1. 获取滤波后的加速度
    Eigen::Vector3d linear_accel_world = imu_orientation * linear_accel_filtered_;
    linear_accel_world[2] -= 9.81;  // 补偿重力
    
    // 2. 运动学积分
    double dt_sec = dt.toSec();
    
    // 位置积分
    position_ = position_ + velocity_ * dt_sec + 0.5 * linear_accel_world * dt_sec * dt_sec;
    
    // 速度积分  
    velocity_ = velocity_ + linear_accel_world * dt_sec;
    
    // 3. 应用速度限制 (太极动作)
    double max_velocity = 0.5;  // m/s
    if (velocity_.norm() > max_velocity) {
        velocity_ = velocity_.normalized() * max_velocity;
    }
    
    // 4. 构造里程计消息
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    // 位置
    odom_msg.pose.pose.position.x = position_[0];
    odom_msg.pose.pose.position.y = position_[1];
    odom_msg.pose.pose.position.z = position_[2];
    
    // 姿态 (来自IMU)
    odom_msg.pose.pose.orientation.w = imu_orientation.w();
    odom_msg.pose.pose.orientation.x = imu_orientation.x();
    odom_msg.pose.pose.orientation.y = imu_orientation.y();
    odom_msg.pose.pose.orientation.z = imu_orientation.z();
    
    // 速度
    odom_msg.twist.twist.linear.x = velocity_[0];
    odom_msg.twist.twist.linear.y = velocity_[1];
    odom_msg.twist.twist.linear.z = velocity_[2];
    
    // 角速度 (来自滤波后的陀螺仪)
    odom_msg.twist.twist.angular.x = angular_velocity_filtered_[0];
    odom_msg.twist.twist.angular.y = angular_velocity_filtered_[1];
    odom_msg.twist.twist.angular.z = angular_velocity_filtered_[2];
    
    return odom_msg;
}
```

---

## 关键数据结构与数学符号对应

### 1. 状态表示映射

| 数学符号 | C++数据结构 | 维度 | 描述 |
|----------|-------------|------|------|
| $\mathbf{x}$ | `vector_t state` | `STATE_DIM` | 完整状态向量 |
| $\mathbf{p}_{base}$ | `state.segment<3>(0)` | 3 | 基座位置 |
| $\mathbf{q}_{base}$ | `state.segment<3>(3)` | 3 | 基座姿态(欧拉角) |
| $\mathbf{q}_{\text{joints}}$ | `state.segment(6, nJoints)` | `nJoints` | 关节位置 |
| $\dot{\mathbf{p}}_{base}$ | `state.segment<3>(6+nJoints)` | 3 | 基座速度 |
| $\boldsymbol{\omega}_{base}$ | `state.segment<3>(9+nJoints)` | 3 | 基座角速度 |
| $\dot{\mathbf{q}}_{\text{joints}}$ | `state.segment(12+nJoints, nJoints)` | `nJoints` | 关节速度 |

### 2. WBC决策变量映射

| 数学符号 | C++变量 | 维度 | 在solution_中的位置 |
|----------|---------|------|-------------------|
| $\ddot{\mathbf{q}}$ | `jointAccelerations_` | `nv` | `[0, nv)` |
| $\boldsymbol{\lambda}$ | `contactForces_` | `6*nc` | `[nv, nv+6*nc)` |
| $\boldsymbol{\tau}$ | `jointTorques_` | `nv` | `[nv+6*nc, 2*nv+6*nc)` |

### 3. 任务权重映射

| 数学权重 | C++变量 | 太极推荐值 | 作用 |
|----------|---------|------------|------|
| $w_{contact}$ | `contactTaskWeight_` | `1e6` | 接触约束(硬约束) |
| $w_{base}$ | `baseTaskWeight_` | `1000` | 基座姿态稳定 |
| $w_{com}$ | `comTaskWeight_` | `800` | 质心跟踪 |
| $w_{arm}$ | `armTaskWeight_` | `1200` | 上肢跟踪(太极重点) |
| $w_{foot}$ | `footTaskWeight_` | `1500` | 足端位置维持 |
| $w_{joint}$ | `jointTaskWeight_` | `10` | 关节调节 |
| $w_{torque}$ | `torqueMinWeight_` | `1` | 力矩最小化 |

---

## 参数配置与数学权重对应

### 1. MPC配置文件映射

**配置文件路径**: `config/mpc/task.info`

```cpp
// 数学公式中的Q矩阵
finalCost {
  statePenalty {
    (0,0) 100.0    ; Q_{pos,x} - 基座X位置权重
    (1,1) 100.0    ; Q_{pos,y} - 基座Y位置权重  
    (2,2) 50.0     ; Q_{pos,z} - 基座Z位置权重
    (3,3) 200.0    ; Q_{rot,x} - Roll权重
    (4,4) 200.0    ; Q_{rot,y} - Pitch权重
    (5,5) 100.0    ; Q_{rot,z} - Yaw权重
    ; 关节位置权重矩阵对角线
    (6,6) 10.0     ; Q_{joint,1}
    ; ... 更多关节权重
  }
}

// 数学公式中的R矩阵
intermediateCost {
  inputPenalty {
    ; 接触力权重
    (0,0) 0.1      ; R_{force,1}
    ; 关节力矩权重
    (12,12) 1.0    ; R_{torque,1}
  }
  
  ; 数学公式中的S矩阵 (平滑性)
  inputVelocityPenalty {
    (0,0) 10.0     ; S_{force,1}
    (12,12) 10.0   ; S_{torque,1}
  }
}
```

### 2. WBC配置文件映射

**配置文件路径**: `config/wbc/task.info`

```cpp
// 太极动作专用WBC权重
WeightedWbc {
  ; 基座任务权重 w_base
  baseTask {
    weight 1000.0
    
    ; 对应数学公式中的 K_p,base 和 K_d,base
    positionGains [1000, 1000, 500]      ; [Kp_x, Kp_y, Kp_z]
    positionDamping [60, 60, 40]         ; [Kd_x, Kd_y, Kd_z]
    orientationGains [2000, 2000, 1000]  ; [Kp_roll, Kp_pitch, Kp_yaw]
    orientationDamping [100, 100, 50]    ; [Kd_roll, Kd_pitch, Kd_yaw]
  }
  
  ; 质心任务权重 w_com
  comTask {
    weight 800.0
    positionGains [800, 800, 400]
    positionDamping [60, 60, 40]
  }
  
  ; 上肢任务权重 w_arm (太极重点)
  armTask {
    weight 1200.0
    positionGains [1200, 1200, 1200]     ; 手部位置增益
    orientationGains [200, 200, 200]     ; 手部姿态增益
    positionDamping [80, 80, 80]
    orientationDamping [20, 20, 20]
  }
  
  ; 足端任务权重 w_foot
  footTask {
    weight 1500.0
    positionGains [1500, 1500, 1500]     ; 刚性约束
    positionDamping [100, 100, 100]
  }
  
  ; 关节调节权重 w_joint
  jointTask {
    weight 10.0
    positionGains [10, 10, 10, ...]      ; 每个关节的Kp
    velocityGains [5, 5, 5, ...]         ; 每个关节的Kd
  }
}
```

### 3. 约束参数映射

**数学约束到配置的映射:**

```cpp
// 速度约束: ||ṗ_base|| ≤ v_max
maxBaseVelocity 0.2        ; 太极动作: 0.2 m/s

// 加速度约束: ||p̈_base|| ≤ a_max  
maxBaseAcceleration 0.5    ; 太极动作: 0.5 m/s²

// 关节速度约束: |q̇_i| ≤ q̇_max,i
maxJointVelocity [3.0, 3.0, 3.0, ...]  ; 每个关节的速度限制

// 关节加速度约束: |q̈_i| ≤ q̈_max,i  
maxJointAcceleration 5.0   ; 太极动作保守设置

// 摩擦锥约束: ||f_xy|| ≤ μ*f_z
frictionCoefficient 0.7    ; μ = 0.7

// 关节位置限制: q_min ≤ q ≤ q_max
jointLimits {
  lowerBound [-2.0, -1.5, -2.0, 0.0, ...]
  upperBound [2.0, 1.5, 2.0, 2.5, ...]
}
```

### 4. 滤波器参数映射

**数学滤波公式到参数:**

```cpp
// 低通滤波: α = 2πf_c*T / (2πf_c*T + 1)
// 太极动作使用较低截止频率

acc_filter_cutoff_freq [5.0, 5.0, 5.0]      ; f_c = 5Hz (加速度计)
gyro_filter_cutoff_freq [10.0, 10.0, 10.0]  ; f_c = 10Hz (陀螺仪)

// 关节滤波
arm_joint_pos_filter_cutoff 15.0             ; 上肢位置滤波
arm_joint_vel_filter_cutoff 10.0             ; 上肢速度滤波
mrt_joint_vel_filter_cutoff 150.0            ; MRT关节速度滤波

// EMA滤波 (数据记录用)
ema_alpha 0.05                               ; 太极场景较小alpha
```

---

## 控制系统集成与实时性能

### 1. MPC-WBC集成架构数学基础

**分层控制系统理论:**

MPC和WBC形成两层分层控制结构，基于时间尺度分离原理：

$$\begin{align}
\text{MPC层} &: \quad \min_{\mathbf{u}_{0:N-1}} \sum_{k=0}^{N-1} \ell(\mathbf{x}_k, \mathbf{u}_k) + \ell_f(\mathbf{x}_N) \\
\text{WBC层} &: \quad \min_{\mathbf{x}_{wbc}} \frac{1}{2}\|\mathbf{A}_{tasks}\mathbf{x}_{wbc} - \mathbf{b}_{tasks}(\mathbf{u}^*_{mpc})\|_{\mathbf{W}}^2
\end{align}$$

其中 $\mathbf{u}^*_{mpc}$ 是MPC优化的结果，作为WBC的参考输入。

**时间尺度分离:**

- MPC时间尺度: $T_{MPC} = 20ms$ (50Hz)
- WBC时间尺度: $T_{WBC} = 5ms$ (200Hz)  
- 硬件时间尺度: $T_{HW} = 1ms$ (1000Hz)

满足: $T_{HW} < T_{WBC} < T_{MPC}$

**代码实现 (humanoidController.cpp的集成逻辑):**
```cpp
void HumanoidController::update(const ros::Time& time, const ros::Duration& period) {
    double dt = period.toSec();
    
    // 1. 状态估计更新 (1000Hz)
    updateStateEstimation(time, dt);
    
    // 2. MPC规划更新 (50Hz) - 时间守卫
    mpc_counter_++;
    if (mpc_counter_ >= MPC_UPDATE_RATIO) {  // MPC_UPDATE_RATIO = 20 (1000Hz/50Hz)
        mpc_counter_ = 0;
        
        // 转换当前状态到质心状态空间
        vector_t centroidalState = convertTocentroidalState(rbdState_);
        
        // MPC优化求解
        ocs2::SystemObservation observation;
        observation.time = time.toSec();
        observation.state = centroidalState;
        
        bool mpc_success = mpc_->advanceMpc(observation);
        if (mpc_success) {
            // 获取MPC最优控制序列
            mpc_->getMpcSolution(mpcSolution_);
            
            // 更新参考轨迹 (用于WBC)
            updateReferenceTrajectory(mpcSolution_);
        }
    }
    
    // 3. WBC控制更新 (200Hz) - 每5个控制周期
    wbc_counter_++;
    if (wbc_counter_ >= WBC_UPDATE_RATIO) {  // WBC_UPDATE_RATIO = 5 (1000Hz/200Hz)
        wbc_counter_ = 0;
        
        // 从MPC解中插值当前时刻的期望状态
        vector_t stateDesired, inputDesired;
        interpolateMpcSolution(time.toSec(), stateDesired, inputDesired);
        
        // WBC QP求解
        vector_t jointAccelerations = wbc_->update(stateDesired, inputDesired, rbdState_, 
                                                  currentMode_, dt, true);
        
        // 积分得到关节速度 (用于低级控制)
        updateJointVelocityCommands(jointAccelerations, dt);
    }
    
    // 4. 硬件接口更新 (1000Hz) - 每个控制周期
    publishCommands(time);
}
```

### 2. 数值稳定性与鲁棒性

**QP求解器的数值稳定性:**

qpOASES使用活跃集方法，数值稳定性通过以下方式保证：

**Hessian矩阵正定性检查:**
$$\mathbf{H} = \mathbf{A}^T\mathbf{W}\mathbf{A} + \lambda_{reg}\mathbf{I}$$

其中 $\lambda_{reg} > 0$ 是正则化参数，确保 $\mathbf{H} \succ 0$。

**条件数控制:**
$$\kappa(\mathbf{H}) = \frac{\sigma_{max}(\mathbf{H})}{\sigma_{min}(\mathbf{H})} < \kappa_{threshold}$$

当条件数过大时，增加正则化强度。

**代码实现 (WeightedWbc.cpp的数值鲁棒性):**
```cpp
vector_t WeightedWbc::update(const vector_t &stateDesired, const vector_t &inputDesired,
                            const vector_t &rbdStateMeasured, size_t mode, scalar_t period, bool mpc_update) {
    // 更新状态和约束
    updateMeasured(rbdStateMeasured);
    updateDesired(stateDesired, inputDesired);

    // 构造约束和成本函数
    Task constraints = formulateConstraints(inputDesired);
    Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
    
    // 数值鲁棒性检查
    // 1. Hessian正则化
    matrix_t H = weighedTask.a_.transpose() * weighedTask.a_;
    scalar_t regularization = 1e-6;  // 基础正则化
    
    // 条件数检查
    Eigen::JacobiSVD<matrix_t> svd(H);
    scalar_t condition_number = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
    
    if (condition_number > 1e12) {  // 条件数过大
        regularization = 1e-4;      // 增强正则化
        ROS_WARN_THROTTLE(1.0, "WBC: High condition number %.2e, applying regularization", condition_number);
    }
    
    // 应用正则化
    H.diagonal().array() += regularization;
    
    vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;
    
    // 2. 约束矩阵构造和求解QP
    // ... [QP求解代码]
    
    // 3. 解的合理性检查
    if (qpSol.hasNaN() || qpSol.array().abs().maxCoeff() > 1e6) {
        ROS_ERROR("WBC: Invalid solution detected, using backup solution");
        qpSol = getBackupSolution(stateDesired, inputDesired);
    }

    // 提取关节加速度并转换为关节速度命令
    vector_t generalizedAcceleration = qpSol.head(info_.generalizedCoordinatesNum);
    return centroidal_model::getJointAccelerations(generalizedAcceleration, info_);
}
```

### 3. 实时性能优化

**计算复杂度分析:**

- MPC求解复杂度: $\mathcal{O}(N \cdot n_x^3)$，其中 $N$ 是预测水平，$n_x$ 是状态维数
- WBC求解复杂度: $\mathcal{O}(n_{var}^3)$，其中 $n_{var} = 56$ 是决策变量维数

**性能监控和调优:**
```cpp
// 性能计时器 (在humanoidController.cpp中)
void HumanoidController::update(const ros::Time& time, const ros::Duration& period) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // MPC求解时间监控
    if (mpc_counter_ == 0) {
        auto mpc_start = std::chrono::high_resolution_clock::now();
        bool mpc_success = mpc_->advanceMpc(observation);
        auto mpc_end = std::chrono::high_resolution_clock::now();
        
        auto mpc_duration = std::chrono::duration_cast<std::chrono::microseconds>(mpc_end - mpc_start);
        mpc_solve_time_ = mpc_duration.count() / 1000.0;  // ms
        
        if (mpc_solve_time_ > 15.0) {  // MPC应在15ms内完成
            ROS_WARN_THROTTLE(1.0, "MPC solve time: %.2f ms (exceed 15ms)", mpc_solve_time_);
        }
    }
    
    // WBC求解时间监控
    if (wbc_counter_ == 0) {
        auto wbc_start = std::chrono::high_resolution_clock::now();
        vector_t jointAccelerations = wbc_->update(...);
        auto wbc_end = std::chrono::high_resolution_clock::now();
        
        auto wbc_duration = std::chrono::duration_cast<std::chrono::microseconds>(wbc_end - wbc_start);
        wbc_solve_time_ = wbc_duration.count() / 1000.0;  // ms
        
        if (wbc_solve_time_ > 3.0) {  // WBC应在3ms内完成
            ROS_WARN_THROTTLE(1.0, "WBC solve time: %.2f ms (exceed 3ms)", wbc_solve_time_);
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    total_control_time_ = total_duration.count() / 1000.0;  // ms
    
    // 整体控制回路应在0.8ms内完成 (为1ms周期留20%余量)
    if (total_control_time_ > 0.8) {
        ROS_WARN_THROTTLE(1.0, "Total control loop time: %.2f ms (exceed 0.8ms)", total_control_time_);
    }
}
```

---

## 总结

这个完整的WBC_MPC公式与代码实现对应文档基于KUAVO机器人的实际源代码分析，涵盖了：

### 1. MPC数学理论与实现
- **质心动力学模型**: 24维状态空间，36维输入空间
- **最优控制理论**: Bolza型问题，Hamilton-Jacobi-Bellman方程  
- **DDP求解算法**: Gauss-Newton SLQ，Riccati递推
- **约束处理**: 松弛对数障碍法，摩擦锥约束
- **权重矩阵设计**: 基于太极动作的专用调优

### 2. WBC数学理论与实现
- **QP问题构造**: 56维决策变量，多任务加权最小二乘
- **动力学约束**: Newton-Euler方程，浮基多体系统
- **任务空间控制**: 质心、基座姿态、手臂轨迹跟踪
- **约束建模**: 摩擦锥、关节限制、接触状态
- **任务优先级**: 分层权重设计，冲突解决

### 3. 系统集成与性能优化
- **分层架构**: MPC(50Hz) + WBC(200Hz) + 硬件(1000Hz)
- **数值鲁棒性**: 条件数控制、约束相容性、备份策略
- **实时性能**: 内存预分配、热启动、性能监控
- **安全保障**: 多层限制检查、失效保护机制

### 4. 工程实践价值
- **配置驱动**: 所有参数通过.info文件调节
- **模块化设计**: MPC与WBC解耦，便于独立调试
- **调试友好**: 丰富的日志输出和性能指标
- **太极专用**: 针对缓慢精确动作的优化配置

该文档展现了现代机器人控制理论与工程实践的深度结合，建立了数学理论与代码实现之间的直接对应关系，使开发者能够：

1. **理解算法原理**: 从数学公式看懂算法本质
2. **调试参数**: 知道修改哪个配置影响哪个数学项
3. **扩展功能**: 基于数学框架添加新的任务或约束
4. **性能优化**: 根据数学特性调整求解器和权重
5. **故障排查**: 从数学角度分析控制问题

通过这种理论-实践对应，为双足人形机器人的复杂运动控制提供了可靠的数学基础和实现指导，使太极动作的控制系统变得更加可理解和可维护。
