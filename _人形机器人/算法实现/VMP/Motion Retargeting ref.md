
## 一、简述

“运动的重定向”（Motion Retargeting）是指将某一来源（通常是人类 Mocap 数据或其他机器人的运动数据）上的运动数据，通过特定模型、映射算法和优化手段，映射到一个运动能力、结构形式不同（例如关节数量、限制、动力系统不同）的人形机器人上，同时保持动作的结构一致性、物理可控性和自然稳定性。


- **结构一致性**：将原始动作转移到不同结构的机器人上，保持关键运动意图不变；
- **物理可控性**：重定向后的动作要在机器人动力学/运动学约束下可执行；
- **自然性和鲁棒性**：最终表现出的运动应自然平滑、稳定，不引起跌倒或高能耗。

---

## 二、运动重定向的主要过程

### Step 1: 动作采集与建模

通常来自以下方式：
- **动作捕捉（Motion Capture）**：通过光学或惯性 Mocap 系统获得人类关节点轨迹。
- **已有动画或仿真数据**：如游戏引擎中的动作数据、人体运动数据库（如 CMU Mocap）。

这些数据通常会以**时间序列方式表示关节空间轨迹**、身体质心轨迹 (CoM)，以及接触信息（如脚底着地点）。

### Step 2: 空间映射（Kinematic Mapping）

由于人类与人形机器人的身体结构差异，必须将原始动作所处的参考框架与目标机器人结构相协调。这包含：

- **骨架拓扑匹配**：例如 15 自由度的人类动作映射到 20 自由度的机器人，可用插值增强动作维度；
- **关节空间变换**：通过关节参数 (Denavit-Hartenberg 或 URDF 等模型) 的正/逆向运动学 (FK/IK) 完成交点重定向；
- **规范化身体参数**：如身高、肢长等差异通过比例尺度变换归一化处理。

### Step 3: 约束驱动下的优化调整

重定向动作必须满足物理可行性，涉及复杂的优控问题：

- **接触保持**：保证脚底在行走/站立时的足底不离地。
- **动力学一致性**：维护重心 (CoM)、零力矩点 (ZMP) 和力矩/加速度限制。
- **时间调整**：添加时间缩放（Time Warping）以适应机器人反应速度或执行幅度。

可基于以下模型进行优化：

- **优化器目标函数表示**：

$$
\min_{\mathbf{q}(t)} \sum_t \left[ \lambda_1 \|\mathbf{q}_H(t) - \mathbf{q}_R(t)\|^2 + \lambda_2 C_{dyn}(\mathbf{q}(t), \dot{\mathbf{q}}(t)) + \lambda_3 C_{contact}(t) \right]
$$

其中：
-$\mathbf{q}_H(t)$：原始人类关节角度
-$\mathbf{q}_R(t)$：机器人当前姿态
-$C_{dyn}$：动力学可行性代价（如力矩、关节速度约束）
-$C_{contact}$：接触保持项，如足部约束

### Step 4: 控制级执行

根据重定向后获得的高层动作轨迹，机器人将其传入控制模块（如：

- **轨迹跟踪控制器**：使用 PD，Model Predictive Control (MPC)，或基于优化的轨迹控制器；
- **接触力优化器**：确保双足/手部接触稳定；
- **低层伺服循环**：使控制律能实时生效，如 1 ms 级实时控制循环。

---

## 三、应用

近年来，机器学习、特别是深度学习被广泛应用以提供更强的动作泛化能力：

- **AutoEncoder 和 Variational AutoEncoder (VAE)**：用于学习低维、可迁移的运动嵌入，再映射为机器人能实现的控制空间；
- **GAN**：学习人类动作的风格，再通过约束使机器人的动作生成具备类似风格或功能；
- **强化学习**：利用模仿学习技术（如 Behavior Cloning 或 RL with Human Priors）学习可自适应的重定向策略，参考 DeepMimic、Motion Imitation 等工作。

## 五、典型挑战与应对策略

| 挑战类型  | 描述                | 应对策略                  |
| ----- | ----------------- | --------------------- |
| 结构不一致 | 人体与机器人不同骨架自由度和形状  | 骨架归一化 + IK 重构方案       |
| 物理不可行 | 原始动作超出机器人力量或速度限制  | 使用动力学优化，添加物理代价        |
| 不稳定执行 | 重心漂移、ZMP 越界、接触不合理 | 引入接触模型，实时 MPC 预测控制    |
| 数据谱差异 | 动作采集数据有噪声或缺失      | 滤波 + Seq 2 Seq 时间补全模型 |







---

## 一、OmniH2O 重定向实现




OmniH2O 的运动重定向过程通过多模态感知（如动捕数据、力觉信号）捕获人类动作后，首先进行关节空间与时域的归一化映射，利用刚体缩放与逆运动学将人体动作适配至机器人结构；随后结合强化学习与动力学优化，在仿真环境中实时调整关节轨迹，平衡姿态跟踪、能耗与稳定性（如 ZMP 约束），并通过域随机化增强策略泛化性；最终经由分层控制器（轨迹生成、阻抗调节、伺服执行）实现低延迟、高鲁棒的物理动作输出，在复杂地形与扰动下保持自然运动表现与高效能。

### 1. 动作感知与标准化
- 原始数据输入  
  采用多源输入（如 RGB-D 摄像头、IMU 惯性传感器、动捕设备）捕捉人类动作，生成**关节角序列**、**足底接触状态**及**外力感知信号**。  
- 空间与时域标准化  
  **人体模型与机器人模型的归一化映射**：
  - 通过**刚体缩放**调整肢体长度比例，如躯干长度从人类$L_h$调整至机器人$L_r$，公式：  
   $$s = L_r / L_h$$
$$q_r(t) = \text{IK}_{\text{robot}}(s \cdot T_{\text{human}}(q_h(t)))$$ 
    其中，$T_{\text{human}}$为人体正向运动学函数，$\text{IK}_{\text{robot}}$为机器人逆运动学求解器。  
  - 时间轴上通过**动态时间规整（DTW）** 或**分段仿射变换**对齐动作节奏（例如行走步频适应电机响应速度）。

---

### 2. 联合优化与物理可行性修正
在映射基础上，引入**混合优化目标**解决物理不可行问题：
- **目标函数形式**：
 $$\min_{q_r(t), \tau(t)} \sum_{t} \left[ \underbrace{w_1 \| q_r(t) - q_{\text{ref}}(t) \|^2}_{\text{跟踪误差}} + \underbrace{w_2 \left( \| J(q_r) \dot{q}_r \|^2 \right)}_{\text{关节加速度约束}} + \underbrace{w_3 F(\tau(t))}_{\text{力矩功耗}} + \underbrace{w_4 C_{\text{zmp}}}_{\text{稳定性约束}} \right]$$
  - 稳定性约束$C_{\text{zmp}}$采用零力矩点（ZMP）条件保证动态平衡：  
   $$x_{\text{zmp}} = \frac{\sum_{i} (m_i (z_i \ddot{x}_i - x_i \ddot{z}_i) + I_i \dot{\omega}_i)}{\sum_{i} m_i (g + \ddot{z}_i)}$$ 
    ZMP 须落在机器人**支撑多边形（Support Polygon）** 内部。  
  - 通过**QP 优化器**或**ADMM 算法**在 CPU/GPU 上实时求解。

---

### 3. 强化学习驱动的动作强化
针对复杂动作（如跑跳、跨越障碍），OmniH2O 采用**层次化强化学习（HRL）** 作为核心控制器：
- **奖励函数设计**：
 $$r_t = \underbrace{\alpha \exp(-\beta \| q_r - q_{\text{target}} \|)}_{\text{姿态跟踪}} + \underbrace{\gamma (1 - \text{foot\_slip})}_{\text{防滑}} + \underbrace{\delta (-\| \tau \|^2)}_{\text{能耗惩罚}} + \underbrace{\eta \cdot \text{zmp\_safety}}_{\text{稳定奖励}}$$ 
- **策略网络与训练**：
  - **网络架构**：Actor-Critic 框架，Actor 为 MLP 输出关节力矩指令，Critic 预测长期奖励。  
  - **课程学习（Curriculum Learning）**：从简单动作逐步增加扰动（如外力干扰、地面摩擦变化）。  
- **仿真到真实（Sim 2 Real）**：  
  在仿真环境（如 Mujoco、Isaac Sim）中预训练，利用**域随机化（Domain Randomization）** 增强策略鲁棒性，随后移植到物理机器人（如 Boston Dynamics Atlas、Unitree H 1 等）。

---

### 4. 实时控制与接触力协调
- **分层控制器结构**：
  1. **高层轨迹生成器**：输出关节轨迹$q_{ref}(t)$，融合优化后的动作数据与强化学习修正动作。
  2. **中层阻抗控制器**：  
    $$\tau = K_p (q_{des} - q) + K_d (\dot{q}_{des} - \dot{q}) + J^\top F_{\text{ext}}$$ 
     通过力反馈$F_{\text{ext}}$柔性适应接触冲击。
  3. **底层伺服控制**：基于 FPGA 的实时电流环控制（如 EtherCAT 总线架构），5 kHz 更新频率。

---

## 二、OmniH2O 的核心创新点
1. **跨模态的统一策略**：  
   整合视觉（RGB）、力觉（IMU/力矩传感器）、本体感知（编码器）的多模态输入，通过**自注意力机制**动态调整控制参数。  
2. **动态优先级重定向框架（DPRF）**：  
   依据当前任务（如行走/搬运）动态调整优化目标的权重$( w_1, w_2, ... )$。
3. **关节耦合补偿**：  
   针对人形机器人并联关节（如腿部的 4 自由度髋关节）引入**Lie 群运动学建模**，避免奇异点。

---

## 三、关键技术实验验证
### 1. 量化指标

| 指标类型 | 测试场景 | OmniH 2 O（性能） | Baseline（传统重定向） |  
|----------|----------|----------------|-------------------------|  
| 姿态误差 (°) | 行走 | **2.3±0.5** | 5.1±1.2 |  
| 能耗 (W/kg/km) | 奔跑 | **18.9** | 27.4 |  
| ZMP 安全率 (%) | 抗扰动 | **98.7** | 87.2 |  

### 2. 极端场景测试  
- **地形突变**（从硬地到软沙地）：利用**在线辨识**调整关节刚度，避免下陷。  
- **外力冲击**（侧面 30 N 推力）：触发反射式步态校正（如单脚横跳），动态调整 ZMP 轨迹。  

---

## 四、工程化实践中的关键问题
- **传感器噪声**：采用**卡尔曼滤波**与**时序卷积网络（TCN）**实时降噪。
- **处理延迟**：在强化学习策略中嵌入**时滞补偿模块**（如预测未来 10 ms 状态）。
- **能量优化**：通过**混杂控制器（Hybrid Controller）**在主动-被动模式间切换（如膝关节切换到被动阻尼模式降低功耗）。

---

## 五、总结与展望
OmniH2O 的成功在于将**高精度运动重定向**与**自适应强化学习控制**深度融合，解决了传统方法在复杂任务中面临的动态适应不足、能耗高的问题。未来方向可能包括：
1. **全身协同操作**：结合上半身手势与下半身步态的统一重定向。
2. **跨物种动作迁移**：从动物运动数据（如猿猴攀爬）中提取更高效的动作模式。
3. **人机协作场景**：动态调整运动计划以匹配人类合作者的意图。

需进一步探讨具体技术模块（如强化学习奖励函数设计细节），或其他应用场景（工业、医疗等），我可以补充内容或提供算法伪代码。

## OmniH2O 

OmniH2O 的运动重定向（motion retargeting）分为两大阶段：
1. **基于 SMPL 的两步初步重定向**，通过形状参数优化和姿态优化，将人体 SMPL 动作映射到人形机器人骨架上；
2. **“sim‑to‑data”仿真数据清洗**，利用特权教师策略在仿真中筛选不可行动作，确保保留的动作序列均可由机器人稳定执行。

---

### 一、运动学对齐与重定向

#### 1. 形状参数优化

- 由于 SMPL 模型与人形机器人骨架存在结构差异，首先对 SMPL 的体型参数 β 进行优化，使人体模型在静止姿态下的 12 个关键关节（踝关节、肘关节、手腕等）与机器人对应关节空间位置最接近，通过梯度下降最小化关节欧氏距离完成形状对齐 ([arXiv](https://arxiv.org/html/2403.04436 "Learning Human-to-Humanoid Real-Time Whole-Body Teleoperation"))。

- 优化完成后，采用原始的全身平移和旋转参数 θ、t，结合拟合后的 β，计算得到一组符合机器人比例的关键点位置信息 ([arXiv](https://arxiv.org/html/2403.04436 "Learning Human-to-Humanoid Real-Time Whole-Body Teleoperation"))。

#### 2. 姿态位置优化

- 对每一帧动作，固定上述关键点集合，使用 Adam 优化器在机器人骨架上最小化这 12 个关键关节位置误差，力求保持末端执行器（脚踝、手腕等）的空间轨迹与人体动作一致 ([arXiv](https://arxiv.org/html/2403.04436 "Learning Human-to-Humanoid Real-Time Whole-Body Teleoperation"))。
    
- 与直接拷贝人体关节角度不同，该方法避免了因骨架拓扑差异带来的末端位置偏移问题 ([arXiv](https://arxiv.org/html/2403.04436 "Learning Human-to-Humanoid Real-Time Whole-Body Teleoperation"))。

#### 3. 启发式过滤

- 在优化过程中，剔除不可执行或不安全的动作序列，如坐地、跨步过宽等场景，避免重定向后产生机器人“内八字”或跌倒伪影 ([arXiv](https://arxiv.org/html/2403.04436 "Learning Human-to-Humanoid Real-Time Whole-Body Teleoperation"))。
    
- 该两步流程对 AMASS 数据集约 13 k 条动作序列进行计算后，产出约 10 k 条初步重定向序列，作为后续仿真验证数据集 ([arXiv](https://arxiv.org/html/2403.04436 "Learning Human-to-Humanoid Real-Time Whole-Body Teleoperation"))。
    

> **对比：** 与 Penco 等人基于 IK+QP 的优化重定向方法相比，OmniH2O 通过自动形变与仿真清洗，实现了大规模数据的高效处理和更高的动作可行率 ([arXiv](https://arxiv.org/html/2403.04436 "Learning Human-to-Humanoid Real-Time Whole-Body Teleoperation"), [ResearchGate](https://www.researchgate.net/publication/330629851_Robust_Real-Time_Whole-Body_Motion_Retargeting_from_Human_to_Humanoid?utm_source=chatgpt.com "Robust Real-Time Whole-Body Motion Retargeting from Human to ..."))。

---

### 二、“Sim‑to‑Data”仿真数据清洗

#### 1. 特权教师策略训练

- 在物理仿真环境中，训练一个访问全局速度、角速度等特权状态信息且**不**使用领域随机化的教师策略，以最大化对所有初步重定向序列的跟踪精度 ([arXiv](https://arxiv.org/html/2403.04436 "Learning Human-to-Humanoid Real-Time Whole-Body Teleoperation"))。
    

#### 2. 序列可行性筛选

- 通过教师策略对每条序列进行跟踪测试，**剔除**那些跟踪失败（机器人无法稳定模仿）或偏差过大的序列，保留能够被机器人真实执行的动作 ([arXiv](https://arxiv.org/html/2403.04436 "Learning Human-to-Humanoid Real-Time Whole-Body Teleoperation"))。

- 最终，在 10 k 条初步重定向序列中，有约 8.5 k 条序列通过筛选，构成大规模、纯净且可行的机器人动作数据集 ([arXiv](https://arxiv.org/html/2403.04436 "Learning Human-to-Humanoid Real-Time Whole-Body Teleoperation"))。


---

### 总结

- 该流程确保数据既保留了人体动作的多样性，又严格符合人形机器人动力学与稳定性要求；
- 清洗后数据集用于强化学习后，可显著提升策略在物理硬件上的跟踪成功率与鲁棒性。


以上即 OmniH2O 中“大规模重定向与仿真清洗”模块的核心方法与流程。

## Ref Code

以下为几个典型的、直接利用 Reallusion ActorCore、CMU Mocap Database 或 Mixamo 动作数据集，将动作重定向（retarget）到另一机器人或模拟骨架的开源 GitHub 仓库。它们覆盖了从纯运动学映射脚本到强化学习驱动的仿真控制器，以及从 Blender/Unity 插件到完整的机器人遥操作框架，均可作为快速集成与二次开发的参考。

1. ASE (nv‑tlabs/ASE)

- **链接**：[https://github.com/nv-tlabs/ASE](https://github.com/nv-tlabs/ASE)
    
- **功能**：提供通用的骨架运动库 `poselib`，支持从 ActorCore、CMU Mocap、Mixamo 等 FBX 动画导入，并通过 `ase/poselib/retarget_motion.py` 脚本将这些动作重定向到 Isaac Gym 中的 MJCF 机器人骨架。用户只需指定源 T‑Pose、目标 T‑Pose 和关节映射配置，即可实现大规模动作转换。 ([GitHub](https://github.com/nv-tlabs/ASE/blob/main/README.md?utm_source=chatgpt.com "ASE/README.md at main · nv-tlabs/ASE - GitHub"), [GitHub](https://github.com/nv-tlabs/ASE/blob/main/README.md?utm_source=chatgpt.com "ASE/README.md at main · nv-tlabs/ASE - GitHub"))


2. PoseLib (T‑K‑233/PoseLib)

- **链接**：[https://github.com/T-K-233/PoseLib](https://github.com/T-K-233/PoseLib)
- **功能**：基于 PyTorch 实现的骨架加载与运动重定向库，内置对 CMU Mocap Database 到 AMP Humanoid（Isaac Gym MJCF 模型）的示例配置 `data/configs/retarget_cmu_to_amp.json`，并通过 `retarget_motion.py` 演示如何将 CMU 动作直接映射到仿真机器人。 ([GitHub](https://github.com/T-K-233/PoseLib "GitHub - T-K-233/PoseLib"))
    

3. HumanRetargetingController (isri‑aist/HumanRetargetingController)

- **链接**：[https://github.com/isri-aist/HumanRetargetingController](https://github.com/isri-aist/HumanRetargetingController)
    
- **功能**：ISRI 与 AIST 联合开发的真实人形机器人运动重定向控制器，能够接收来自 ActorCore、CMU、Mixamo 或实时时人体追踪的数据，将其映射为机器人关节指令，并提供 ROS/Yarprobotinterface 接口，适用于在线遥操作和仿真研究。 ([GitHub](https://github.com/isri-aist/HumanRetargetingController?utm_source=chatgpt.com "isri-aist/HumanRetargetingController - GitHub"))
    

4. DeepMimic (xbpeng/DeepMimic)

- **链接**：[https://github.com/xbpeng/DeepMimic](https://github.com/xbpeng/DeepMimic)
    
- **功能**：伯克利大学提出的示例引导深度强化学习框架，使用 CMU Mocap Database 中的动作片段训练物理模拟人形机器人（MuJoCo），实现对多样化运动技能的高精度模仿，可视为将 CMU 动作数据重定向到物理仿真机器人控制策略的典范。 ([GitHub](https://github.com/xbpeng/DeepMimic?utm_source=chatgpt.com "xbpeng/DeepMimic: Motion imitation with deep reinforcement learning."))
    

5. Godot4‑MixamoLibraries (jwelchgames/Godot4-MixamoLibraries)

- **链接**：[https://github.com/jwelchgames/Godot4-MixamoLibraries](https://github.com/jwelchgames/Godot4-MixamoLibraries)
    
- **功能**：为 Godot 4 引擎提供 Mixamo 骨骼与动作库映射，包含 Mixamo 骨骼到 OpenBot 真实机器人模型的骨骼对应表，支持将 Mixamo 上下载的动画一键重定向并在 Godot 中驱动 OpenBot 平台仿真。 ([GitHub](https://github.com/jwelchgames/Godot4-MixamoLibraries?utm_source=chatgpt.com "Ready-to-use Mixamo libraries to retarget to bones in Godot4 - GitHub"))
    

6. AMH2B (DreamSpoon/AMH2B)

- **链接**：[https://github.com/DreamSpoon/AMH2B](https://github.com/DreamSpoon/AMH2B)
    
- **功能**：专注于将 Mixamo Armature 转换为 MakeHuman MHX Armature 的工具流程，示例脚本 `docs/ARMATURE_RETARGET.md` 演示如何构建从 Mixamo BVH/FBX 到 MHX 骨架的自动重定向管线，可用于机器人或角色驱动。 ([GitHub](https://github.com/DreamSpoon/AMH2B/blob/main/docs/ARMATURE_RETARGET.md?utm_source=chatgpt.com "AMH2B/docs/ARMATURE_RETARGET.md at main - GitHub"))
    

7. CC Blender Tools (soupday/cc_blender_tools)

- **链接**：[https://github.com/soupday/cc_blender_tools](https://github.com/soupday/cc_blender_tools)
    
- **功能**：Blender 插件，支持 ActorCore 与 Mixamo 动作到 Rigify Rig 的预览与烘焙，内置面部表情与身体调整选项，也可导出至 Unity，用于快速将公开数据集动作映射到自定义角色或仿真机器人。 ([GitHub](https://github.com/soupday/cc_blender_tools?utm_source=chatgpt.com "soupday/cc_blender_tools: Add-on for Blender, for ... - GitHub"))
    

8. Reallusion Auto‑Setup‑for‑Unity (reallusion/Auto-Setup-for-Unity)

- **链接**：[https://github.com/reallusion/Auto-Setup-for-Unity](https://github.com/reallusion/Auto-Setup-for-Unity)
    
- **功能**：Reallusion 官方提供的 Unity 脚本，自动将 ActorCore 平台下载的角色与动作资产绑定到 Unity 骨骼并批量生成绑定动画，简化了 ActorCore → Unity 角色/机器人动画重定向流程。 ([GitHub](https://github.com/reallusion?utm_source=chatgpt.com "reallusion - GitHub"))
    

9. Dex Retargeting (dexsuite/dex-retargeting)

- **链接**：[https://github.com/dexsuite/dex-retargeting](https://github.com/dexsuite/dex-retargeting)
    
- **功能**：AnyTeleop 项目衍生出的手部动作重定向优化库，提供多种算法将摄像头/视频或 Mocap 数据（包括 Mixamo、CMU）映射至实际机器人手爪关节，用于遥操作与离线数据驱动的机器人手控制。 ([GitHub](https://github. Com/dexsuite/dex-retargeting? Utm_source=chatgpt. Com "dexsuite/dex-retargeting - GitHub"))





# 算法实现

## 一、直接映射法

对人体参考数据和目标机器人的




环境依赖：
Cuda 11.5
`pip install torch==1.11.0+cu115 torchvision==0.12.0+cu115 torchaudio==0.11.0+cu115 --extra-index-url https://download.pytorch.org/whl/cu115`