
# HuB: 执行复杂平衡任务
`HuB: Learning Extreme Humanoid Balance`  - 2025.5
***Abstract***
为解决人形机器人在执行高难度平衡任务的难点，本文提出了一个名为 HuB（Humanoid Balance） 的运动框架。该框架主要包括运动重定向优化、运动平衡策略学习、Sim 2Real 三个模块，更偏向于工程化论文。

***Contribution***：
- **运动重定向动作优化**：
	- 初始对齐，将机器人模型与 SMPL 模型的初始姿态进行对齐
	- 质心过滤：将质心偏移支撑脚中心超过 0.2 m 的轨迹放弃
	- 足底纠正：将单脚支撑状态的时足底静止不动，避免打滑
	- 稳定化过渡：扩充复杂平衡任务前后的帧数，即复制参考运动的第一帧和最后一帧，使他们持续时间等于平衡任务的时间。
- **运动平衡策略学习**：
	- 松弛化的跟踪目标：并非要求策略严格跟踪参考轨迹，允许策略在参考轨迹附近优化调整，以探索到更稳定的运动模式。
	- 接触点约束惩罚：避免脚底发生意外滑动

- **Sim 2 Real**：
	-  局部参考跟踪：因为状态空间中包含质心投影，论文中弃用全局坐标系，建立以机器人躯干为中心的局部控制框架，因此参考数据和机器人在局部坐标系下对齐。
	- IMU 随机噪声：IMU 增加高斯白噪声
	- 高频扰动：对质心和关节增加随机推力

![[Pasted image 20250527144713.png]]


# FLAM: 基于模型的强化学习算法实现稳定姿态
`FLAM: Foundation Model-Based Body Stabilization for Humanoid  Locomotion and Manipulation` -2025
> Harbin Institute of Technology && Peng Cheng Laboratory

***Abstract***
该论文提出一种提升机器人身体稳定性的运控操作方法：FLAM。该方法将**稳定化奖励函数**和**基于模型的强化学习**（TD-MPC 2）算法进行结合，鼓励机器人学习稳定的姿态。

![[Pasted image 20250409173511.png|475]]


***Contribution***
- **稳定化奖励函数**：通过对比运动重构前后姿态差异，判断机器人姿态的稳定性。
- **基于模型的强化学习算法**：采用基于模型的强化学习算法 TD-MPC 2，该算法采用数据驱动的模型预测控制方法（MPC），通过时序差分（TD）训练潜在动力学模型和价值函数。
--- 

***A. Stabilizing Reward Function***
稳定性奖励函数可以帮助机器人学习稳定的姿态，主要分为两个部分：
- **Robot-human pose mapping**：将机器人的运动轨迹映射为含噪声的人类运动轨迹
- **Human motion reconstruction**: 通过一种基于人类运动重构基础模型（RoHM, a diffusion-based foundation model），将含有噪声的人类运动轨迹进行重构，生成稳定的人类运动轨迹。
稳定性奖励函数通过对比含噪声的人类运动轨迹和重构的稳定的人类运动轨迹进行奖励函数的设计：如果两者之间差异小于特定的阈值，则获得正奖励：
$$ \Gamma(s^i_h, \hat{s}^i_h) = \begin{cases} r_j, & \text{if } \| s^i_h - \hat{s}^i_h \| < t_j \\ 0, & \text{otherwise} \end{cases} $$
$$ R_S = \sum_{i=1}^{N} \Gamma(s^i_h, \hat{s}^i_h) \times r_j $$

![[Pasted image 20250409182619.png|375]]

![[Pasted image 20250409182847.png|375]]

---

***B. Basic Policy***

基准策略采用 [[RL for Robot#^fc94a5|TD-MPC2]] 算法与环境进行交互训练，该框架采用时序差分学习来训练潜在动力学模型和价值函数，TD-MPC2 包括五个组件：
-  **Encoder**: 对状态进行编码，用于生成潜在表示 $z = E(s)$ ，其中 $s$ 代表状态。
- **Latent Dynamics**: 利用潜在表示 $z$ 和动作 $a$ 预测下一个潜在状态 $z_{t+1} = D(z_t, a)$  
- **Reward Function**: 在潜在空间中使用奖赏函数 $\hat r= R(z, a)$ 来评估模仿行为的好坏。
- **Terminal Value**: 使用终止值函数 $qˆ = Q(z, a)$ 评估策略在长期的收益情况。
- **Policy Prior**: 基于潜在表示生成策略先验 $\hat a = P(z)$ . 
 
该策略框架在 FLAM 中主要用于根据稳定奖励（stabilizing reward）与任务奖励（task reward）来训练策略。FLAM 将稳定奖励与任务奖励组合起来引导策略的学习过程，以实现更高效更稳定性的任务完成。总体奖励函数计算公式为：

$$
R=R_T + \lambda \frac{q}{l_e} \cdot R_S
$$
- $R_T$：任务型奖励函数，主要指导智能体高效的完成设置的任务。
- $R_S$: 稳定性奖励，引导智能体在完成任务的同时保持身体的稳定性，使其更符合人类的运动模式，即偏风格性奖励。
- $\lambda$ : 缩放因子，平衡稳定性奖励所占比例，以满足不同任务的需求。
- $\frac{q}{l_e}$: $q$为特定任务下的期望的回报值，$l_e$为回合步长，$\frac{q}{l_e}$用于估计每一步的期望贡献。



# ULC：单一策略的全身运动控制
` ULC: A Unified and Fine-Grained Controller for Humanoid Loco-Manipulation` -2025 
> 哈尔滨工业大学 && 西湖大学

本文提出了一个名为 ULC （Unified Loco-Manipulation Controller）的全身运动框架，通过单一策略同时协调下肢运动和上肢操作。ULC 提出了包括序列技能学习、残差动作建模、指令多项式插值的方式来实现更高的动作协调性、精度与稳定性。

![[Pasted image 20250717171301.png]]

***Contribution：***
- **渐进课程学习**（Sequence Skill Acquisition）：任务课程学习和命令课程学习，从简单控制任务开始学习，只有当前课程任务学好后才到下一个任务。
- **残差动作建模**（residual action modeling）：提高命令跟踪能力，尤其针对手臂关节，对重力进行补偿。
- **指令多项式插值**（Command Polynomial Interpolation）：实现控制指令平滑过渡



# UniTracker: 
`# UniTracker: Learning Universal Whole-Body Motion Tracker for Humanoid Robots` -2025
> 上海交通大学  && 上海人工智能实验室

本文基于 Teacher-Student 框架提出一个通过单一策略实现全身运动控制的架构，UniTracker 的核心是将条件变分编码器（Conditional Variational Autoencoder ，CVAE)）整合至学生策略中，CVAE 将本体感知数据和参考数据映射成一个潜在变量的分布，再通过解码器几何当前观测和潜在变量分布生成动作。

![[Pasted image 20250718152318.png]]

***Contribution***
- 条件变分编码器建模：通过 CVAE 的潜在变量捕获从状态观测&参考运动到动作运动的模糊性，提高动作的合理性。

***CVAE 结构***
- 编码器 $\epsilon (z_t|p_t^{p-deploy}, s_t^{g-deploy})$：接收与教师策略相同的特权信息，描述期望的理论的运动分布
- 先验分布 $p(z_t|p_t^{p-deploy},s_t^{g-deploy})$ ,条件先验，同时结合自身的状态和参考运动，与直接只结合参考运动的方式相比，生成的语义抽象更符合自身情况。
- 解码器  $D(a_t|s_t^{p-deploy},z_t)$：动作生成，根据潜在变量和当前观测生成动作

> 编码器、先验分布和解码器是同时学习的

**编码器是被建模为先验分布的残差**：
- CVAE 的优化目标（ELBO）中会有一个 KL 散度项，它衡量了编码器学到的分布和先验分布的差异，**鼓励编码器学习一个接近先验分布的潜在空间**（编码器被建模为先验分布的残差），这样做的目的是有助于防止编码器学习到一个过于复杂或退化的潜在空间，从而提高泛化性。所以这个项是**在潜在空间的正则化和重构质量**进行进行平衡。
- 先验分布是可部署的参考信息，编码器输入是更全面的特权信息，KL 散度的作用是确保在特权信息下学到的潜在特征与可部署信息下定义的潜在空间保持一致性和合理性，同时允许编码器通过额外的特权信息进行更精确的调整，这种残差建模进一步强化了这一点，**即编码器学习的是对先验提供的基准的修正**。