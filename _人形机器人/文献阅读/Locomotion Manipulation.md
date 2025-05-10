
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