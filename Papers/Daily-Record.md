

### 1. FRoM-W 1: Towards General Humanoid Whole-Body Control with Language Instructions

- **作者：** (待查完整名单, 核心技术源于 H-GPT 团队)
    
- **核心架构：** 两阶段框架。第一阶段 **H-GPT** 利用大规模人体运动数据生成语言驱动的全身动作；第二阶段 **H-ACT** 通过强化学习预训练（RPT）和微调（RFT）将人体动作重定向至机器人硬件。
    
- **解决的问题：** 如何让不同形态的人形机器人通用地理解自然语言指令并执行物理上可行的全身动作。
    
- **核心创新点：** 引入了 9 B 参数的大模型理解复杂指令，并结合 **RL 推理侧微调（Inference-phase Fine-tuning）** 动态修正动作跟踪误差。
    
- **实验结果：** 在 HumanML 3 D-X 榜单取得领先，并将动作跟踪误差（MPJPE）降低了 15%，显著提升了物理部署的稳定性。
    

### 2. FastStair: Learning to Run Up Stairs with Humanoid Robots

- **作者：** Yan Liu, et al.
    
- **核心架构：** 规划器引导的多阶段学习框架。将并行的“模型驱动足迹规划器”嵌入强化学习环路，并采用 **LoRA（低秩自适应）** 技术融合不同速度段的专家策略。
    
- **解决的问题：** 人形机器人在高速爬楼梯时难以兼顾灵活性（Agility）与稳定性（Stability）的固有矛盾。
    
- **核心创新点：** 克服了纯 RL 探索的盲目性，通过规划器先验限制搜索空间，使机器人能学习到物理可行的“奔跑爬坡”动作。
    
- **实验结果：** 在 **Oli 人形机器人**上实现 1.65 m/s 的高速爬楼，并成功挑战了 33 级螺旋楼梯。
    

### 3. Walk the PLANC: Physics-Guided RL for Agile Humanoid Locomotion on Constrained Footholds

- **作者：** (arXiv: 2601.06286)
    
- **核心架构：** 基于**控制李雅普诺夫函数 (CLF)** 奖励的物理引导强化学习。结合简化的步进规划器与数据驱动的自适应策略。
    
- **解决的问题：** 机器人在梅花桩、窄木梁等受限地形（Constrained Footholds）行走时，对感知噪声极其敏感且易发生灾难性跌倒。
    
- **核心创新点：** 摒弃了传统的手工奖励设计，通过 CLF 奖励确保策略始终朝着物理稳定的状态流形收敛。
    
- **实验结果：** 相比纯模型驱动或纯 RL 基准，在非连续地形上的行走成功率和鲁棒性有质的飞跃。
    

### 4. Uncertainty-Aware Robotic World Model Makes Offline Model-Based RL Work on Real Robots

- **作者：** (修订于 2026.01.08)
    
- **核心架构：** **RWM-U 架构**。扩展了自回归世界模型，引入认识不确定性（Epistemic Uncertainty）评估，并与 MOPO-PPO 算法结合。
    
- **解决的问题：** 离线强化学习（Offline RL）在人形机器人全身控制中存在的“误差累积”和“分布偏移”问题。
    
- **核心创新点：** 实现了在纯离线数据集上训练人形机器人策略，并能通过不确定性感知进行长时程（Long-horizon）预测。
    
- **实验结果：** 证明了仅靠离线数据训练的策略可在真实人形机器人上实现稳定的行走和操纵任务。
    

### 5. Embracing Bulky Objects with Humanoid Robots: Whole-Body Manipulation with RL

- **作者：** Chunxin Zheng, et al.
    
- **核心架构：** 集成了预训练人体运动先验与 **神经符号距离场（NSDF）** 的强化学习框架。
    
- **解决的问题：** 人形机器人如何搬运超过末端执行器抓取能力的大型重物（全身体抱/环抱任务）。
    
- **核心创新点：** 利用 NSDF 提供精确的几何感知，使机器人能利用手臂、胸部等多点接触（Multi-contact）增加负载能力。
    
- **实验结果：** 在模拟和实物实验中，成功完成了对多种形状、大小重物的“环抱搬运”，展示了极强的 Sim-to-Real 迁移能力。
    

### 6. It Takes Two: Learning Interactive Whole-Body Control Between Humanoid Robots

- **作者：** (arXiv: 2510.10206)
    
- **核心架构：** **Harmanoid 框架**。包含接触感知动作重定向模块和交互驱动运动控制器。
    
- **解决的问题：** 两个人形机器人之间、或人与机器人之间的物理协同操作（如共同抬桌子、社交互动）。
    
- **核心创新点：** 提出了**交互式奖励函数**，不仅关注个体稳定性，更强调多机协作中的接触相干性和物理一致性。
    
- **实验结果：** 实现了两台人形机器人步调一致的协同搬运，动作展现出极高的拟人度。
    

### 7. LIPM-Guided Reinforcement Learning for Stable and Perceptive Locomotion

- **作者：** (arXiv: 2509.09106)
    
- **核心架构：** 基于**线性倒立摆模型 (LIPM)** 引导的双判别器强化学习架构。
    
- **解决的问题：** 盲目搜索导致的 RL 步态不自然、无法应对户外复杂感知的稳定性难题。
    
- **核心创新点：** 引入 **Reward Fusion Module (RFM)**，根据实时环境反馈动态权衡“速度跟踪”与“身体平衡”。
    
- **实验结果：** 点足人形机器人在碎石地、斜坡等户外环境表现出极强的动态平衡能力。
    

### 8. Chasing Stability: Humanoid Running via CLF-Guided Reinforcement Learning

- **作者：** (arXiv: 2509.19573)
    
- **核心架构：** 非线性控制理论与深度强化学习的深度耦合。
    
- **解决的问题：** 人形机器人“单支撑相”与“腾空相”交替的动态奔跑控制。
    
- **核心创新点：** 将控制李雅普诺夫函数转化为可导的奖励项，使神经网络能学到具备**可证明稳定性（Certifiable Stability）**的奔跑策略。
    
- **实验结果：** 实现了具有明显腾空阶段的人形机器人奔跑，且能抵抗躯干受到的突发侧向冲击。
    

### 9. Language-directed Humanoid Whole-Body Control via End-to-end Learning

- **作者：** (RSS 2025/arXiv 最新版)
    
- **核心架构：** 基于 **Conditional VAE (CVAE)** 的端到端闭环控制模型。
    
- **解决的问题：** 语言指令到全身关节执行器电流/力矩的直接映射，减少中间抽象层的延迟。
    
- **核心创新点：** 在潜在空间（Latent Space）进行插值，实现了不同动作（如行走中挥手）之间的平滑过渡（Smooth Transition）。
    
- **实验结果：** 机器人能够实时响应如“快速向左跑并挥手”这类复合型、长程指令。
    

### 10. UniTracker: Learning Universal Whole-Body Motion Tracker for Humanoid Robots

- **作者：** (arXiv: 2507. Xxxxx, 2025 下半年热点)
    
- **核心架构：** 通用动作跟踪策略。采用 Transformer 架构处理高维全身状态序列。
    
- **解决的问题：** 传统 WBC 控制器在面对剧烈、非周期性动作时容易崩溃的问题。
    
- **核心创新点：** 将“跟踪任务”视为一种通用预测问题，使单一策略能适配多种不同比例的人形机器人硬件（Body-agnostic）。
    
- **实验结果：** 能够精确复现从跑酷到太极的各类复杂动作，成功率较传统方法提升 40%。
    













