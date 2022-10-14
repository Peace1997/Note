# BicNet: Multiagent Bidirectionally-Coordinated Nets



## 一、简述

- 任务目标：协调多个智能体作为一个团队来击败他们的敌人。
- 创新点：
  - CommNet中负责通信的**全连接层替换为一个双向的RNN（Bi-RNN）**，不同于CommNet显式地在智能体之间进行通信，BiCNet通过双向RNN实现**隐层通信**。智能体之间的通信与关联均出现在隐层特征中，这样能够传递更高级的信息。
  - BiCNet与DIAL, CommNet的不同在于，BiCNet可以调整agents加入communication network的顺序，支持连续的动作空间。
  - Bi-RNN的引入使得智能体之间的**通信可以不完全对称**，优先级的不同可以学到更好的集体动作。
- 不足：BiCNet要求每个agent都能获取全局观测信息。而没有考虑更为实际的局部观测。



## 二、 算法框架

<img src="img/Paper14_1.png" alt="image-20220617154431143" style="zoom: 25%;" />

### 主要流程

整个模型是一个基于双向RNN网络的确定性Actor-Critic结构。BiNet使用了Bi-RNN替代MLP作为actor网络与critic网络的模型，但是其Actor网络的输入是全观测信息。BiCNet 同样遵循 CTCE 框架。



**Actor：**

对于每个Actor网络输入所有智能体的状态信息，即所有智能体共享同样的观测状态，actor网络和critic网络均使用了Bi-RNN框架，参数在不同智能体之间共享，最后会输出该智能体的行动决策。

**Critic：**

将actor网络的状态和行为作为输入，返回该智能体独立的Q值。

### 实现细节

在这里，隐藏特征不是在时间步之间流动，而是**在智能体之间流动**，即智能体之间会共享RNN的参数，Bi-RNN中包含一个前向层和一个反向层，因此隐藏特征会通过这两层网络双向流动。作者提出，Bi-RNN的存在不仅可以作为智能体之间的**通信信道**（相互传递隐层特征），还可以作为**局部记忆保存者**，每个智能体既可以保存隐层特征，也可以将其共享给其他协作者。



BiCNet 是为了解决 zero-sum Stochastic Game（SG）而提出的，本文中假设敌方的 policy 是固定的（即敌方智能体遵循一个固定的策略，只有己方智能体的策略是不断更新的），所以上述 SG 就转化为了一个 MDP，那么我们就可以使用类似 DDPG 这样的算法来去解这样一个 MDP 问题。由于所有的智能体都共享同一套参数，并且对于环境的观察也是共享的全局观察，那么对于特定任务（例如足球比赛），如何**体现智能体在角色上的差别呢**？BiCNet 算法做出了以下两点改进：

- 设计新的回报函数
  $$
  \begin{aligned}
  r_{i}(\mathbf{s}, \mathbf{a}, \mathbf{b}) \equiv & \frac{1}{\mid \text { top-K-u(i)|}} \sum_{j \in \text { top-K-u(i) }} \Delta \mathcal{R}_{j}^{t}(\mathbf{s}, \mathbf{a}, \mathbf{b})-\\
  & \frac{1}{\mid \text { top-K-e(i)|}} \sum_{i^{\prime} \in \text { top-K-e }(i)} \Delta \mathcal{R}_{i^{\prime}}^{t}(\mathbf{s}, \mathbf{a}, \mathbf{b})
  \end{aligned}
  $$

  > 其中第一项正比于敌方上一时刻到这一时刻被扣的血量，第二项正比于我方上一时刻到这一时刻被扣的血量。

- 固定智能体在 Bi-RNN 的位置

**智能体可以通过放在 Bi-RNN 的不同位置来体现其在任务中扮演的角色的不同**；而每个智能体使用不同的回报函数，只是增大了这种智能体之间的差异。



**BiCNet 中所有的智能体都拥有独立的回报函数以及 Q-network 以及 policy network，但这些 network 中部分参数是共享的。这些智能体一起在环境中进行数据采样，最后将所有的数据集中起来，更新他们的共享部分的参数。**



