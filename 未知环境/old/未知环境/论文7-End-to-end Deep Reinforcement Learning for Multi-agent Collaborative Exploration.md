### End-to-end Deep Reinforcement Learning for Multi-agent Collaborative Exploration

#### 面向多智能体协作探索的端到端的深度强化学习



### 1. 简述

- 整体流程：通过CNN对原始输入图像数据进行特征提取，然后将这些特征输入到PPO网络中得到每个智能体的动作，与此同时通过课程学习和内在动机加快训练，完成探索任务。

- 此前研究
  - 针对边界点的分配
  - 基于分段的探索

- 应用：
  - 救援任务和搜索
  - 监视
  - 扫地机器人
  - 收集物体

- 创新点：
  - CNN + ‘MA’ +  PPO
  
  - 课程学习 & 内在奖励
  
    > this paper is the first kind that applies deep reinforcement learning for multi-agent exploration in unknown environment with only visual observation as the inputs

### 2. CMAPPO Model

![image-20210913152439003](/Users/mapeixin/Documents/Typora/未知环境/img/Paper7_1.png)

> 已探索区域为绿色；未探索区域黑色；障碍物红色；智能体蓝色

- **主要过程**：首先使用CNN用于特征提取，再通过该PPO神经网络得出每个智能体的目标动作。
- **CNN**： 三个卷积层 + 三个全连接层
  - 第一个卷积层过滤 84x84x3的输入图片；其中输入包括智能体本身的通道$O^t_a$，其他智能体通道$O^t_{a'}$，障碍物通道$O^t_w$，未探索区域通道 $O^t_g$，探索区域通道$O^t_e$。 每个通道都对应着16个大小为8x8的滤波器，其中步长为4。
  - 第二个卷积层处理第一个卷积层处理过数据，他有32个大小为4x4的滤波器，其中步长为2
  - 第三个卷积层与第二个卷积层相同
  - 每一个卷积层后都对应着一个隐藏层，每个隐藏层包含256个神经元，每个神经元后使用的激活函数为ReLU
  - 最后在接三个全连接层，每个全连接层使用的激活函数也为ReLU

- **每个智能体通过学习更新同一个神经网络 CMAPPO**

  - **策略更新**： 
    $$
    L^{K L P E N}(\theta)=\mathbb{E}\left[\frac{\pi_{\theta}\left(a_{t} \mid s_{t}\right)}{\pi_{\theta_{o l d}}\left(a_{t} \mid s_{t}\right)} A_{t}\left(s_{t}, a_{t}\right)-\beta K L\left[\pi_{\theta_{o l d}}\left(\cdot \mid s_{t}\right), \pi_{\theta}\left(\cdot \mid s_{t}\right)\right]\right]
    $$

    > $A_t$：优势函数； $\pi_{\theta}$：策略  ； $a$：所选动作  ； $\beta$：用于下次策略更新的参数

    通过KL约束和优势估计来优化KL发散。在每次迭代中，通过所选的动作进行损失函数（$L^{K L P E N}(\theta)$）计算，该损失函数在K回合下的KL发散约束通过Adam优化器优化更新$\theta$

    

    广义优势估计函数的截断版本
    $$
    \hat{A}_{i}^{t}=\sum_{l=0}^{T_{i}}(\gamma \lambda)^{l-t+1} \delta_{i}^{l-1}, \delta_{i}^{t}=r_{i}^{t}+\gamma V\left(s_{i}^{t+1}\right)-V\left(s_{i}^{t}\right)
    $$

    - 折扣参数$\gamma$：0.7 ；

  - **值函数更新**：
    $$
    L^{V}(\phi)=-\sum_{i=1}^{N} \sum_{t=1}^{T_{i}}\left(\sum_{t^{\prime}>t} \gamma^{t^{\prime}-t} r^{t^{\prime}}-V_{\phi}\left(s_{i}^{t}\right)\right)^{2}
    $$

- **state space & action space & reward function**

  - **state space**：$o_{i}^{t}=\left(o_{a}^{t}, o_{a^{\prime}}^{t}, o_{w}^{t}, o_{g}^{t}, o_{e}^{t}\right)$

    - 它提供 84x84距离值矩阵作为每个通道的输入；共三层通道。$o_{i}^{t} \in \mathbb{R}^{3 x 84 x 84}$

  - **action sapce**：

    - 离散空间：上、下、左、右
    - 规则：每个智能体每次只能移动一个单元格；智能体只能在单个单元格的范围内扫描其周围。
    - 输出大小：4$i$ ；$i$为智能体数量

  - **reward function**：
    $$
    r_{i}^{t}=\left({ }^{g} \mathbf{r}\right)_{i}^{t}+\left({ }^{p} \mathbf{r}\right)_{i}^{t}+\left({ }^{f} \mathbf{r}\right)_{i}^{t}
    $$

    - 到达未探索区域奖励；0.2
    - 每个时间步的惩罚；0.001
    - 所有区域抖探索的奖励；1.0

- **算法**

  <img src="/Users/mapeixin/Library/Application Support/typora-user-images/image-20211118104005083.png" alt="image-20211118104005083" style="zoom:50%;" />



### 3. CMAPPO WITH  CURRICULUM

- **Curriculum learning**

  - 将**课程学习**方法应用于CMAPPO，以师生(tearcher-student)方式指导学习过程，包括不同的课程级别，以简化学习过程。

  - 课程—训练分布($Q_{\lambda}$)：
    $$
    Q_{\lambda}(z) \propto W_{\lambda}(z) P(z) \quad \forall z
    $$

    - $z$：随机变量
    - $P(z)$：目标训练分布
    - $ W_{\lambda}(z)$：在课程顺序的阶段级别$\lambda$上应用$z$的权重。 $ 0\leq\lambda \leq 1$

  - $Q_{\lambda}$分布的熵随$ W_{\lambda}(z)$的增加而增加，并且满足以下要求：
    $$
    H\left(Q_{\lambda}\right)<H\left(Q_{\lambda+\epsilon}\right) \quad \forall \epsilon>0
    $$

- **Intrunsic reward**

  - **内在奖励**：加权损失函数 —— 加速训练
    $$
    { }^{i} \mathbf{r}{=\eta L_{F}\left(\hat{\phi}\left(s_{t+1}\right), \phi\left(s_{t}\right)\right)}
    $$

    - $\eta$：比例因子 ； 1/128

    - $L_F$：前向模型的

    - $\hat{\phi}(s_{t+1})$：基于预测特征的**正向模型**
      $$
      \hat{\phi}(s_{t+1}) = f\left(\phi\left(s_{t}\right), a_{t} ; \theta_{F}\right)
      $$
      基于先前的状态$\hat\phi(s_{t})$和上一步采取的行动$a_t$，预测$s_{t+1}$状态的特征值；**$\theta_{F}$是$\hat{\phi}(s_{t+1})$特征的似然估计参数**

  

- **评价标准**

  - **总体奖励**
    $$
    \widehat{r}_{i}^{t}=r_{i}^{t}+\zeta\left({ }^{i} \mathrm{r}\right)_{i}^{t}
    $$

    - $\zeta \in [0,1]$：衡量内在奖励在总体奖励中的比例； 0.01

  - **反向模型**：$\hat{a}_{t}=\left(s_{t}, s_{t+1} ; \theta_{I}\right)$

    通过给定的两个连续状态，确定最优动作

  - **总体评价标准**
    $$
    \min _{\theta_{\pi}, \theta_{I}, \theta_{F}}\left[-\lambda \mathbb{E}_{\pi\left(s_{t} ; \theta_{\pi}\right)}\left[\Sigma_{t} r_{t}\right]+(1-\beta) L_{I}+\beta L_{F}\right]
    $$

    - $\theta_\pi$：总体奖励期望、$\theta_I$反向模型的似然估计、$\theta_F$：正向模型的似然估计
    - $L_I$反向模型误差、$L_F$：正向模型误差
    - $\beta \in [0,1]$：对正向模型和反向模型损失的重要性进行权衡；0.001？
    - $\lambda > 0$：对策略梯度损失和学习内在奖励信号的重要性进行权衡；0.95



### 4. 实验结果

- **实验内容**

  - **实验环境**：Unity；传感器：近似激光传感器；环境大小：20 x 20 单元格

  - **基本流程**：当智能体完成任务或超时(1000步 )时会重新初始化任务环境，即随机初始化地图(maze 、office)、随机放置智能体和障碍物。
  - **课程学习**：将智能体放入4钟不同的环境中，这些环境具有不同的复杂性或难度。 智能体会先探索简单环境，然后不断探索复杂环境，
  - **评价标准**：Average Moving Distance (AMD)；不同环境下，100次实验的平均移动单元数量

- **实验结果**

  - 若未使用课程学习和内在奖励无法很好完成全部探索;四个阶段代表难度不同的四个环境。

    ![image-20211119111213358](/Users/mapeixin/Documents/Typora/未知环境/img/Paper7_4.png)

  - **CMAPPO vs Frontier-based** 

    ![image-20211119111509585](/Users/mapeixin/Documents/Typora/未知环境/img/Paper7_3.png)

  相较于基于边界的探索算法，CMAPPO能够达到较好的协作，在不同的环境下，完成探索移动距离更短。

  CMAPPO可以直接从环境地图的**原始输入图像**中获取有效的协调策略，无需明确考虑其他智能体的效用或成本信息

  > Frontier-based边界探索算法思想为：根据已构建的地图环境信息，检测已知地图与未知地图之间的边界，选取与机器人距离最近的边界作为探索目标点；机器人探索完该目标点后，再根据更新的地图确定下一个探索目标点，以此不断迭代，直到完成对所有未知环境的探索。

### 5.问题：

- 虽然对未知探索区域进行探索，但是输入到神经网络的原始图像，还是包含了障碍物的信息。因此不是未知环境探索，所探索重建的区域都是安全的区域。
- 离散动作
- 环境较为简单，虽然环境有所不同，但是大小固定（20x20）
- 多智能体没有明显的合作和交流，而是通过输入的原始图像，直接获得接下来每个智能体要移动的位置。

