### Deep Reinforcement Learning-Based Automatic Exploration for Navigation in Unknown Environment

### 未知环境下基于深度强化学习的自动探索导航

#### 1. 简述

- **任务完成**：

  本文提出基于地图构建模块、决策模块、规划模块的机器人**探索**的开发模块。该框架下每个模块相互独立实现。

- **问题解决**

  - POMDP
  - 自主探索
  - 加快
  - 收敛速度
  - 减少仿真环境到真实环境的差距

- 应用场景
  - 救援机器人的搜索任务
  - 未知环境下扫地机器人
  
  

#### 2. 3个模块

- **Decision module**

  - 主要功能：接收从地图模块获得的数据，发送下一个目标点给Planning module

  - 输入：机器人从0时刻到t时刻的位置（历史观测值）、当前t时刻的地图周边信息

    > 这里0—t时刻机器人的位置可以用循环神经网络代替

  - 输出：下一个目标点 

  - 方法：基于深度强化学习的AFCQN

- **Planning module**

  - 主要功能：计算从当前位置到目标位置的潜在轨迹，并将控制数据发送Mapping module

  - 输入：机器人从0时刻到t时刻的位置、目标点、t时刻的地图周边信息

  - 输出：从当前位置到目标机器人控制轨迹

  - 方法：

    - **A*** 搜索算法：输出离散点组成的路径 （全局规划）
    - timed-elastic-band（**TEB**） ：将路径转换为机器人的速度  （局部规划）
    - light detection and ranging（**LIDAR**）：提供实时点云信息；避障

    > 一般的调查框架分为两种：全局规划方法和局部规划方法。
    >
    > 1.全局规划：确定从当前位置到目标位置的最短路径。
    >
    > 2.局部规划：将路径转化为轨迹，并根据实时传感器数据调整轨迹

    > 相较于原始的轨迹跟踪，通过实时点云信息，避开构建好地图以外的障碍物

- **Mapping module**

  - 主要功能：依次接受传感器的数据，更新机器人的状态和地图信息
  - 输入：上一时刻(t-1)机器人的位置、上一时刻地图信息、当前时刻到控制信息和观测信息

  - 输出：当前时刻地图信息、当前时刻机器人所在位置

  - 方法：基于图的**Karto SLAM**

    

#### 3.AFCQN（Fully convolutional Q-network with an auxiliary task）

- **目标函数**
  $$
  c\left(\hat{m}, x_{t=0: T}\right)=\min _{u_{t=0: T}}\|m-\hat{m}\|^{2}+L\left(x_{t=0: T}\right) .
  $$

  > 目标函数：评价所建地图的不确定性
  >
  > $\hat{m}$：估算图
  >
  > $m$：实际图
  >
  > $L(.)$：时间从0时刻到t时刻路径长度
  >
  > **地图熵**：（Shannon熵）
  > $$
  > H(m)=-\sum_{i} \sum_{j} p\left(m_{i, j}\right) \log p\left(m_{i, j}\right)
  > $$
  > $p(m_{i,j})$：第i列第j行栅格图的占用概率
  >
  > 每个格子有三种状态：空闲、未知、被占用；注意与动作区分。

  

- **动作空间**

  - 动作是**离散**的；执行一次动作即在已有的动作空间中选择点。

  - 动作空间是基于栅格地图，采样的方法是光栅格化（rasterized sampling），动作空间由已采样的点组成。

    ![image-20210716211600398]( Paper5_1.png)

    > 白色区域（free area）为已探索区域；灰色区域为未探索区域
    >
    > 绿色点表示相对安全的动作；红色点为靠近障碍物的动作：紫色点表示未知区域的动作

    

- **奖励函数**
  $$
  r_{t}=\alpha\left(H\left(m_{t-1}\right)-H\left(m_{t}\right)-L\left(x_{t-1}, x_{t}\right)\right)
  $$

  > $\alpha$：稳定系数
  >
  > 来自最优路径的推导公式：
  > $$
  > \begin{aligned}
  > x_{t=0: T}^{*} &=\arg \min H\left(m_{T}\right)+L\left(x_{t=0: T}\right) \\
  > &=\arg \min H\left(m_{T}\right)-H\left(m_{0}\right)+L\left(x_{t=0: T}\right) \\
  > &=\arg \min \sum_{t=1}^{T}\left[H\left(m_{t}\right)-H\left(m_{t-1}\right)\right]+\sum_{t=1}^{T} L\left(x_{t-1}, x_{t}\right) \\
  > &=\arg \max \sum^{T}\left[H\left(m_{t-1}\right)-H\left(m_{t}\right)-L\left(x_{t-1}, x_{t}\right)\right]
  > \end{aligned}
  > $$

  为了安全性，增加启发式奖励：
  $$
  r_{t}= \begin{cases}-1 & \text { if the next point is in unknown space, } \\ -1 & \text { if the next point is too close to the obstacle. }\end{cases}
  $$
  为了及时停止探索，引入了终端动作，并定义了该行为的奖励
  $$
  r_{t}= \begin{cases}1 & \text { if the ratio of explored region in map } \rho>0.85 \\ -1 & \text { otherwise }\end{cases}
  $$

  > $\rho$：地图中探索区域的比例；平衡区域探索率和探索效率的超参数

- ==**AFCQN网络结构**==

  - **目的**：通过神经网络来估计地图中每个点的值。因为是**基于FCN**的，FCN对像素点进行分类，在这里将动作空间中的各个点看作是像素点对其进行分类。

  - **优点**：

    - 相较于原有的DNQs，**参数更少**，FCQN更容易训练，具有较少过拟合风险。

      > 原DQNs在特征图展平后接全连接层，因此引入了大量可训练参数，而FCQN只有几个卷积核参数。

    - DQNs因为采用全连接层，所以输入必须具有相同的维度，FCQN取消了全连接层，可以输入任意维度的图像。因此对**地图大小变化具有适应性**。

    ![image-20210719203246314]( Paper5_2.png)

    

  - **整体结构**

    - 开始时是**编码层**（encoder layers），即多个卷积池化操作，与常规DQNs的卷积层相同；其输出维度与动作空间相同。

    - 最上面第一个分支，利用**1x1的卷积层**计算出与上面编码器特征图大小相同的分数图；分数图的每一个元素都代表了动作空间中对应点的优势函数（Q值）。
      $$
      Q(s, a ; \alpha, \beta)=V(s ; \beta)+\left(A(s, a ; \alpha)-\frac{1}{|A|} \sum_{a^{\prime}} A\left(s, a^{\prime} ; \beta\right)\right)
      $$
      
    - 第二个分支，通过**最大池化层**操作来获取整个地图的特征向量；随后通过全连接层估计终端动作的Q值，
      $$
      Q(s, a ; \beta)=V(s ; \beta)+\left(A(s, a ; \beta)-\frac{1}{|A|} \sum_{a^{\prime}} A\left(s, a^{\prime} ; \beta\right)\right)
      $$
    
      > 这两个分支+前面的编码层被称为是**FCQN**；主要用于**估计Q值**
    
    - 第三个分支，增加了**解码层**，经过卷积池化后较小的特征图后，增加两个**反卷积层**，这样就可以实现输出的图片和输入的图片具有相同的维度。因为要评判预测地图的结果和真实地图的差距，所以还要增加一个**损失函数**。
      $$
      L_{s}=-\sum_{p} \sum_{c=1}^{M} y_{o, c} \log p_{o, c}
      $$
    
      >  $p$：地图中的各个像素；因为FCN是基于像素级的分类，因此需要对每个像素点进行分类。
      >
      > $p_{o,c}$：预测观测值o为标签 c 的概率
      >
      > $y_{o,c}$：二进制指标表，用来指示标签 c 是否为观测值o的正确分类。
      >
      > $M$：3种物体类别；障碍物轮廓、边界、其他。在该文中所有的像素点一共可分为3类。
    
  - **参数更新**
    $$
    \Delta w=-\epsilon\left(\frac{\partial L^{D Q N}}{\partial w}-\lambda \frac{\partial L_{s}}{\partial w}\right)
    $$
  
    > 其中$L^{DQN}$：$L^{D Q N}=E\left[\left(R_{t+1}+\max _{a^{\prime} \in A} Q\left(S_{t+1}, a^{\prime} ; \theta^{-}\right)-Q\left(S_{t}, A_{t} ; \theta\right)\right)^{2}\right]$
    >
    > $\theta$：学习率
    >
    > $\lambda$：分段任务的平衡参数
  
- **辅助任务（Auxliary task）**

  相较于**经验特征（empirical）**，把整个图片作为DRL的输入，造成搜索空间过大，本文提出了**边缘分割**作为FCQN辅助任务。它由两个元素组成：

  - 障碍物的轮廓线：避免在导航过程中发生碰撞
  - 自由空间和未知空间的边界（frontier），用来产生候选点。

  

#### 4. 系统流程

![image-20210719204534966]( Paper5_3.png)

1. 机器人通过扫描环境得到**测距法**数据以及点云数据，然后将数据发送到地图模块（Mapping module），本文采用**gmapping**的方法生成地图和姿态

   > 测距法：使用运动传感器的数据来估计位置随时间的变化
   >
   > gmapping：基于**2D激光雷达**使用RBPF(Rao-Blackwellized Particle Filters)算法完成**二维栅格地图**构建的**SLAM**算法。
   >
   > 主要由地图模块完成

2. 基于地图模块的结果，通过**FCQN、AFCQN**深度强化学习方法得到Q值，利用构建的地图信息和机器人的位置信息作为输入，输出下一个目标点（**贪婪策略**选择动作）。

   > 主要由决策模块完成

3. 对于全局规划模块，通过**A*算法**从地图模块接收生成的地图，从决策模块接受目标点，将其转换为由离散点组成的可行路径。对于本地规划模块，将路径通过**TEB**算法转换为速度，发送给机器人。

   > TEB：轨迹优化算法；对全局路径规划模块生成的处事轨迹进行后续的修订，从而优化机器人的路径轨迹。

4. 机器人在环境中移动，接收新的传感器数据并构建新的地图，不断迭代循环，直到达到迭代停止要求。



#### 5. 实验模拟

![](Paper5_4.png)

- **评价指标**：
  - 探测区域率：勘探期间所建地图完整性
  - 平均路径长度：评价移动效率
  - 探测效率：折中标准，单位长度所减少的熵

- **训练**：

  - DQN：因为有较多的参数，能够记住复杂的决策序列，因此学习较慢，但return略高于FCQN
  - AFCQN：边缘分割，可以加速探索，快速增加路径长度，但可能会存在一定的冗余动作，参数相较于DQN较少，因此收敛过快，AFCQN更注重**所建地图的完整性**。AFCQN的勘探区域率最高。
  - FCQN：相对于AFCQN避免了冗余运动，勘探率会高于AFCQN，同时平均路径相对于AFCQN也会短一些，不过他的探测区域率会低于AFCQN

- 测试

  - 地图：其中测试地图1、2大小与训练地图相同但布局不同；测试地图3、4与训练地图大小和布局各不同。

  - DQN：因为DQN采样的是全连接层限制了他的输出维数，因此DQN对不同尺寸的地图适应性较差。

  - FCQN：在布局相对宽敞的测试图1、3中的具有较好的探索效率，原因是FCQN能及时结束勘探。勘探路径较短。

  - AFCQN：添加了边缘分割作为辅助任务，AFCQN关注的是构建地图的完整性（AFCQN具有较高的探测区域率），从而导致探测区域过长，探测效率较低。AFCQN具有更好的完整检测能力。

  - 基于边界（frontier-based）：AFCQN的方法来源于基于边界的方法，虽然基于边界的方法的区域探索率略高于AFCQN，但有时存在失败的情况，即机器人和边界的中心会发生重合，此时停止探索，而AFCQN的输入包括机器人当前位置，以及上一个决策位置，因此可以避免发生这样的情况。

    > 基于边界的方法：决策点是从边界中心选取的，它不断探索环境，直到没有可到达点为止。

- **AFCQN探索过程的决策**

  - 此次共有八步，每个步骤分为三层，最下层是已建占用栅格图。中间层为边界图，最上层为Q值图

    > 边界图：因为使用边缘分割作为辅助任务，输入的信息主要包括两类边界信息，降低搜索空间
    >
    > Q值图：通过FCQN网络计算出的响应位置的Q值

  - 在决策的早期，有价值的决策在边界位置，随着地图的扩大，边界的大小也随之变大，AFCQN会向自由区域增加最快的地方学习。

  ![image-20210720154851539](/Users/mapeixin/Library/Application Support/typora-user-images/image-20210720154851539.png)

- **真实环境测试**：

  - 配置：万向节和底盘，底盘包括全向轮和类似激光雷达的RPlidar用于绘制图像
  - AFCQN方法具有较高的性能，其在虚拟环境和真实环境的轨迹比较一致。而DQN对输入很敏感，很容是失败。

![](Paper5_6.png)

#### 评价

- 创新点：将FCN网络与DQN相结合，同时加入边缘分割作为辅助任务来训练特征提取辅助网络，提高了训练效率和泛化性能。
- 可改进：
  - 动作空间是离散的
  - 增加循环神经网络增强记忆性
  - 增加领域知识





A*

FCN

启发式

信息论：

POMDP

香农熵

LSTM



内在好奇心模块（ICM）+ A3C + 策略熵

规划单元 planning module

SLAM（filter-based；graph-based；神经网络拟合函数）

内在奖励

辅助任务

> Some researchers proposed auxiliary tasks by enhancing prior knowledge to improve training efficiency. Mirowski et al. [22] implemented depth prediction and loop closure prediction as the auxiliary task to improve the performance and training efficiency of DRL algorithms. Lei et al. [20] trained the deep convolutional network with classification task and then exploited this network to extract the features for the RL. The feature input drastically reduces the state dimension and exploration time instead of the original image input.
> Unfortunately, these features are usually not the task-oriented, there is no guarantee for the quality of the features.

frontier-based



一般的调查框架分为两种：全局计划方法和局部计划方法。

1.全局规划：确定从当前位置到目标位置的最短路径。

2.局部规划：将路径转化为轨迹，并根据实时传感器数据调整轨迹

最近提出了规划单元的方法，将传感器数据和目标点作为输入，建立从输入到输出的映射。



