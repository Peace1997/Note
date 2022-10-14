## Voronoi-Based Multi-Robot Autonomous Exploration in Unknown Environments via Deep Reinforcement Learning

### 基于Voronoi的多机器人深度强化学习在未知环境下的自主探索



### 1. 简述

- 任务目标：针对未知环境下的多智能体系统，提出了基于协同探索策略和深度强化学习的无地图碰撞算法。

  

- 介绍：该实验为未知环境下基于Voronoi和深度强化学习的多移动机器人自主探索，为了更加有效率的完成多移动机器人协作探索任务，使用了一种分层控制结构，该结构包括一个高层决策层和低层目标跟踪层。其中高层决策层通过Voronoi分区和选点公式为每个移动机器人分配不同的目标位置来最小化重复的探索；低层目标跟踪层使用基于深度强化学习的方法来使每个移动机器人无碰撞的抵达相应的目标位置。



- 多智能体对比单智能体优势
  - 减少任务探索时间
  - 增加整个系统容错率
  - 提高本地化效率
- 难点
  - 避免重复探索
  - 提升输出性能、加速训练过程、更好结合人类经验

- 应用：

  - 搜索与救援
  - 故障检测与监测
  - 定位与制图

  

- 创新点：

  - 动态Voronoi分区避免机器人探索重复区域
  - DDPG算法与Prioritised Experience Replay（PER）优先经验回放池相结合。
  - 基于深度强化学习的方法可以使机器人到达目标位置，与此同时从人工演示数据中学习，加快训练速度提高最终性能。
  - 应用于实际环境

- 缺点
  - 使用单智能体的策略
  - 只是分区探索，没有体现出多智能体的协作能力

### 2.Voronoi Partitions

- **沃罗诺伊图**（Voronoi Diagram // 又叫泰森多边形或Dirichlet图）：对离散数据点合理地连成三角网（对空间进行划分），然后对这个几何体进行探究。

- 划分依据：按距离划分邻近区域

- 划分算法：分治法、扫描线算法和Delaunay三角剖分算法

- 基本术语

  - **基点site**：具有一些几何意义的点，在这里的话即为 移动的机器人
  - **细胞Cell**：这个Cell中的任何一个点到Cell中基点的距离都是最近的，离其他Site比离内部Site的距离都要远。 每个Cell都是凸（凸多边形）的，图中一定会有一些Cell是无界的。每个机器人在自己的区域内进行探索。

- 建立Voronoi图：

  ![](img/Paper6_1.png)

  - 离散点自动构建三角网，即构建Delaunay三角网。对离散点和形成的三角形编号，记录每个三角形是由哪三个离散点构成的。
  - 计算每个三角形的外接圆圆心，并记录之。
  - 遍历三角形链表，寻找与当前三角形pTri三边共边的相邻三角形TriA，TriB和TriC。
  - 如果找到，则把寻找到的三角形的外心与pTri的外心连接，存入维诺边链表中。如果找不到，则求出最外边的中垂线射线存入维诺边链表中。
  - 遍历结束，所有维诺边被找到，根据边画出维诺图。
    

- 特点：
  - 因为是在迭代的基础上执行的，因此 Voronoi partitions 是动态的。Voronoi分区可以通过机器人间通信以分散的方式实现。
  
  - 区域内的所有空间，会被可移动机器人覆盖。
  
  - cell内的点到该cell基点（site）的距离小于到其他基点的距离
  
    <img src="https://pic4.zhimg.com/80/v2-1bf26ccb28f59dde0157f6451418d63b_720w.jpg" alt="img" style="zoom: 50%;" />
  
  - Vertex Edge上的点x处于几个Cell的公共交上，那么x到这些Cell对应的Site的距离都是相等的
  
    <img src="https://pic2.zhimg.com/80/v2-d5c70c0492da24a6c0b4d8ef9cd8cc21_720w.jpg" alt="img" style="zoom: 50%;" />
- 应用
  - 在多智能体自主探索时，可能会出现多个智能体探索同一个区域的现象出现，这样会效率低下，浪费时间。

- ref

  https://zhuanlan.zhihu.com/p/33896575

### 3. Cooperative Exploration Strategy

#### A. Hierarchical Control Architecture

![](img/Paper6_2.png)

- 在第一层，根据Voronoi分区和同步地图选择所需的下一个边界点；
- 第二层，记录第一层得出的下一个位置信息，然后使用DRL神经网络避开潜在障碍物的同时到达所需位置。
- 假设信息：
  - 全向传感系统；可以监测空闲区域，以及障碍物的边界信息；监测范围：$r_s$
  - 广播信息；每个机器人广播自己的相对位置信息，同时可以接受自己邻居节点信息；交流范围 $r_c$;   $r_c >r_s$
- **具体流程：**
  - 首先，每个机器人会通过传感器获取自己的局部的地图信息，每个机器人获取的信息会更新总体的地图
  - 每个机器人会根据自己传感器获取的局部地图的信息得到边界点，同时通过Voronoi分区会划分每个机器人可行的区域，机器人会通过信息点的效用函数选择在自己分区内的边界点，作为接下来运动的点。
  - 将获取的这个目标点的信息，以及自身传感器获得的数据和机器人当前的位置，输入到DRL中，输出相应的线速度和角速度，在向目标点前进的同时，若突然出现障碍物，也会及时发现，并躲避障碍物。

#### B. Coordination Algorithm for Explorer Robots

<img src="/Users/mapeixin/Documents/Typora/未知环境/img/Paper_3.png" alt="image-20210908192119636" style="zoom:50%;" />

- **信息点**；在感测区域和未被传感器覆盖的开放区域之间的边界上生成一些新的边界点，所选的接下来要到达的点，即为接下来的信息点。当设定一个信息点时，即可在传感器范围边界产生新的信息点。当找不到边界点时说明整个环境得到了充分的开发。信息点可以是真实传感器设备或虚拟目标点

- **信息图**：

  ![image-20210926114415257](/Users/mapeixin/Documents/Typora/未知环境/img/Paper6_15.png)

  信息图共有三部分组成：

  - $\mathcal{V}$：由每次记录的信息点组成
  - $\mathcal{E}$：由信息点组成的边集
  - $\mathcal{A}$：相邻信息点权重(即两点之间的距离)集合

  任意两点之间的最短路径：连接他们两点之间所有边的权重之和——————？？？

- **Voronoi Partitions**：

  ![image-20210926153709627](img/paper6_14.png)

  > 在分区内所有点到基点的距离都要小于到其他基点的距离
  >
  > $\mathcal{N}_{R_i}$：代表$R_i$邻居节点的集合

  使用动态Voronoi图进行动态划分时，每个智能体计算它的区域只需要知道邻居节点的坐标就可以

- **信息点的效用函数**：
  $$
  \Omega_{i k}=\lambda d_{i k}+(1-\lambda) \phi_{i k}
  $$

  > 同时考虑路径成本和目标距离的效用函数，用于确定所需的下一个边界点，可以根据不同的场景选择深度优先和广度优先模式。
  >
  > - $d_{ik}$：机器人和边界点的信息
  > - $\phi_{i k}$：边界点和起始点的位置 —————— 距离公式？？？？
  > - 当$\lambda=0$时，主要进行广度优先搜索，在下一个深度层次的边界点之前，即尽可能在当前深度搜索其邻居边界节点
  > - 当$\lambda=1$,主要进行深度优先搜索，即在回溯前，尽可能探索更深的边界节点

- **基本运行流程：**

  - 对于单个智能体而言：当一个智能体放置一个信息的信息点（information node），信息图（Information graph）I 就会更新，每个智能体也会广播更新的信息图。每个智能体会以最短路径到达信息点。当智能体到达边界点时，会重新放置新的信息点。

  - **随着机器人的运动，整个环境的Voronoi分区也会随时间变化。**

  - 对于多个智能体而言：通过Voronoi划分的方法，每个智能体只能去到属于该区域边界点的位置。

    <img src="/Users/mapeixin/Documents/Typora/未知环境/img/Paper6_3.png" alt="image-20210909105700377" style="zoom:50%;" />

- 基于Voronoi协作探索策略

  <img src="/Users/mapeixin/Documents/Typora/未知环境/img/Paper6_4.png" alt="image-20210909105941495" style="zoom:50%;" />



#### C. Deep Reinforcement Learning Setup

- **避碰算法**：根据上一步机器人向航路点（waypoint）移动过程中没有考虑障碍物的信息，因此需要一种避碰算法来产生反应性机动。

- **观测空间：**
  - 激光测距仪状态($l_t$)：	24维 
  - 先前速度状态($v_{t-1}$)：   2维
  - 相对目标位置状态($p_t$)： 2维

- **动作空间**(连续)

  - 线速度($v_l$)：
  - 角速度($v_a$)：

- **奖励空间**：
  $$
  r=r_{d}+r_{c l}+r_{a v}+r_{l v}
  $$

  - $r_{d}= \begin{cases}r_{a} & \text { if } d_{p}<d_{p \min } \\ \Delta_{d_{p}} & \text { otherwise }\end{cases}$

    > 距离奖励 —— 鼓励前往目标点 
    >
    > - 到达目标点的奖励
    > - 最后一个时间步向目标移动的距离；即如果t时间相较于t-1时间步距离目标点距离更近会获得一个正奖励，否则获得一个负奖励

  - $r_{c l}= \begin{cases}r_{c p} & \text { if } d_{o \max } \leq d_{p}<2 d_{o \max } \\ r_{c p o} & \text { if } d_{p}<d_{o \max } \\ 0 & \text { otherwise }\end{cases}$

    > 安全界限奖励 —— 靠近障碍物会给予惩罚；与障碍物保持安全距离；
    >
    > - 当agent障碍物距离大于 $d_{o \max}$，小于 2$d_{o \max}$就会给予一定较少的负奖励
    > - 当agent与障碍物的距离小于$d_{o \max}$，会给予较大的负奖励
    > - 与障碍物保持安全距离不会受到惩罚

  - $r_{a v}= \begin{cases}r_{a p} & \text { if }\left|v_{a}\right|>v_{a \max } \\ 0 & \text { otherwise }\end{cases}$
    $r_{l v}= \begin{cases}r_{l p} & \text { if } v_{l}<v_{l \min } \\ 0 & \text { otherwise }\end{cases}$

    > $r_{av}$：限定最大角速度 —— 因为采用
    >
    > - 当智能体大于设定的角速度值时会惩罚
    >
    > $r_{lv}$：限定最小线速度
    >
    > - 当智能体的线速度小于设定的线速度会惩罚

- **神经网络结构**

  - actor网络

  <img src="/Users/mapeixin/Documents/Typora/未知环境/img/Paper6_5.png" alt="image-20210910095140331" style="zoom:67%;" />

  - critic网络

    #### <img src="/Users/mapeixin/Documents/Typora/未知环境/img/Paper6_6.png" alt="image-20210910100550854" style="zoom: 67%;" />

- **Integration of DDPG and PER.**

  <img src="/Users/mapeixin/Documents/Typora/未知环境/img/Paper6_7.png" alt="image-20210910103927077" style="zoom:67%;" />

  ​	通过观察agent行为以及 Gazebo的环境，我们可以控制agent躲避障碍物的同时向目标点移动。通过将DDPG算法和PER（Prioritised Experience Replay）**优先经验回放池**相结合，通过计算每个状态的优先值来优先采样重要的数据。与此同时，通过学习人类范例数据（human demonstration data）去提高DDPG算法性能。

  > 虽然是多智能体进行探索，不过这里使用的还是单智能体的策略，所有的智能体共用一个网络进行到目标点的非碰撞移动。
  
  <img src="/Users/mapeixin/Documents/Typora/未知环境/img/Paper6_8.png" alt="image-20210910105201210" style="zoom:50%;" />
  
  > $p_{i}=\delta_{i}^{2}+\lambda\left|\nabla_{a} Q\left(s_{i}, a_{i} \mid \theta^{Q}\right)\right|^{2}+\tau_{p}+\tau_{D}$
  >
  > ​	$p_i$：状态转移优先值
  >
  > ​	$\delta$： TD Error
  >
  > ​	$\lambda\left|\nabla_{a} Q\left(s_{i}, a_{i} \mid \theta^{Q}\right)\right|^{2}$： actor loss
  >
  > ​	$\tau_{p}$：每个状态转移的小正抽样概率，确保可以对每个状态转移进行采样
  >
  > ​	$\tau_{D}$：用于增加演示数据的采样频率
  >
  > $P(i)$：第 i 次状态转移到采样概率
  >
  > $w_i$：每个状态转移的采样权重
  >
  > $B$：代表Batch-Size大小
  >
  > $\beta$：调整采样权重的常数值



### 4. Results and analysis

#### a.碰撞策略的训练和评估

![image-20210911105307750](/Users/mapeixin/Documents/Typora/未知环境/img/Paper6_9.png)

> 左侧为训练地图，右侧为测试评估地图，评估地图增加了难度

- 目标：在如上图所示的环境下，躲避障碍物，找到红色目标点，每次找到红色目标点会随机更换位置。

- 参数设置：

  - 机器人与目标点的最近距离(即机器人到达目标点)： $d_{p \min } = 0.2m$ 
  - 机器人与障碍物的最大距离$d_{o \max} =0.25m$
  - 机器人最大角速度：$d_{a \max}= 0.8 rad/s$
  - 机器人最小线速度：$d_{l \min} = 0.052 m/s$

- 训练结果：

  ![image-20210911113511970](/Users/mapeixin/Documents/Typora/未知环境/img/Paper6_10.png)

- 测试结果

  在图7（b）所示的环境中进行测试，所提出的训练方法只训练10000次，DDPG训练50000次，对于20个随机任务，所提出的方法能够全部完成， 且有三次能找到长墙后的目标点，而DDPG算法却只能完成15个随机任务，且在这15个随机任务中存在不必要的转弯。因此提出的方法又快又好。



#### b. 合作探索策略评估

 ![image-20210912135900856](/Users/mapeixin/Documents/Typora/未知环境/img/Paper6_12.png)

> 虚线表示基于每个机器人当前位置生成的Voronoi分区。机器人在顶部开始运动，在此处防止第一个信息点在运动过程，每个机器人在传感区域边缘生成多个边界点。图中实体原点代表每个智能体历史放置的信息点。
>
> 传感器监测范围 $r_s = 1.3m$ ； 智能体交流范围 $r_c = 5m$
>
> 协调参数$\lambda =0.8$ ，不同的参数值会影响总探索的时间，不会影响任务是否完成。

图a，两个智能体用时873s；图b使用了三个智能体，用时598s；图c使用了四个智能体，用时539s；



###  c. 算法对比

- 将该算法与 “Cooperative exploration and networking while preserving collision avoidance,”算法进行比较，原始算法只能够解决相同的边界点不会被不同机器人访问，无法很好的解决机器人协作探索冲突的问题。

- 当机器人数量达到五个后， 在增加机器人的数量，减少探索时间不会很明显。



### d. 真实环境验证

- 实验环境设备：

  - 三个 TurtleBot3 Waffle Pi 机器人装备激光雷达（监测范围为1m）进行实验
  - 基于ROS环境进行机器人之间交流
  - 每个机器人装载了Raspberry Pi 3用于向主机发送传感器信息，并从主机接受控制命令

  ![image-20210912150858484](/Users/mapeixin/Documents/Typora/未知环境/img/Paper6_13.png)

- 测试方法：

  每个机器人被放置在房间的中间位置，并在他们的初始位置放置了信息点。并且为了测试多智能体对动态环境的鲁棒性，在第二个机器人要到达下一个边界点的途中突然放置了一个障碍物，基于DRL的方法，机器人可以快速调整路线，避免发生碰撞。基于Voronoi的方法，可以很好的降低任务完成的时间，避免冲突。

  ![image-20210912152323265](/Users/mapeixin/Library/Application Support/typora-user-images/image-20210912152323265.png)







- finite-time consensus algorithm 优先时间一致性算法
