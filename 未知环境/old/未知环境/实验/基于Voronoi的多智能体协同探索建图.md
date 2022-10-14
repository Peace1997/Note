## 简介：

该实验为未知环境下基于Voronoi和深度强化学习的多移动机器人自主探索，为了更加有效率的完成多移动机器人协作探索任务，使用了一种分层控制结构，该结构包括一个高层决策层和低层目标跟踪层。其中高层决策层通过Voronoi分区和选点公式为每个移动机器人分配不同的目标位置来最小化重复的探索；低层目标跟踪层使用基于深度强化学习的方法来使每个移动机器人无碰撞的抵达相应的目标位置。

其中实验环境基于ROS（Robot Operating System）平台，移动机器人使用的是ROS开源智能小车Turtlebot3，每辆小车装载激光雷达传感器，其中移动机器人的传感器检测范围为0.1m-3.5m，检测范围为-90度-90度，共24条激光雷达距离信息。

### 1.单智能体避障

- PER + DDPG + 人工经验数据

### 2. 多智能体选点

- 可选目标点计算：
  $$
  x: robot\_position\_x + range\_max*\sin\theta \\
  y: robot\_position\_y - range\_max*\cos\theta \\
  \theta:index*angle\_increment  + robot\_position\_z + (\pi/2-angle\_max)
  $$

  > $range\_max$: 3.5m
  >
  > $robot\_position\_z$： 表示当前机器人偏转的角度
  >
  > $angle\_increment$:   0.13 / 7.5度
  >
  > $angle\_min$:开始角度值；-1.57
  >
  > $angle\_max$:结束角度值；1.57

- Voronoi选点

  可利用Voronoi分区选点的一条性质：在某个基点所属的cell中，该cell内所有的点到这个基点的距离都要小于到其他基点的距离。因此在所有待选的边界点中选择到该基点的距离小于到其他基点的 边界点组成新的待选边界点集合，然后在通过上面位置选择计算公式选择使得$\Omega$最小的点作为接下来的目标点。

- 判断是否到达目标点：

  当与目标点的距离小于 1m时，即可认为到达目标点。

- 位置选择计算公式1
  $$
  \Omega_{i k}=\lambda d_{i k}+(1-\lambda) \phi_{i k}
  $$
  其中 $d_{i k}$  （机器人当前位置与可选目标点的距离）和 $\phi_{i k}$ （机器人初始位置与可选目标点的距离）在这里都表示直线距离。此时$\lambda =0.8$

- 位置选择计算公式2
  $$
  \Omega_{i k}=\alpha d_{i k}+\beta \phi_{i k} - \gamma l_{i k} \\
  \alpha + \beta + \gamma =1
  $$
  其中 $d_{i k}$  （机器人当前位置/上一次目标点的位置  与下一个可选目标点的距离）和 $\phi_{i k}$ （机器人初始位置与可选目标点的距离）在这里都表示直线距离。其中$l_{ik}$表示在交流范围内，与其他智能体之间的距离。$\alpha、 \beta 、 \gamma$  分别取值为 0.2、0.6、0.2

  > 因为在判断机器人是否到达目标点时，小于一定距离即可认为到达目标点，因此将上一次目标点的位置作为$d_{ik}$的距离，就可以使得$d_{ik}$的值为可变值。
  >
  > 

- 初始选点：

  如果最开始采用基于Voronoi随机选点的方式，会使得移动的方式更加丰富，如果一开始就按照给予的位置公式选点，每个智能体移动的路线基本确定，会减少一定的随机性。



### 3. 评价标准：

$$
\text { Explored region rate }=\frac{\text { Number of explored free cells }}{\text { Number of free cells in the real map }}
$$

> 地图大小为：160x160(162)。  25600/25920
>
> 当实时获取地图信息时，-1 代表未探索区域，0表示free区域，100表示障碍物；
>
> 当地图保存为pgm格式时，205表示未探索区域，254表示free区域，0表示障碍物。
>
> 对于公式分母，通过包含完整地图信息的pgm图片来获取，其free区域数为6251。
>
> 公式分子，即为每次采用不同方式建图，图片的free区域数量。

![image-20211112150437986](/Users/mapeixin/Documents/Typora/未知环境/实验/img/true_map.png)

![test_map](/Users/mapeixin/Documents/Typora/未知环境/实验/img/test_map.png)

三个智能体，使用Vornoi分区 + 位置选择公式。 五次平均用时: 55s

四个智能体，仅使用位置选择公式。 五次平均用时: 63s

```
问题:经常会发生智能体碰撞
```

单个智能体。仅使用位置选择公式

如果运行算法，经常会在局部最优解处停住。基本无法达到80%的效果，测试中最好达到过55%。 当在局部最优解停住或碰撞到障碍物时，会返回原点，保留之前的建图效果， 此时平均用时大约在 113s。



### 3.运行效果

#### 初始运行效果

- 地图1-  没有障碍物. map1_no_obs.mp4

- 地图1 - 突然出现障碍物  map1_add_obs.mp4

- 地图1 - 未加入Voronoi.  map1_no_voronoi 

- 地图1 - 加入Voronoi.    map1_voronoi.mp4

- 地图2 - 未加入Voronoi.   map2_no_voronoi

- 地图2 - 加入Voronoi.    map2_voronoi.mp4+



1.实际完成情况：当3个自主体进行协同探索时，其效率是单个智能体的两倍，且高于4个自主体独自探索效率。

2.存在的主要问题及意见：当使用基于Voronoi分区方法选点时，对于机器人所在位置较为紧密的情况，可能会使智能体提前终止，陷入局部最优，因此在选择接下来要到达的目标点时，可以记录之前移动机器人边界点，共同作为接下来可选的目标点；在进行路径规划时在未训练环境下可能会发生碰撞的情况，因此在训练时可以增加训练地图的难度，或使用其他强化学习的方法进行解决。

3.后续发展及有关成果转化的思路：接下来可以通过多智能体深度强化学习方法替代传统选点方式，通过好奇心机制、课程学习等方法作为辅助任务，加速训练，简化学习过程，从而更好的指导移动机器人更有效率的完成自主探索任务。


### 11.7算法更新后运行效果

-  新地图1两次测试

  <video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/test_env_1_1.mp4"></video>

  <video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/test_env_1_2.mp4"></video>

- 新地图2两次测试

  <video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/test_env_2_1.mp4"></video>

  <video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/test_env_2_2.mp4"></video>

- 地图2突然增加障碍物避障能力测试

  <video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/test_env_2_3.mp4"></video>

- 地图三测试

  <video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/test_env_3_1.mp4"></video>

-  新地图1仿真 + 建图

  <video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/仿真&amp;建图.mov"></video>
  
  通过修改gmapping文件参数（minimumScore、particles），调节建图效果
  
  <video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/仿真&amp;建图2.mp4"></video>

![simulation_gmapping](/Users/mapeixin/GitProject/Voronoi_Multi_Robot_Collaborate_Exploration/imgs/simulation_gmapping.gif)



### 3. 问题存在

- 使用Voronoi分区，存在特殊情况，会使机器人的运动提前终止

  - 当机器人所在位置较为紧密时，此时还没完成探索，中间机器人可能就会提前停止，认为自身探索任务会结束。因为此时中间智能体所有可选的边界点都在其他两个智能体的分区内，接下来没有可选的边界点，会认为达到终止条件，从而结束探索。
  - 同样，如果初始时有存在类似的较为紧密的站位情况，可能从开始就无法进行下去。

  <img src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/question.png" alt="image-20211022145604859" style="zoom:50%;" />

- 单智能体避障，在到达某一目标点时的训练测试效果比较好，但是融合于多智能体自主探索时，效果较差，经常会发生碰撞。

- 单智能体避障存在碰撞情况，卡在某个角落
- 陷入局部最优解



- **选点：** 基于Voronoi的选点公式是启发式的算法，固定策略，效率低。
- **到达选定目标点：** 基于人工先验数据的PER-DDDG算法单智能避障鲁棒性不强，在一个未训练的陌生环境中可能会发生碰撞的情况。
  - 原因分析：
    - 用于训练的地图较少，仅用一、两张地图完成训练，且待训练地图较为简单，因此在新地图环境下表现不好。
    - 人工先验数据需要人类在某个地图自主控制机器人完成探索并记录相关数据，对这张地图适配较好，在新地图上直接采用这些数据效果不好。
    - DDPG算法本身的缺陷

![image-20220303111616064](/Users/mapeixin/GitProject/Voronoi_Multi_Robot_Collaborate_Exploration/imgs/origin&true.png)



涉及package

```
turtlebot3:
git clone https://github.com/ROBOTIS-GIT/turbot3.git
cd ~/catkin_ws && catkin_make

多机地图合并软件包:
sudo apt install ros-melodic-multirobot-map-merge ros-melodic-explore-lite

```

