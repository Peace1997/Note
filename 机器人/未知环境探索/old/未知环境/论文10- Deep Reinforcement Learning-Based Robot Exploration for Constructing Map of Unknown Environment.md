# Deep Reinforcement Learning-Based Robot Exploration for Constructing Map of Unknown Environment

### 未知环境下基于强化学习的探索和建图

INFORMATION SYSTEMS FRONTIERS  6.191

 

## 1. 简述

- **整体流程：** 本文基于双重结构模型（dual-model architecture）理论，提出了基于DQN的探索模型和基于DQN的导航模型，通过一定的转换机制，解决快速发现未探索区域和跳出局部最优解的问题。
- **主要问题解决**：
  - 跳出局部最优解（模型转换）
  - 尽可能避免重复探索（区域计数）
- **创新点**：
  - 双模型结构
- **缺点：**
  - 消耗资源
  - 探索时间长



## 2. 总体设计

共有两个模型，BE控制器用于实现主要的探索任务，当陷入局部最优时会转换为BN控制器跳出局部最优解（辅助任务）

### 2.1  Switch Model Mechanism

**初始时**： 

通过BE控制器尽可能的前往未探索区域，在这个过程中未与避免障碍物发生碰撞。

**BE控制器转BN控制器**：

当机器人陷入局部最优解时，将转换为BN模型。BE转换BN时需要两个条件：

- 周边的方块都是已经探索过的区域，并且机器人在每个区域的步数都达到了最大限制。
- 探索过程中因碰撞障碍物而卡住。

**BN控制器转BE控制器**：

当转换为BN控制器时，机器人需要在已经行走的边界区域中选择步数最少的区域，通过 BN 控制机器人移动到该点。到达该点后重新转化为BE控制器。

<img src="img/paper10_1.png" alt="image-20220513212550920" style="zoom:50%;" />



### 2.2 Basic explore controller

**目标**：更有效率的引导智能体前往未探索区域

**模型参数**

- $\alpha$ :Reward Function Balance Adjustment Parameters : 平衡每个小奖励对总奖励的影响。
- $\beta_i$:Stage Task（？）：将地图进行划分为几个阶段，即任务分阶段完成，并设计了不同阶段之间的关系，得分越高，越稳定。

- $MGR_n$:Map Reward Value：建图差值
  $$
  S=\operatorname{size}\left(M G R_{n}\right)-\operatorname{size}\left(M G R_{n-1}\right)
  $$

- $flag_{(i,j)}$:Calculation of Steps in Individual Areas：记录机器人在每个方格上走过的次数，目的是驱使机器人每轮都更倾向于未探测区域。

  - 机器人的预设初始位置为（0,0,0）
  - 每个网格的中心位置代表该网格的代表坐标点
  - 每个回合，判断机器人是否通过网格代表点（通过机器人轨迹判定），并进行记录。

### 2.3  Basic navigation controller (BN)

BE中通过区域计数的方法虽然可以避免重复探索，但训练过程较难收敛，当智能体所在的区域勘探次数达到限定值时（这里为7），此时称该区域为困境区域（dilemma area），切换为BN模型，机器人设置好起始点和终止点后（？），避免碰撞引导机器人前往该点。

![image-20220513223500990](Paper10_3.png)

### 2.4 DQN模型设计

**State：**
$$
\text { State }=\operatorname{LDS}([\mathrm{n}])+\text { Map's growth rate }(\mathrm{n})+\text { Flag }(\mathrm{n})
$$

> 激光雷达传感器的距离数据信息 +  建图差值 + 当前状态下在该方格区域走过的次数

**Action：**

直行 + 线速度  +  角速度 （3dim）

**Reward：**
$
R_{f}=-p o w\left(2, f l a g_{(i, j)}\right) \\
R_{g}=M G R_{t} \cdot \alpha \cdot \beta \\
\mathrm{R}=R_{f}+R_{g}
$

> $R_f$:机器人前往某个方格区域的次数越多，惩罚越大
>
> $R_g$：建图越快，奖励越大。

**网络结构**

<img src="img/Paper10_2.png" alt="image-20220513222958419" style="zoom:50%;" />

## 三、 实验验证

### 3.1 探索模型实验

**选择最合适的 $\alpha$ :（Reward Function Balance Adjustment Parameters : ）**

$\alpha$ 设置的范围为5-10，观察奖励函数的变化情况。

<img src="../../../Library/Application Support/typora-user-images/image-20220514111950628.png" alt="image-20220514111950628" style="zoom:50%;" />



### 3.2 性能对比实验

**与Slam-Gmapping算法相对比**

设计了四种环境，与在相同的步数下，对地图的探索区域进行对比。Slam-gmapping是基于已知地图的方法，本文提出的方法是基于没有没有地图的方法，在相同的步数下：

- 本文提出的方法能够更快跳出局部最优解
- Slam-Gmapping的方法地图探索更高（相差5%）

<img src="../../../Library/Application Support/typora-user-images/image-20220514112452790.png" alt="image-20220514112452790" style="zoom: 50%;" />