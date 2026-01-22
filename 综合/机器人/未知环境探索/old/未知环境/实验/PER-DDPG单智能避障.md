# 论文6 —— 复现



![multiple-turtlebot-collision-avoidance](/img/multiple-turtlebot-collision-avoidance.png)



### 一、人工经验数据

人类玩家操纵机器人进行移动，与此同时会记录移动过程中的 state、action、reward、next_state、done信息，将其保存在mat文件中。在训练开始前会将这些数据，提取出来，输入到优先经验回放池中。

<img src="/image-20211007152157041.png" alt="image-20211007152157041" style="zoom:50%;" />
![]("/image-20211007152157041.png)
论文参考：**Accelerated Sim-to-Real Deep Reinforcement Learning: Learning Collision Avoidance from Human Player** 





### 1. 单智能体障碍物躲避

#### 1.1 只使用DDPG算法：

- 未对无穷数据处理

  <video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/run1_1_1.mov"></video>

  一个回合共500步，在一个回合内发生碰撞，agent和目标也会重新初始化，不过会继续累积步数，直至500步后会进入下一个回合。

  此次我训练了300个回合共计15w步，作者共训练了8w步，效果同样不是很好。

- 对无穷大数据处理后

  <video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/run1_1_2.mov"></video>

  > 有时会抵达目标点，不过碰到障碍物的情况也是比较多的

  对于测试环境:

  <video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/run1_1_3.mov"></video>

  > 无法很好有效的到达目标点

### 1.2 使用PER +DDPG

<video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/run1_2_1.mov"></video>

效果相较于单纯DDPG算法要好一些，偶尔会发生碰撞

测试环境同上，无法很好的找到目标点



### 1.3 使用PER +人工经验数据 +DDPG

<video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/run1_3_1.mov"></video>

也会发生碰撞，但是碰撞的次数更少一些，收敛速度更快，获得的return也要更高。

对于测试环境：可以较好的找到目标点，会有较少的碰撞。

<video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/run1_3_2.mov"></video>





#### 1.4. 使用PER +人工经验数据 +DDPG. 修改传感器检测范围

<video src="/Users/mapeixin/Documents/Typora/未知环境/实验/img/run1-4.mov"></video>

当修改传感器的检测范围为 -90到90时，当目标点在障碍物后面的时候，避障能力较差，经常会发生碰撞。



#### 1-5 ： 雷达检测范围[-80,80]

#### 1-6： 雷达检测范围[-90,90]

#### 1-7： 雷达检测范围[-90,90] 增加训练次数

