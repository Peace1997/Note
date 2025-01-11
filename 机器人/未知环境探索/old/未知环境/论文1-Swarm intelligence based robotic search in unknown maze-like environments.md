### Swarm intelligence based robotic search in unknown maze-like environments 

## 未知迷宫环境下基于群体智能的机器人搜索

- 简述
  
  - 研究重点：未知复杂迷宫问题搜索、避障
  
  - 实验效果：能够在有效时间内，在迷宫中避障并找到目标，即使迷宫环境变得复杂，也能保持一定的性能
  
  - 实际应用：用于搜救搜索
  
  - 关键因素：机器人的运动的目的和环境条件
  
  - 适应度函数：
    $$
    f(x)=\frac{1}{\left\|x_{\text {target }}-x\right\|}
    $$
  
  > x：表示机器人的位置
  >
  > $x_{target}$：目标位置
  >
  > 越大越好
  
- 相关任务：
  
  - 本文解决的问题：
    - 在未知的复杂环境中，寻找隐藏的目标，同时克服障碍物的影响，以及跳出局部最优区域
  - 相关问题研究：
    - 机器人的路径规划问题
    - 未知环境中寻找隐藏目标的问题
    - 在复杂的搜索空间中，避障并搜索到目标
  
- 关键的技术
  - 基于粒子群算法的思想
  - 工具箱包括两个算法：（==创新点==）
    - 用于避障的：**Angle of rotation(AoR)** —— 旋转角度来避障，同时通过一定的参数来防止角度变化过快
    
      > 当碰撞时，粒子变热， T会变到最大，Θ 会相较原状态变化较大，$T_{RR}$会逐渐接近于1，快速旋转；结束碰撞后，粒子会变冷，T和 Θ 都会变到最小，$T_{RR}$逐渐趋近于0，也就意味着粒子不会旋转，逐渐恢复到原来移动方向。
      >
      > T `和$T_{RR}$就是那个控制结束碰撞后，使粒子升温，尽量减缓 Θ 降低的方式
    
    - 跳出局部最优解的：**Memory （Mem)** —— 改变原有的位置，使单个粒子跳出局部最优，
    
      > 通过重新定义个人最佳(Pbest)和全局最佳(Gbest)来改变机器人原有的运动方向。
  
- 过程：

![]()

  **初始化阶段：**

  - 随机初始化各个粒子i速度V和位置s
  - 设置T、角度 Θ 、DC为0；
  - 初始化各粒子i的内存Memory

  **粒子寻优阶段：**

  - 通过公式7-9来更新当前时刻(t)的T、Θ，然后通过公式10来更新当前位置
    $$
    \begin{aligned}
    &R V_{t}^{i}=\left[\begin{array}{cc}
    \cos \left(\Theta_{t}^{i}\right) & -\sin \left(\Theta_{t}^{i}\right) \\
    \sin \left(\Theta_{t}^{i}\right) & \cos \left(\Theta_{t}^{i}\right)
    \end{array}\right] \cdot V_{t}^{i} \\
    &\Theta_{t+1}^{i}=T_{t}^{i} \cdot \Theta_{t}^{i} \\
    &T_{t+1}^{i}=T_{R R} \cdot T_{t}^{i} \\
    &x_{t}^{i}=x_{t-1}^{i}+R V_{t}^{i}
    \end{aligned}
    $$

  - 如果更新后的位置和未更新的位置之间的距离小于一定的阈值(MD)则：
    - 根据MPP_2准则(重复选择一个位置会受到惩罚)，降低内存中((τ,ν)对)靠近$P_{t-1}$位置(τ)附近对应的v值(适应度值)

  - 根据算法4的方案来更新当前状态P_t和Gbest（见论文）

  -  根据公式10-11来更新η1和 η2
    $$
    \begin{aligned}
    &x_{t}^{i}=x_{t-1}^{i}+R V_{t}^{i} \\
    &\eta_{1}^{i} t=\left\{\begin{array}{ll}
    1 & \text { fitness }\left(P_{t}^{i}\right)>F F_{T H R} \\
    0 & \text { else }
    \end{array}\right.
    \end{aligned}
    $$

  - 如果η1 和 η2 同时为0，则：
    -  粒子重置

  - 否则:
    - 使用公式13来更新位置信息
      $$
      \begin{aligned}
      &V_{t}^{i}=V_{t-1}^{i}+\eta_{1}^{i} t \cdot r_{1} \cdot c_{1} \cdot\left(P_{t-1}^{i}-x_{t-1}^{i}\right)+\eta_{2}^{i} t \cdot r_{2} \cdot c_{2} \cdot\left(G_{t-1}-x_{t-1}^{i}\right)\\
      \end{aligned}
      $$

    - 如果期间发生了任何碰撞，则：

      - 通过公式4，修改$s_t$来调整下一时刻(t+1)的角度Θ
        $$
        \begin{aligned}
        &\Theta_{t+1}^{i} \leftarrow \Theta_{t}^{i}+s_{t}^{i}\\
        \end{aligned}
        $$

      - 同时把 $T_{t+1}$ 设置为最大

      - 通过公式5来增大 Dc的值
        $$
        \begin{aligned}
        &D c_{t}^{i}=D c_{t}^{i}+k\\
        \end{aligned}
        $$

      - 如果$Dc_t$的值大于rand()：
        - 则将$s_t$反向(*(- 1))，将$Dc^i_t$置为0

      - 否则：

        - 利用公式6逐渐降低Dc_t的值

        $$
        \begin{aligned}
        &D c_{t}^{i}=p \cdot D c_{t}^{i}
        \end{aligned}
        $$

  - 不停迭代循环直至满足迭代终止条件



- 仿真验证
  - 使用unity 3d 进行简单到复杂的5个环境搭建
  - 相关参数灵敏度检测
  - 在一个简单环境中，同目前几种常用算法（A-RPSO、RBA、ARBA）进行比较
  - 在5个环境中，设计了三个实验，进行验证，比较使用AoR和Mem带来的性能的提升

