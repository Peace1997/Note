
# 二、 PID控制器

## 1. 基础制导律学习
在惯性制导系统中，常用的制导律包括比例导引（Proportional Guidance）、比例-微分导引（Proportional-Derivative Guidance）和比例-积分-微分导引（Proportional-Integral-Derivative Guidance）。这些导引律是用于计算制导命令的数学算法，以实现导弹或飞行器的精确制导和控制。

1. **比例导引**（Proportional Guidance）：
比例导引是最简单的导引律之一。它根据目标与导弹之间的位置差异来生成制导命令。制导命令的大小与目标与导弹之间的距离成正比。比例导引的数学表达式如下：
$$C = K_p \cdot \delta$$
其中，$C$ 是制导命令，$K_p$是比例增益，$\delta$ 是目标与导弹之间的位置差异（通常是位置矢量的差）。

2. **比例-微分导引**（Proportional-Derivative Guidance）：
比例-微分导引在比例导引的基础上增加了速度差异的考虑。除了目标与导弹之间的位置差异，它还考虑了它们之间的速度差异。这可以提供更快的响应和更好的稳定性。比例-微分导引的数学表达式如下：
$$C = K_p \cdot \delta + K_d \cdot \frac{d\delta}{dt}$$
其中，$C$ 是制导命令，$K_p$ 是比例增益，$K_d$ 是微分增益，$\delta$ 是目标与导弹之间的位置差异，$\frac{d\delta}{dt}$ 是位置差异的导数（速度差异）。

3. **比例-积分-微分导引**（Proportional-Integral-Derivative Guidance）：
比例-积分-微分导引是进一步扩展了比例-微分导引的导引律。除了位置差异和速度差异，它还考虑了时间上的积分效应，用于消除系统的静态误差。比例-积分-微分导引的数学表达式如下：
$$C = K_p \cdot \delta + K_i \cdot \int \delta \, dt + K_d \cdot \frac{d\delta}{dt}$$
其中，$C$ 是制导命令，$K_p$ 是比例增益，$K_i$ 是积分增益，$K_d$ 是微分增益，$\delta$ 是目标与导弹之间的位置差异，$\int \delta \, dt$ 是位置差异的时间积分，$\frac{d\delta}{dt}$ 是位置差异的导数（速度差异）。


对于比例控制，$K_p$ 越大反应越大，越容易到达目标值附近，存在稳态/静态误差。
为了消除稳态误差，就要引入积分控制，通过对历史误差求和
为了解决系统查了



