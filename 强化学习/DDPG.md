# DDPG 
Deep Deterministic Policy Gradient

# 一、背景

**深度确定性策略梯度算法(Deep Deterministic Policy Gradient)** 是一种同时使用Q-function和Policy的算法。它使用off-policy(异策)数据和Bellman方程去学习Q-function，然后使用Q-function 来指导Pollicy

DDPG与Q-learning紧密相关：如果知道最优动作价值函数 $Q^*(s,a)$，在任何state下，都可以通过下列公式求解得到最优的动作$a^*(s)$:
$$
a^{*}(s)=\arg \max _{a} Q^{*}(s, a)
$$
DDPG采用了近似(approximator)$Q^*(s,a)$和近似$a^*(s)$相结合的方法，这种方法很适合具有连续动作空间的环境。

- DDPG是一种off-policy算法
- DDPG仅可用于连续动作空间的环境
- DDPG可以看作是基于连续动作空间的深度Q学习(DQN)


DDPG主要包括两部分，一个是Q-function，另一个就是policy，接下来将详细介绍这两个部分

# 二、DDPG中的Q-Learning

通过Bellman方程描述最优动作-价值函数：
$$
Q^{*}(s, a)=\underset{s^{\prime} \sim P}{\mathrm{E}}\left[r(s, a)+\gamma \max _{a^{\prime}} Q^{*}\left(s^{\prime}, a^{\prime}\right)\right]
$$

> 其中$s^{\prime} \sim P$表示，下一个状态$s^{\prime}$是在环境中，从$P(\cdot \mid s, a)$分布中采样获得的。

基于该Bellman方程，然后假设该近似器是带有参数$\phi$的神经网络$Q_{\phi}(s,a)$，同时也收集一组转换元组$(s,a,r,s^{\prime},d)$  $\mathcal{D}$。我们可以建立均方Bellman误差（**MSBE**）函数：
$$
L(\phi, \mathcal{D})=\underset{\left(s, a, r, s^{\prime}, d\right) \sim \mathcal{D}}{\mathrm{E}}\left[\left(Q_{\phi}(s, a)-\left(r+\gamma(1-d) \max _{a^{\prime}} Q_{\phi}\left(s^{\prime}, a^{\prime}\right)\right)\right)^{2}\right]
$$

> 对于参数 $d$，如果$d==True(1)$则说明$s^{\prime}$是处于终止状态，表示agent在该状态$s$之后没有获得其他奖励。

对于Q-Learning算法的函数近似：DQN和DDPG，大多都采用最小化MSBE损失函数，在这里用到了以下两个技术：

## 经验回放

通常用于训练深度神经网络的算法都采用经验回放池，其中$\mathcal{D}$表示之前的经验，为了使算法能够有稳定的行为，经验回放池应该足够大去包含更多的经验，但是保留所有的内容可能会降低学习的速度，但是保留的内容过少，可能会出现过拟合的现象，从而导致中断。

  > 之所以采用之前的经验，是因为 Bellman 方程不关心我们采用了哪个转换元组，选择了什么样的动作，或者在给定转换元组后发生了什么。因为最优 Q 函数应当满足 Bellman 方程的所有可能的转换元组，因此当最小化 MSBE 来拟合 Q 函数逼近时，任何经验转换元组都是公平的。
  
## 目标网络

目标网络之所以被称作"目标"，是因为当最小化MSBE误差时，需要让Q-function更逼近这个“目标”。同时，目标网络又依赖于我们训练的网络:$\phi$，这样会导致MSBE最小化变得不稳定。解决方案是使用一组参数逼近$\phi$时，具有一定的时间延迟，也就是说，目标网络的更新落后于第一个网络。通常表示目标网络的参数称为：$\phi_{targ}$。

在基于DQN的算法中，每隔一定固定数量的步数，目标网络就从主网络中复制更新一次。在DDPG风格的算法中，目标网络的更新，是通过polyak平均来实现的：
  $$
  \phi_{\text {targ }} \leftarrow \rho \phi_{\text {targ }}+(1-\rho) \phi
  $$

  > $\rho$是一个在 0 — 1之间，接近于1的一个超参数

## 在目标网络中计算最大动作

DDPG使用**目标策略网络**来计算一个近似使$Q_{\phi_{targ}}$最大的动作。该目标策略网络的更新与目标Q-function相同。

综上所述，DDPG的Q-Learning的目标是通过随机梯度下降使MSBE损失最小化：
$$
L(\phi, \mathcal{D})=\underset{\left(s, a, r, s^{\prime}, d\right) \sim \mathcal{D}}{\mathrm{E}}\left[\left(Q_{\phi}(s, a)-\left(r+\gamma(1-d) Q_{\phi_{\mathrm{targ}}}\left(s^{\prime}, \mu_{\theta_{\mathrm{targ}}}\left(s^{\prime}\right)\right)\right)\right)^{2}\right]
$$

> 其中$\mu_{\theta_{\mathrm{targ}}}$是目标策略

# 三、DDPG中的策略

在DDPG中，通过学习一种**确定性策略$u_{\theta}(s)$**，该策略可以得出使$Q_{\phi_{targ}}$最大化的动作。因为动作空间是连续的，并且我们假设Q-function相对于动作是可微的，我们可以执行梯度上升(仅针对策略参数)来解决。
$$
\max _{\theta} \underset{s \sim D}{\mathrm{E}}\left[Q_{\phi}\left(s, \mu_{\theta}(s)\right)\right]
$$

> 在这里，将Q-function参数视为常量

## DDPG的训练

DDPG以off-policy的方式来训练确定性策略，因为该策略是确定性的，如果使用on-policy的方式来进行探索，从一开始，就可能不会尝试各种动作去找到更多有用的学习信息。同时为了使DDPG策略**探索**的更好，我们在训练时给动作增加噪音。为了方便获得获得高质量的训练数据，可以在训练过程中减少该噪声的大小(但在实现时，通常会将噪声设为一个固值)

> 原始DDPG的作者建议使用与时间相关的OU噪声，但最近的研究表明，不相关的均值零高斯噪声表现的更好

在DDPG在进行训练时，通常设定一个**start_steps**参数，小于该值时动作的选择是随机采样的方式进行探索，大于该值后，就通过正常的DDPG来进行探索。

## DDPG伪代码

![](img/ddpg.png)

> 对于actor、critic网络的更新：
>
> critic网络更新：从经验回放池中采样一批数据，将$s'$放入目标actor网络中，得到$s'$状态下的动作$a'$，然后将状态 $s'$及其动作$a'$输入到目标cirtic网络中，得到相应的Q值，通过公式计算得出此时的目标值$y$，然后将其与$s,a$作为输入的Q值求均方误差，用以得出目标神经网络（定期更新）与预测神经网络（实时更新）的差距，通过梯度下降的方法，缩小两者的差距。
>
> actor网络更新：根据当前状态$s$，输入到actor网络中得到$s$状态下的动作$a$，将$s、a$输入到critic神经网络中，得到相应的Q值，根据梯度上升到方法，尽可能更新神经网络使其获得更大的Q值。
>
> 因此Q值越高对于actor网络的影响越好，进而critic网络，从一开始的选择较差的状态行为得到较低的Q值，到后来不断选择好的行为，得到较高的Q值，最终达到较好的效果。



### DDPG问题

- 探索能力较差、训练慢、不稳定、超参数较多。