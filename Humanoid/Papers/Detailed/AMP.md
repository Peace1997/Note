

## Overview 

论文提出一种对抗性运动先验（Adversarial Motion Priors，AMP）的框架，指导物理模拟角色的应该执行怎样的任务，以及以什么样的运动风格执行该任务。**AMP 算法能够在完成高级任务目标的同时，模仿数据集中的运动风格**。该框架利用**对抗性模仿学习**（Adversarial Imitation Learning.），通过无结构的运动数据样本训练一个对抗性判别器，作为运动风格的先验，指导**强化学习算法**训练物理模拟角色完成各种任务。
![[Pasted image 20241130114644.png|500]]


______
### Framework
AMP 算法通过结合**目标条件强化学习**（Goal-Conditioned Reinforcement Learning ）和**生成对抗模仿学习** （Generative Adversarial Imitation Learning） 的方法，用于从大型非结构化运动数据集中学习运动先验，并控制角色的行为风格。

- **判别器**：用于预测输入的状态转换是来自真实运动数据集的 "真实" 样本，还是由角色策略生成的 "虚假" 样本。
	- 输入：是连续的状态转换 ($s_t, s_{t+1}$)，描述了角色在两个连续时间步长的运动。
	- 输出：策略网络生成的数据与真实数据集的数据的相似性评分。

- **生成器**：由强化学习算法构成，结合了任务奖励和风格奖励，使用近端策略优化 (PPO) 算法进行训练。

- **奖励函数**：任务奖励 $r^G$ 和风格奖励 $r^S$ 的线性组合
	- 任务奖励$r^G$ ： 特定于任务，用于定义角色应该满足的高级目标
	- 风格奖励$r^S$：由判别器提供，用于鼓励角色产生类似于数据集中运动的运动
$$
r\left(\mathrm{~s}_t, \mathrm{a}_t, \mathrm{~s}_{t+1}, \mathrm{~g}\right)=w^G r^G\left(\mathrm{~s}_t, \mathrm{a}_t, \mathrm{~s}_t, \mathrm{~g}\right)+w^S r^S\left(\mathrm{~s}_t, \mathrm{~s}_{t+1}\right) .
$$


### Contribution

 **1. 对抗性运动先验架构**：该方法将目标条件强化学习与生成对抗模仿学习方法相结合，使角色能够从大型非结构化运动数据集中学习，鼓励角色在执行高级目标任务（期望速度、期望目标点）时采用类似于数据集中运动的行为。
> 该框架不需要像传统的动作捕捉技术那样手动设计跟踪目标或进行基于相位的运动同步

**2. 稳定对抗性训练的技巧**
	- **从观察中模仿**（Imitation from Observations）： 将 GAIL 扩展到只有状态可观察的设置，允许使用仅包含状态信息的运动片段进行训练，无需提供专家动作。
	- **最小二乘判别器**（Least-Squares Discriminator）： 采用最小二乘 GAN（LSGAN）的损失函数，以解决标准 GAN 目标中梯度消失的问题，从而提高训练的稳定性和结果的质量。
	- **判别器观测**（Discriminator Observations）： 精心设计判别器使用的特征集，以提供对确定给定运动特征至关重要的信息，从而为策略提供有效的反馈
	- **梯度惩罚**（Gradient Penalty）： 使用梯度惩罚来减轻对抗性训练过程中的不稳定性，从而提高训练的稳定性和性能。
_________
## Adversarial Motion Prior

### Imitation from Observations
GAIL 通常需要状态-动作对作为输入，由于采集的运动片段数据集并未提供状态迁移的动作，因此 AMP 只需要状态转换 $(s_t, s_{t+1})$ 作为输入。这使得 AMP 可以直接从运动片段数据中学习，而无需访问专家的动作信息。

$$
\underset{D}{\arg \min }-\mathbb{E}_{d^{\mathcal{M}}\left(\mathrm{s}, \mathrm{~s}^{\prime}\right)}\left[\log \left(D\left(\mathrm{~s}, \mathrm{~s}^{\prime}\right)\right)\right]-\mathbb{E}_{d^\pi\left(\mathrm{s}, \mathrm{~s}^{\prime}\right)}\left[\log \left(1-D\left(\mathrm{~s}, \mathrm{~s}^{\prime}\right)\right)\right] .
$$

因此判别器的输入为**连续两帧**的状态观测信息，这个观测信息可以是来单独来自于专家数据，也可以是单独来自于智能体与环境的观测信息，而不是专家数据和智能体交互的观测数据的组合。通过最小化上面的损失函数，来优化判别器。

### Least-Squares Discriminator

在标准 GAN 中常采用 Sigmoid 交叉熵作为损失函数，随着 Sigmoid 函数饱和，可能会出现梯度消失的情况。因此，在 AMP 采用**最小二乘 GAN (LSGAN) 的损失函数**，
$$
\underset{D}{\arg \min } \mathbb{E}_{d^{\mathcal{M}}\left(\mathrm{s}, \mathrm{~s}^{\prime}\right)}\left[\left(D\left(\mathrm{~s}, \mathrm{~s}^{\prime}\right)-1\right)^2\right]+\mathbb{E}_{d^\pi\left(\mathrm{s}, \mathrm{~s}^{\prime}\right)}\left[\left(D\left(\mathrm{~s}, \mathrm{~s}^{\prime}\right)+1\right)^2\right] .
$$

判别器网络的评分不是 1 和-1 的离散值的，而是在 -1 到 1 之间的**连续值**。对于风格奖励的设置，设定为 \[0,1]。
$$
r^S\left(\mathrm{~s}_t, \mathrm{~s}_{t+1}\right)=\max \left[\begin{array}{ll}
0, & \left.1-0.25\left(D\left(\mathrm{~s}_t, \mathrm{~s}_{t+1}\right)-1\right)^2\right] .
\end{array}\right.
$$

Sigmoid 交叉熵：
$$
\underset{D}{\arg \min }-\mathbb{E}_{d^{\mathcal{M}}\left(\mathrm{s}, \mathrm{~s}^{\prime}\right)}\left[\log \left(D\left(\mathrm{~s}, \mathrm{~s}^{\prime}\right)\right)\right]-\mathbb{E}_{d^\pi\left(\mathrm{s}, \mathrm{~s}^{\prime}\right)}\left[\log \left(1-D\left(\mathrm{~s}, \mathrm{~s}^{\prime}\right)\right)\right] .
$$
详细 GAN 描述： [[GAN]]


### Discriminator Observations
  
AMP 使用观察映射 $\phi(s)$ 从状态中提取与确定运动特征相关的特征，然后再将这些特征输入到判别器中，判别器的输入为：
**这些特征包括：**
- 基体的线速度和角速度
- 关节位置
- 关节速度
- 末端执行器（例如手和脚）的位置（笛卡尔空间）
- 末端执行器（例如手和脚）的位置（笛卡尔空间）
- 末端执行器（例如手和脚）的位置（笛卡尔空间）

对观察数据进行映射，可以提取出通用性运动特征，然后将这些特征输入到判别器中，这样做的好处是：通过观测数据的特征提取，判别器可以适配不同的问题。


### Gradient Penalty

为解决生成器出现的**过度拟合和模式崩溃**的问题，该文提出了**梯度惩罚**的方法，该惩罚项与真实数据流形上**判别器输出的梯度范数的平方成正比**。这样做可以避免判别器过度强大，导致其对生成器生成的样本过于敏感，梯度惩罚阻止判别器对输入的细微变化过于敏感，从而避免生成过于陡峭的决策边界。
$$\begin{aligned}
\arg\operatorname*{min}_{D} & \mathbb{E}_{d^{M}(\mathrm{s,s^{\prime}})}\left[\left(D(\Phi(\mathrm{s}),\Phi(\mathrm{s^{\prime}}))-1\right)^{2}\right] \\
 & +\mathbb{E}_{d^{\pi}(s,s^{\prime})}\left[\left(D\left(\Phi(s),\Phi(s^{\prime})\right)+1\right)^{2}\right] \\
 & +\frac{w^{\mathrm{gp}}}{2}\mathbb{E}_{d^{M}(\mathrm{s,s^{\prime}})}\left[\left\|\nabla_{\phi}D(\phi)\right|_{\phi=(\Phi(\mathrm{s}),\Phi(\mathrm{s^{\prime}}))}\right\|^{2}
\end{aligned}$$
- $\frac{w^{\mathrm{gp}}}{2}$为人为设定的系数

## Summary

### Details

***1. 判别器损失 && 风格奖励***
- 在判别器损失函数的计算过程中，判别器需要进行两次判定：
	- 第一次为从专家数据集中随机采样 mini-batch 大小的数据，并对连续的两帧数据进行打分 $d^M(s,s')$；
	- 第二次为从智能体与环境交互的历史数据集中随机采样 mini-bacth 的数据，并对其中连续两真的数据进行打分 $d^\pi(s,s')$
	- 根据两次判别的打分，计算判别器的损失。
- 在风格奖励计算过程中，仅进行一次判定，即前后两帧$d^\pi(s_t,s_{t+1}$)的本体观测数据，
- 在风格奖励计算过程中，仅进行一次判定，即前后两帧$d^\pi(s_t,s_{t+1}$)的本体观测数据，



### AMP & GAIL

**相同点：**

- **两者都使用对抗性判别器**： AMP 和 GAIL 都使用一个对抗性判别器来区分专家演示的行为和策略生成的行为。判别器充当奖励函数，鼓励策略生成与专家演示相似的行为。
- **两者都使用强化学习**： AMP 和 GAIL 都使用强化学习算法（例如 PPO）来训练策略，以最大化从判别器获得的奖励。

**不同点：**

- **输入数据**： GAIL 通常需要状态-动作对作为输入，而 AMP 只需要状态转换 $(s_t, s_{t+1}$) 作为输入。这使得 AMP 可以直接从运动片段数据中学习，而无需访问专家的动作信息。
- **目标函数**： GAIL 最初的目标是最小化策略生成的数据分布与专家演示的数据分布之间的 Jensen-Shannon 散度。 AMP 则采用最小二乘 GAN (LSGAN) 的损失函数，旨在最小化 Pearson χ2 散度.
- **梯度惩罚**： AMP 使用梯度惩罚来增强训练稳定性，而 GAIL 并没有明确要求使用梯度惩罚。梯度惩罚通过惩罚数据流形上的非零梯度，防止判别器过度拟合，并使策略能够更有效地探索状态空间。


## Coding Implementation