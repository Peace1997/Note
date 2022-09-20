## Decision Transformer: Reinforcement Learning via Sequence Modeling 
>Chen L, Lu K, Rajeswaran A, et al. Decision transformer: Reinforcement learning via sequence modeling[J]. Advances in neural information processing systems, 2021, 34: 15084-15097.


### 简述
RL Transformer 的开山之作了-- Decision Transformer (DT) 
将其强化学习抽象为序列建模模型

#### 什么是 DT

**D**ecision **T**ransformer 是一种**自回归模型**。在 NLP 任务中，自回归模型是指给定前面的单词，预测当前时刻的单词，当前时刻的单词, 即计算 $p\left(x_t \mid x_{t-1}, x_{t-2}, \cdots, x_1\right)$ 。而在 DT 中, 给定前面的剩余可得回报 (Return-to-go)、状态以及动作, 即可预测当前应该执行的动作, 即计算 $p\left(a_t \mid s_t, \hat{R}_t, a_{t-1}, s_{t-1}, \hat{R}_{t-1}, \cdots, a_1, s_t, \hat{R}_1\right)$

>自回归模型（Autoregressive model，简称AR模型），是统计上一种处理时间序列的方法，

#### 为什么需要 Decision Transformer

利用 Transformer 强化的表征和时序建模能力，对 RL 问题转换为序列建模问题，直接输出动作（做出决策），需要要去计算策略梯度、去拟合价值函数。

### 整体流程
如同传统的自回归模型


![](img/architecture.gif)
#### 输入

$(\hat{R}_1, s_1, a_1, \hat{R}_2, s_2, a_2, \cdots, \hat{R}_T, s_T)$

作者在输入序列中之所以使用 [剩余可得回报](PPO#rewards to go) 而不是即时奖励, 是为了使模型能基于末来的期望回报来生成动作 , 而不是基于过去的回报，给定 $\hat{R}_t, \cdots, \hat{R}_1$ 就相当于给定了 $r_1, \cdots, r_T$. 因此，一个Trajectory可以使用剩余可得回报表示
$$
\tau=\left(\hat{R}_1, s_1, a_1, \hat{R}_2, s_2, a_2, \cdots, \hat{R}_T, s_T, a_T\right)
$$

#### 输出
$a_T$
