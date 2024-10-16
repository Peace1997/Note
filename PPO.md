# 一、简述

PPO 是一种改进的策略梯度的算法，结合了**重要性采样**、**广义优势估计** 和 **截断式优化目标（KL 散度）** 等思想，对新旧策略的分布进行限制，保证每一次新计算出的策略能够和原策略相差不大，稳步提升算法的表现，降低约束优化问题复杂度。

PPO 算法采用了 Actor-Critic (AC) 架构，由两个网络组成：
1. Actor（策略网络）：生成动作的策略；通过与环境的交互，采样动作以最大化回报。Actor 使用策略梯度方法更新其策略参数。
2. Critic（价值网络）：评估当前策略的表现；通过估计状态值（value function）来指导策略的优化。Critic 通过最小化价值损失函数来更新其参数。
![[ppo.png]]


通过重要性采样对新旧策略分布偏差进行校正，但是没有对新旧策略分布的差距进行控制。PPO 为了避免不同分布之间差异过大，可以通过 KL 散度（**PPO 1**）、裁剪（**PPO 2**）的方法来对分布差异进行纠正。

# 二、策略网络

## PPO 1

对于近端策略优化惩罚 (PPO-penalty)，有一个**自适应** **KL** **散度**（adaptive K divergence） 的纠正方法，策略网络的目标函数为：

$$L^{K L P E N}(\theta)=\hat{\mathbb{E}}_t\left[\frac{\pi_\theta\left(a_t \mid s_t\right)}{\pi_{\theta_{\text {old }}}\left(a_t \mid s_t\right)} \hat{A}_t-\beta \operatorname{KL}\left[\pi_{\theta_{\text {old }}}\left(\cdot \mid s_t\right), \pi_\theta\left(\cdot \mid s_t\right)\right]\right]$$

## PPO 2

对于近端策略优化裁剪（PPO-clip），策略目标函数里面是没有 KL 散度的，而是增加了一个**裁剪**操作。策略网络的目标函数为：

$$ L^{C L I P}(\theta)=\hat{\mathbb{E}}_t\left[\min \left(r_t(\theta) \hat{A}_t, \operatorname{clip}\left(r_t(\theta), 1-\epsilon, 1+\epsilon\right) \hat{A}_t\right)\right] $$

-  $r_t(\theta)=\frac{\pi_\theta\left(a_t \mid s_t\right)}{\pi_{\theta_{\text {old }}}\left(a_t \mid s_t\right)}$  ：新旧策略的概率比
- $\hat{A}_t$：优势函数值，可通过 GAE 计算
  
# 三、价值网络
对于 PPO 1 和 PPO 2 的价值网络，通过最小化价值函数损失来更新网络参数：


$$L^{VF}_t=\hat{\mathbb{E}}\left[(V_\theta(s_t)-V^{target}_t)^2\right]$$

- $V_{\theta}(s_t)$：$s_t$状态下预测价值
- $V_{target}$$：$$s_t$状态下真实价值，可通过 rewards-to-go 计算

# 四、PPO 实现技巧

## Generalized Advantage Estimator

  GAE 是一种计算优势函数的方法，这是一种平衡优势函数估计中偏差和方差的方法。

$$\begin{aligned} \hat{A}_t^{\mathrm{GAE}(\gamma, \lambda)} & =\sum_{l=0}^{\infty}(\gamma \lambda)^l \delta_{t+l}^V \end{aligned}$$

- $\gamma$是折扣因子（Discount Factor）
- $\lambda$是 GAE 的平滑参数（介于 0 和 1 之间，平衡方差与偏差）
- $\delta$是时序差分误差（Temporal Difference Error）：
$$\delta_{t}^V=r+V(s_{t+1})-V(s_t)$$

详细推导可见：
![[Pasted image 20240607153626.png]]
> 
优势函数在 RL 中常用于确定一个动作相对于平均动作的优势，优势函数有助于减少方差，使学习更加稳定。

## 标准化优势函数
标准化优势函数（即对其进行减均值除以标准差的操作）是一个常见的步骤。这一操作有几个重要的原因：
- **减少数值不稳定性**：优势函数的值可以有很大的范围。直接使用未标准化的优势函数可能会导致数值不稳定性，尤其是在使用梯度下降优化算法时。
- **加快收敛速度**： 通过标准化优势函数，可以使得优化过程更加稳定和快速。标准化可以使梯度的尺度更加均匀，从而使得梯度下降优化器的更新步伐更加均衡

## Rewards-to-go
"rewards-to-go" 是一种计算未来回报的方法，在 PPO 算法中通过优势函数来计算回报：

$$
\hat{R}_t=\sum_{k=0}^T \gamma^t r_{k+t}
$$
- $\gamma$是折扣因子（Discount Factor）
- $r$ 是即时奖励
使用这些 rewards-to-go 作为目标值，通过回归来拟合价值函数

## Entropy Regularization
在 PPO 中，熵正则化是一种辅助项，用于在策略优化过程中增加一些随机性，防止策略过早收敛到局部最优。
  $$
- c * S(π(.|s))
$$
- $c$ ：熵正则化系数，平衡熵正则化与其他损失项之间的关系。