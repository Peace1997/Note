当前单智能体强化学习常采用的算法主要包括：Twin Delayed DDPG (TD3)、Soft Actor-Critic (SAC)和 Proximal Policy Optimization (PPO)。


# 一、简述

PPO 是一种改进的策略梯度的算法，结合了**重要性采样**、**广义优势估计** 和 **截断式优化目标（KL散度）** 等思想，对新旧策略的分布进行限制，保证每一次新计算出的策略能够和原策略相差不大，稳步提升算法的表现，降低约束优化问题复杂度。

PPO算法采用了 Actor-Critic (AC) 架构，由两个网络组成：
1. Actor（策略网络）：生成动作的策略；通过与环境的交互，采样动作以最大化回报。Actor 使用策略梯度方法更新其策略参数。
2. Critic（价值网络）：评估当前策略的表现；通过估计状态值（value function）来指导策略的优化。Critic 通过最小化价值损失函数来更新其参数。
![[ppo.png]]


通过重要性采样对新旧策略分布偏差进行校正，但是没有对新旧策略分布的差距进行控制。PPO 为了避免不同分布之间差异过大，可以通过KL 散度（**PPO1**）、裁剪（**PPO2**）的方法来对分布差异进行纠正。

# 二、策略网络

## PPO 1

对于近端策略优化惩罚 (PPO-penalty)，有一个**自适应** **KL** **散度**（adaptive K divergence） 的纠正方法，策略网络的目标函数为：

$$L^{K L P E N}(\theta)=\hat{\mathbb{E}}_t\left[\frac{\pi_\theta\left(a_t \mid s_t\right)}{\pi_{\theta_{\text {old }}}\left(a_t \mid s_t\right)} \hat{A}_t-\beta \operatorname{KL}\left[\pi_{\theta_{\text {old }}}\left(\cdot \mid s_t\right), \pi_\theta\left(\cdot \mid s_t\right)\right]\right]$$

## PPO2

对于近端策略优化裁剪（PPO-clip），策略目标函数里面是没有 KL 散度的，而是增加了一个**裁剪**操作。策略网络的目标函数为：

$$ L^{C L I P}(\theta)=\hat{\mathbb{E}}_t\left[\min \left(r_t(\theta) \hat{A}_t, \operatorname{clip}\left(r_t(\theta), 1-\epsilon, 1+\epsilon\right) \hat{A}_t\right)\right] $$

-  $r_t(\theta)=\frac{\pi_\theta\left(a_t \mid s_t\right)}{\pi_{\theta_{\text {old }}}\left(a_t \mid s_t\right)}$  ：新旧策略的概率比
- $\hat{A}_t$：优势函数值，可通过GAE计算
  
# 三、价值网络
对于PPO1和PPO2的价值网络，通过最小化价值函数损失来更新网络参数：
$$L^{VF}_t=\hat{\mathbb{E}}\left[(V_\theta(s_t)-V^{target}_t)^2\right]$$

- $V_{\theta}(s_t)$：$s_t$状态下预测价值
- $V_{target}$$：$$s_t$状态下真实价值，可通过rewards-to-go计算

# 四、PPO实现技巧

## Generalized Advantage Estimator

  GAE 是一种计算优势函数的方法，这是一种平衡优势函数估计中偏差和方差的方法。

$$\begin{aligned} \hat{A}_t^{\mathrm{GAE}(\gamma, \lambda)} & =\sum_{l=0}^{\infty}(\gamma \lambda)^l \delta_{t+l}^V \end{aligned}$$

- $\gamma$是折扣因子（Discount Factor）
- $\lambda$是 GAE 的平滑参数（介于 0 和 1 之间，平衡方差与偏差）
	- $\lambda$ 越小考虑未来时间步的奖励越少，因此更贴近实际的优势函数（标准优势函数只关注当前时刻的优势值），所以偏差更小，但可能会导致估计结果对每个时间步的变化非常敏感，方差增大，反之亦然。
- $\delta$是时序差分误差（Temporal Difference Error）：$$\delta_{t}^V=r+V(s_{t+1})-V(s_t)$$

详细推导可见：
![[Pasted image 20240607153626.png]]
> 
优势函数在 RL 中常用于确定一个动作相对于平均动作的优势，优势函数有助于减少方差，使学习更加稳定。

## 标准化优势函数
标准化优势函数（即对其进行减均值除以标准差的操作）是一个常见的步骤。这一操作有几个重要的原因：
- **减少数值不稳定性**：优势函数的值可以有很大的范围。直接使用未标准化的优势函数可能会导致数值不稳定性，尤其是在使用梯度下降优化算法时。
- **加快收敛速度**： 通过标准化优势函数，可以使得优化过程更加稳定和快速。标准化可以使梯度的尺度更加均匀，从而使得梯度下降优化器的更新步伐更加均衡

## rewards-to-go
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

### SAC vs PPO
在 SAC 中熵正则化被显式的
SAC（Soft Actor-Critic）和 PPO（Proximal Policy Optimization）是两种强化学习算法，它们在处理策略熵（entropy regularization）时采用了不同的方法。以下是它们的主要区别和求导差异的详细解释。

***SAC 的熵正则化***

SAC 通过引入熵正则化项来鼓励策略的探索。其目标函数包括一个熵项，使得策略不仅要最大化期望回报，还要最大化熵。SAC 的目标是最大化以下目标函数：

$$
J (\pi) = \mathbb{E}_{(s_t, a_t) \sim \rho_{\pi}} \left[ \sum_{t=0}^{\infty} \gamma^t (r (s_t, a_t) + \alpha \mathcal{H}(\pi (\cdot|s_t))) \right]
$$

在策略更新（Actor Update）中，梯度为：

$$
\nabla_{\theta} J_{\pi}(\theta) = \mathbb{E}_{s_t \sim \mathcal{D}, a_t \sim \pi_{\theta}} \left[ \nabla_{\theta} \log \pi_{\theta}(a_t|s_t) (Q_{\phi}(s_t, a_t) - \alpha \log \pi_{\theta}(a_t|s_t) + \alpha H (\pi)) \right]
$$

这里，熵正则化项 \(\alpha \log \pi_{\theta}(a_t|s_t)\) 的梯度被显式地计算并用于更新策略参数。

 ***PPO 的熵正则化***

PPO 通过在其损失函数中引入一个熵奖励项来鼓励策略的探索。PPO 的主要目标是通过最大化一个剪切的目标函数来限制策略的更新步幅。其目标函数为：

$$
L^{CLIP}(\theta) = \mathbb{E}_t \left[ \min \left ( r_t (\theta) \hat{A}_t, \text{clip}(r_t (\theta), 1 - \epsilon, 1 + \epsilon) \hat{A}_t \right) \right] + \beta \mathcal{H}(\pi (\cdot|s_t))
$$


PPO 的梯度为：
$$
\nabla_{\theta} L^{CLIP}(\theta) = \nabla_{\theta} \mathbb{E}_t \left[ \min \left ( r_t (\theta) \hat{A}_t, \text{clip}(r_t (\theta), 1 - \epsilon, 1 + \epsilon) \hat{A}_t \right) \right] + \beta \nabla_{\theta} \mathcal{H}(\pi (\cdot|s_t))
$$

这里，熵正则化项 \(\beta \mathcal{H}(\pi (\cdot|s_t))\) 的梯度被添加到总梯度中，用于更新策略参数。

***主要区别和求导差异***

1. **目标函数**：
   - **SAC**：最大化累积奖励和策略熵的组合，熵项被显式地包含在目标函数中，并且有一个可调参数 \(\alpha\) 控制熵的权重。
   - **PPO**：最大化一个包含剪切项和熵奖励的目标函数，熵项作为额外的奖励加在目标函数中，并且由参数 \(\beta\) 控制熵的权重。

2. **策略更新**：
   - **SAC**：策略更新包含熵项的梯度 $(\alpha \log \pi_{\theta}(a_t|s_t)$)，这使得策略在更新时直接考虑了熵的影响。
   - **PPO**：策略更新包含熵奖励项的梯度 $(\beta \mathcal{H}(\pi (\cdot|s_t)))$，熵作为奖励的一部分，间接影响策略更新。

3. **熵项的处理**：
   - **SAC**：熵项直接影响 Q 函数的估计，并且在策略更新时显式考虑。
   - **PPO**：熵项作为奖励的附加部分，影响策略更新的约束和优化过程。

### 总结

尽管 SAC 和 PPO 都使用熵正则化来鼓励策略的探索，它们的实现和求导方式有所不同。SAC 在策略更新中直接考虑熵项，而 PPO 则在目标函数中添加熵奖励项。SAC 的梯度计算直接包含熵项的影响，而 PPO 的梯度计算则间接通过熵奖励项来影响策略更新。这些差异导致了两者在处理策略熵正则化时的不同行为和效果。

## PPO 并行训练

### 实现方式

**DistributedDataParallel (DDP)** 是 PyTorch 官方推荐的多 GPU 训练方式。比传统的 `DataParallel` 更快、更节省显存、更适合多机多卡。



| 特性       | 说明                                          |
| ---------- | --------------------------------------------- |
| 多卡高效   | 每个GPU启动1个独立进程（不是1个进程控制多卡） |
| 显存占用少 | 不像 `DataParallel` 那样主卡多负担            |
| 跨机支持   | 适合多机多卡训练                              |
| 强制同步   | 通过All-Reduce让每卡参数同步                  |

PPO + DDP 进行并行训练：
- **分布式采样（Distributed Sampling）**：每个进程采集自己的数据，存储在各自进程的私有内存空间，训练时各自用自己的数据计算梯度。
- **参数同步（Shared Parameters）**：所有进程的梯度会聚合，参数同步，保证所有进程的模型参数一致。
	- 所有 GPU **独立计算 loss & backward**
	- DDP **自动 all-reduce**，平均所有 GPU 的梯度，此操作是在所有进程/GPU 上同步聚合的，而不是在一张卡上独立完成的，全部卡同时获得结果。
	- 每个 GPU 调用 `optimizer.step()` 后，模型参数完全一致。


DDP 的设计就是**去中心化、全并行**，避免某卡负载变高。


>[! NOTE]
>
>- **DDP 是同步式分布式训练机制** —— 在反向传播后，会阻塞（等待）所有进程完成，直到每个进程计算完梯度，才进行参数同步（AllReduce）。**同步训练的本质就是要牺牲一点快的 GPU 的效率，换取算法正确性与全局一致性**。
>-  **PPO+DDP 只同步策略和梯度，不同步 episode 数**。不同卡 rollout 的环境随机性决定了 episode 数可以不同，这完全正常。
>- **ALL-Reduce** 本质上是一种**集合通信操作**，每张卡上的模型权重在更新前完全一致，更新后也保持一致，梯度同步在通信中由 NCCL 实现（所有设备组成一个环，分步传递梯度，聚合后再分发回来，性能取决于最慢的卡）。



### 调参训练

多卡训练相比于多卡超参数调整可以略有不同：
  ***1. 学习率 (Learning Rate) - 线性缩放法则***
- 理由：总 batch size 变为单卡 $N$ 倍，理论上学习率也应扩大 $N$ 倍；
- 例如：单卡 $lr=3e^{-4}$，8卡 DDP 建议试 $lr=2.4e^{-3}$；
- 但PPO中实际可能需要 **缩小一些（如2-4倍而非8倍）**，避免过大破坏 clip 机制。
---
 ***2. 采样长度 / batch size***
- 多卡后样本采集数：$num\_step\_per\_env \times num_envs \_len \times N$**；
- 对于两卡以上，可以尝试将 num_steps_per_env 提升至 64～256

---

***3. PPO-specific: clip range, entropy coeff***
- **clip range（如0.2）**：多卡数据更充足，策略更新 variance 变小，可考虑 **缩小为0.1~0.15**；
- **entropy coeff（如0.01）**：多卡时探索充分，略微减小 **鼓励收敛**；
- 但这两项依赖实验调优，不是强规则。

---
***4. 梯度裁剪（gradient clipping）***
- 多卡梯度加和后，**梯度 norm 可能更大**；
- 推荐 clip 得更严些（如 0.5）避免爆炸。

---
****5. 训练轮次（ppo_epochs）***
- 多卡总样本量充足，可考虑**减少每次 update 的 epoch 数（如从10减为5）**；
- 可提升训练速度，但需要保证收敛。

---
***6. 学习率调度（warmup, cosine decay 等）***
- 多卡训练通常会在**前几千步warmup**，防止初期大梯度破坏模型；
- 强烈建议多卡 PPO 配合 warmup。


## 解决多任务冲突与灾难性遗忘
### Curriculum Learning 


让策略“先学简单任务” → 再逐步解锁复杂任务；避免 early PPO 被复杂任务干扰导致复杂任务难学的现象。



### Reward Shaping

让不同难度任务的 reward 设计不同（早期阶段复杂任务 reward 降低，避免复杂任务优势值(Advantage)为负、影响 PPO 更新方向）；后期逐步恢复正常。



### MoE

**MoE**是一种特殊的网络结构，不是让“单一 policy 负责所有任务”，而是：
$$\pi(a|s) = \sum_{i=1}^N g_i(s) \cdot \pi_i(a|s)$$

- 有 **N个Expert子Policy网络**（例如：Expert1负责走路，Expert2负责跳跃...）；
- **Gating network**（门控网络）根据输入状态/latent决定每个expert权重 $g_i(s)$；
- Policy 实际输出 = 各 expert 加权平均。

Gating 网络根据当前 clip latent $z$或环境状态 $s$，动态选择 Expert：
$$\text{Gating}(z) \rightarrow [0.9, 0.1, 0.0] \quad \text{（大概率使用 Expert 1）}$$

### EWC
EWC（Elastic Weight Consolidation）是一种在多任务或持续学习场景下**防止旧任务被遗忘的技术**。其核心思想就是对已经训练好的重要权重施加弹性保护，让后续训练不会轻易改变这些权重。


训练目标变为：

$$L(\theta) = L_{\text{PPO}}(\theta) + \frac{\lambda}{2} \sum_i F_i (\theta_i - \theta_i^*)^2$$

其中：

| 符号                       | 含义                                |
| ------------------------ | --------------------------------- |
| $L_{\text{PPO}}(\theta)$ | 当前任务（新 clip）的 PPO 损失              |
| $\theta_i$               | 当前参数第 i 个                         |
| $\theta_i^*$             | 旧任务（如 simple clip 训练完成后）的权重第 i 个值 |
| $F_i$                    | 第 i 个参数对旧任务的重要性（近似 Fisher 信息矩阵）   |
| $\lambda$                | 正则系数（控制 EWC 强度）                   |



# 五、参考

- https://github.com/leggedrobotics/rsl_rl/tree/master
- PPO 论文：Proximal Policy Optimization Algorithms
- GAE 论文：High-Dimensional Continuous Control Using Generalized Advantage Estimation

