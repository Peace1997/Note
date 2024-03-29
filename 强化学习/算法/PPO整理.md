

# PPO 

PPO (Proximal Policy Optimization) 是一种改进的策略梯度的算法，结合了**重要性采样**、**广义优势估计 GAE** 和 **截断式优化目标（KL 散度）** 等思想，对新旧策略的分布进行限制，保证每一次新计算出的策略能够和原策略相差不大，稳步提升算法的表现，降低约束优化问题复杂度。

## 总结

### TRPO & PPO 

**_（1）TRPO 实现方式_**
TRPO 通过可靠的方向和合适的步长，解决策略稳定提升的问题。利用两个技巧来实现上述目标
- **重要性采样**：利用旧策略采样的固定数据，对数据加权即可近似新策略下的期望，减小策略更新时目标函数的估计**方差**。
- **KL 散度**：通过 KL 散度控制新旧策略的**更新幅度**。

采用当前策略（更新前的策略，旧策略）去收集数据，用收集到的数据去计算一个更新方向，这个更新方向因为有**重要性采样的校正**，因为**有信赖域（Trust Region）的约束**【KL 散度】，因此可以保证策略的提升。


***（2）PPO 和 TRPO 算法的区别***
PPO 解决 TRPO **求解约束优化问题复杂度过高**的问题，主要的方法包括截断式优化目标的方法和显式 KL 散度的方法，对于 KL 散度具体来说：
- 在 TRPO（信任区域策略优化 (trust region policy optimization) PPO 前身）中把 KL 散度当作约束。
$$
J_{\mathrm{TRPO}}^{\theta^{\prime}}(\theta)=\mathbb{E}_{\left(s_t, a_t\right) \sim \pi_{\theta^{\prime}}}\left[\frac{p_\theta\left(a_t \mid s_t\right)}{p_{\theta^{\prime}}\left(a_t \mid s_t\right)} A^{\theta^{\prime}}\left(s_t, a_t\right)\right], \mathrm{KL}\left(\theta, \theta^{\prime}\right)<\delta
$$
- 在 PPO 中 KL 散度应用在优化目标函数的公式中：
$$
\begin{aligned}
&J_{\mathrm{PPO}}^{\theta^{\prime}}(\theta)=J^{\theta^{\prime}}(\theta)-\beta \mathrm{KL}\left(\theta, \theta^{\prime}\right) \\
&J^{\theta^{\prime}}(\theta)=\mathbb{E}_{\left(s_t, a_t\right) \sim \pi_{\theta^{\prime}}}\left[\frac{p_\theta\left(a_t \mid s_t\right)}{p_{\theta^{\prime}}\left(a_t \mid s_t\right)} A^{\theta^{\prime}}\left(s_t, a_t\right)\right]
\end{aligned}
$$


### PPO 1 & PPO 2

 PPO-Penalty（PPO 1）每次只使用新采样数据更新一次, 而 PPO-Clip （PPO 2）可以对同一批数据进行多次更新, 主要原因在于它们优化**目标函数**的不同性质。
 - PPO 1 的优化目标包含一个 KL 散度惩罚项, 用于约束新旧策略的差异，而 KL 散度是基于**状态分布**的, 状态分布会随着策略π_θ的变化而变化。如果在同一批数据上反复更新策略π_θ, 其对应的状态分布也会不断改变。. 这将导致 KL 散度约束项在多次更新后失去约束的意义。
 - PPO 2 的优化目标标函数是对重要性比率进行裁剪，来约束新旧策略的差异，这种**硬约束是不依赖于状态分布的**。即使在同一批固定数据上多次迭代更新，策略目标函数的形式并不会受到影响，新策略相对于旧策略即使产生较大差距也不会影响到策略目标函数的更新。

### PPO 实现技巧

***（1）新的优化目标***
**策略网络**：
基于重要性采样、截断式裁剪（或 KL 散度）、优势函数的方法设计了新的优化目标函数，通过最大化策略梯度来更新策略网络。
**价值网络**：
通过 reward to go 计算每个状态的回报值，通过 GAE 的方法计算优势值，用于指导价值网络的更新。

***（2）重要性采样***
PPO 采用了**重要性采样**的方法，利用旧策略采样的固定数据，对数据加权即可近似新策略下的期望，减小策略更新时目标函数的估计**方差**。可以利用旧策略采样的数据完成新策略的更新。所以 $\theta$ 可以利用 $\theta^k$ 采样的数据完成多次更新，而用 $\theta$ 采样的数据进行更新，只能更新一次。

在策略优化过程中, 我们通常需要根据采样自旧策略的轨迹数据来更新策略。但由于策略已发生变化, 直接使用旧数据估计新策略的优势函数等价值函数时会产生偏差。**重要性采样通过重新加权轨迹数据**, 将其重新适配于新策略的分布, 从而校正这种偏差。

***（3）分布纠正，控制策略更新幅度***
重要性采样可以利用旧策略分布完成新策略分布的更新，并对新旧策略分布差距进行**校正**，但是没有对新旧策略分布的差距进行**控制**。

PPO 为了避免不同分布之间差异过大，可以通过**KL 散度**、**裁剪**的方法来对分布差异进行纠正。而纠正的这个分布差异就是重要性采样的加权系数（重要性权重比率）。


## PPO 1
对于近端策略优化惩罚 (PPO-penalty)，有一个**自适应 KL 散度**（adaptive K divergence） 的纠正方法，其要最大化的目标函数为：
$$
\begin{aligned}
&J_{\mathrm{PPO}}^{\theta^k}(\theta)=J^{\theta^k}(\theta)-\beta \mathrm{KL}\left(\theta, \theta^k\right) \\
&J^{\theta^k}(\theta) \approx \sum_{\left(s_t, a_t\right)} \frac{p_\theta\left(a_t \mid s_t\right)}{p_{\theta^k}\left(a_t \mid s_t\right)} A^{\theta^k}\left(s_t, a_t\right)
\end{aligned}
$$
在这里将 KL 散度作为惩罚项，其中参数惩罚系数 $\beta$ 是可以动态调整的，我们需要事先设定一个可接受的 KL 散度的最大值 $KL_{max}$ 和最小值 $KL_{min}$，如果我们优化一次上述公式后：
- ${KL}\left(\theta, \theta^k\right)$ > $KL_{max}$，说明新旧分布差异是比较大，说明后面的惩罚项没有起到作用，因此应该增加惩罚力度，即**增大** $\beta$ 的值。
- ${KL}\left(\theta, \theta^k\right)$ < $KL_{min}$，说明新旧分布差异是比较小的，就说明后面的惩罚项太强了，因为我们更想要的优化的的目标是前一项，如果新旧分布差异过小的话，就有可能会倾向于优化后一项，因此我们减少 $\beta$ 的值。（“避免喧宾夺主”）

## PPO 2
对于近端策略优化裁剪（PPO-clip），策略目标函数里面是没有 KL 散度的，而是增加了一个**裁剪**操作。其要最大化的目标函数为：
$$
\begin{aligned}
J_{\mathrm{PPO} 2}^{\theta^k}(\theta) \approx \sum_{\left(s_t, a_t\right)} \min \left(\frac{p_\theta\left(a_t \mid s_t\right)}{p_{\theta^k}\left(a_t \mid s_t\right)} A^{\theta^k}\left(s_t, a_t\right),\right.
\left.\operatorname{clip}\left(\frac{p_\theta\left(a_t \mid s_t\right)}{p_{\theta^k}\left(a_t \mid s_t\right)}, 1-\varepsilon, 1+\varepsilon\right) A^{\theta^k}\left(s_t, a_t\right)\right)
\end{aligned}
$$
- **min** 表示将在原始目标函数与裁剪后的目标函数中选择一个较小的项；之所以选择较小项的原因是 PPO 想要每次更新的步幅小一点。
- **clip** 表示直接对分布差异的进行裁剪限制，当分布差异高于上限（$1 + \epsilon$ ） 和低于下限（ $1 - \epsilon$ ）时直接裁剪截断；而 PPO 1 中设定的最大值 $KL_{max}$、最小值 $KL_{min}$ 仅仅是判断条件。其中 $\epsilon$ 是一个超参数，通常设定为 0.1 或 0.2。裁剪效果如下图所示：
![400](img/PPO_clip.png)

对于整个目标函数来看，优势函数 $A^{\theta^k}\left(s_t, a_t\right)$ 的正负值会对目标函数的选取造成影响：
- A > 0 ; 说明当前的状态-动作对是好的，我们希望增大这个状态-动作对的概率 $p_\theta(a_{t}\mid s_{t})$，但是它与 $p_{\theta^k}(a_{t}\mid s_{t})$ 的比值不能超过 $1+\epsilon$ ；
- A < 0 ；说明当前状态-动作对是不好的，我们希望把 $p_{\theta}(a_{t} \mid s_{t})$ 减小。如果这时 $p_\theta(a_{t}\mid s_{t})$ 比 $p_{\theta^k}(a_{t}\mid s_{t})$ 还大的话，明显是错误的，我们需要尽可能的减小 $p_{\theta}(a_{t} \mid s_{t})$，但是它与 $p_{\theta^k}(a_{t}\mid s_{t})$ 的比值不能小于 $1-\epsilon$；
![](PPO_clip2.png)

之所以设定这样的裁剪方式，是 PPO 2 希望每次概率分布更新时，无论是增加某状态-动作对的概率还是减小状态-动作对的概率，都要有一定的限度，**不能使的两个分布差距相差过大**，这也就是 PPO 解决重要性采样缺点的方式。PPO 每次更新的步幅虽然小，但是每次都是有效的。

>例如在优化策略目标函数时，重要性权重比率越高，说明当前策略下轨迹数据的动作概率高于旧策略的动作概率，它对应的优势函数有可能为正，这时我们希望增大该状态-动作对出现的概率。而 KL 散度、裁剪就是对这个更新幅度进行约束。
>同理，重要性权重比率越低，说明当前策略下轨迹数据的动作概率低于旧策略的动作概率，其优势函数有可能为负，这时就要降低这个动作出现的概率，同样 KL 散度、裁剪就是对这个也需要对这个更新幅度进行约束

# 相关知识

## 重要性采样
**_（1）定义：_**
重要性采样(Importance Sampling)：用来从一个分布(提议分布 proposal distribution)中高效地采样,来近似另一个分布(目标分布 target distribution)的**期望值**。

**_（2）基本思想：_**
- 首先从一个较容易采样的提议分布q(x)中采样得到样本x
- 然后利用这些样本及其在目标分布p(x)和提议分布q(x)下的概率密度比值 p(x)/q(x),来近似目标分布p(x)的期望值。

**_（3）数学定义_**
对于给定一个在目标分布 p(x)下的期望:
$$
E_p[f(x)] = ∫f(x)p(x)dx
$$
我们可以利用重要性采样得到其估计值: 
$$
E_p[f(x)] = E_q\frac{f(x)p(x)}{q(x)}≈ \frac{1}{N} * Σ(f(x_i)\frac{p(x_i)}{q(x_i)} ) (其中x_{i} 采样于 q(x))
$$
只要我们有合适的提议分布q(x),并能够计算p(x)/q(x),那么就可以利用重要性采样来近似目标分布下的期望。

***（4）存在的问题***
对于重要性采样，虽然 q 的分布可以是任取的，但是实际上 **p、q 的分布不能差距过大**，否则可能会出现方差较大的问题。

通过下图可知，p、q 分布差距是比较大的，p 分布采样到的 x 主要集中在左侧，而 q 分布采样主要集中在右侧，但是如果采样方差计算的话，两者之间的差距就会表现出来。其中方差的计算公式为：$\operatorname{Var}[X]=E\left[X^2\right]-(E[X])^2$
$$
\begin{align*}
&\operatorname{Var}_{x \sim p}[f(x)]=\mathbb{E}_{x \sim p}\left[f(x)^2\right]-\left(\mathbb{E}_{x \sim p}[f(x)]\right)^{2}\\\\	
\operatorname{Var}_{x \sim q}\left[f(x) \frac{p(x)}{q(x)}\right] &=\mathbb{E}_{x \sim q}\left[\left(f(x) \frac{p(x)}{q(x)}\right)^2\right]-\left(\mathbb{E}_{x \sim q}\left[f(x) \frac{p(x)}{q(x)}\right]\right)^2\\
\\&=\mathbb{E}_{x \sim p}\left[f(x)^2 \frac{p(x)}{q(x)}\right]-\left(\mathbb{E}_{x \sim p}[f(x)]\right)^2
\end{align*}

$$
![[importance_sampling.png]]
通过对比两者的方差公式，主要的差距是体现在方差公式的第一项，从 q 分布的采样方差要比 p 分布的采样方差多一个重要性权重，因此如果重要性权重很大的话，他们的方差差距也是比较大的。

因此从 p、q 中只要采集够多的样本，就可以让他们他们的期望值是比较相似的，但是如果采样的样本不够多的话，由于两者方差比较大，就很容易使最后期望值的计算结果差距比较大。

**解决方法：**
- 从 p、q 分布中采集足够多的样本
- P、q 分布的差异不能过大

## GAE

**_定义_**
广义优势估计（Generalized Advantage Estimator ，GAE），这是一种通过平衡偏差和方差来计算**优势函数**的方法。其中，优势函数定义为期望累积奖励与值函数 (通常为状态值函数)的差值。

**高方差需要更多的样本来训练，偏差会导致不收敛或收敛结果较差。**
>使用全部奖励来估计状态值函数，尽管无偏但是方差大；使用目标值函数来估计状态-动作值函数，能够降低方差但是偏差较大。

**_计算方法_**
GAE 利用**时序差分**的思想, 结合**指数加权平均法**来进行优势函数的估计。
$$
\begin{aligned}
\hat{A}_t^{\mathrm{GAE}(\gamma, \lambda)} & :=(1-\lambda)\left(\hat{A}_t^{(1)}+\lambda \hat{A}_t^{(2)}+\lambda^2 \hat{A}_t^{(3)}+\ldots\right) \\
& =(1-\lambda)\left(\delta_t^V+\lambda\left(\delta_t^V+\gamma \delta_{t+1}^V\right)+\lambda^2\left(\delta_t^V+\gamma \delta_{t+1}^V+\gamma^2 \delta_{t+2}^V\right)+\ldots\right) \\
& =(1-\lambda)\left(\delta_t^V\left(1+\lambda+\lambda^2+\ldots\right)+\gamma \delta_{t+1}^V\left(\lambda+\lambda^2+\lambda^3+\ldots\right)\right. \\
& \left.\quad \quad \quad \quad+\gamma^2 \delta_{t+2}^V\left(\lambda^2+\lambda^3+\lambda^4+\ldots\right)+\ldots\right) \\
& =(1-\lambda)\left(\delta_t^V\left(\frac{1}{1-\lambda}\right)+\gamma \delta_{t+1}^V\left(\frac{\lambda}{1-\lambda}\right)+\gamma^2 \delta_{t+2}^V\left(\frac{\lambda^2}{1-\lambda}\right)+\ldots\right) \\
& =\sum_{l=0}^{\infty}(\gamma \lambda)^l \delta_{t+l}^V
\end{aligned}
$$

其中，
- $\gamma$为折扣因子，表示未来奖励对当前状态的影响程度。
- $\delta$为优势增量，表示为当前奖励加上后继状态的状态值估计, 再减去当前状态的状态值估计：$\delta_t=r_t+\gamma v\left (s_{t+1}\right)-v\left (s_t\right)$
- $\lambda$为平衡方差与偏差的系数，取值范围为 \[0,1]
	- $\lambda$=0 时等同于一步估计, 会引入偏差，但是方差更小
	- $\lambda$=1 时等同于蒙特卡罗采样, 无偏差，但是方差更大。	
$$
\begin{array}{ll}
\operatorname{GAE}(\gamma, 0): & \hat{A}_t:=\delta_t \quad=r_t+\gamma V\left(s_{t+1}\right)-V\left(s_t\right) \\
\operatorname{GAE}(\gamma, 1): & \hat{A}_t:=\sum_{l=0}^{\infty} \gamma^l \delta_{t+l}=\sum_{l=0}^{\infty} \gamma^l r_{t+l}-V\left(s_t\right)
\end{array}
$$

PPO使用GAE估计的优势函数值,作为策略目标函数中的采样权重,引导策略朝着提高奖励的方向更新。

## rewards to go
其中 rewards-to-go（$\hat{R}_t$）是计算每个状态的回报值。
$t$ 时刻的剩余可得回报 $\hat{R}_t$ 是指在一个 Trajectory $\tau=\left(s_1, a_1, r_1, s_2, \cdots, s_T\right)$ 中, $t$ 时刻及其以后所有奖励的累计, 其计算如下所示。
$$
\hat{R}_t=\sum_{k=t}^T r_k
$$
它可以通过**最小化损失函数的方法，指导 Critic 网络（value function）的更新**，使得 Critic 网络更好的预测每个状态的回报值。

## 熵正则化

熵正则化(Entropy Regularization)是强化学习中的一种技术,通过在优化目标函数中加入一个熵项,来鼓励策略具有一定的随机性和探索能力。在PPO(Proximal Policy Optimization)算法中,熵正则化被应用来**改善探索**和提高**训练稳定性**。
$$
- c * H(π(.|s))
$$
熵正则化系数 c 的大小需要适当调节。c 过大会导致策略过于随机探索,训练效率低下;c 过小则可能导致早期收敛或陷入次优。
