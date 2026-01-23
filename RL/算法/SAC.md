SAC 算法综合 TD3 的一些列优点，在策略网络更新过程种，同时增加权重系数，用以权衡最大化奖励和最大化熵之间的权重，从而权衡策略的探索和利用。

无模型强化学习采用 SAC 算法, 该算法基于 Actor-Critic 网络结构, 内部的深度神经网络结构主要由双 $\mathrm{Q}$ 网络 $Q_{\omega_j}$ 和策略网络 $\pi_\theta$ 构成。双 $\mathrm{Q}$ 网络的输入为状态和动作信息, 输出为状态动作对的 $Q$ 值, 使网络参数的更新更稳定, 双 $Q$ 网络的训练损失函数如下所示:
$$
\begin{gathered}
L=\frac{1}{N} \sum_{i=1}^N\left(y_i-Q_{\omega_j}\left(s_i, a_i\right)\right)^2 . \\
y_i=r_i+\gamma \min _{j=1,2} Q_{\omega_j^{-}}\left(s_{i+1}, a_{i+1}\right)-\alpha \log \pi_\theta\left(a_{i+1} \mid s_{i+1}\right), a_{i+1} \sim \pi_\theta\left(\mid s_{i+1}\right) .
\end{gathered}
$$

其中, **$\alpha$ 为最大化奖励和最大化熵之间的权重**, 用于权衡策略的探索和利用; $y_i$为目标值, 双 $Q$ 网络每次选择输出中较小的状态动作值作为目标 $Q$ 值用于更新 $Q$ 网络。策略网络的输入为状态信息, 输出为动作信息, 策略网络的训练损失如下所示:
$$
L_\pi(\theta)=\frac{1}{N} \sum_{i=1}^N\left(\alpha \log \pi_\theta\left(\tilde{a}_i \mid s_i\right)-\min _{j=1,2} Q_{\omega_j}\left(s_i, \tilde{a}_i\right)\right) 。
$$

由于策略是一个分布, 动作采样后无法求导, 因此采用重参数化的方式采样获取动作 $\tilde{a}$ 。

玻尔兹曼