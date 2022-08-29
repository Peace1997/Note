


Critic 网络：
拟合状态值函数，
> 

**为什么需要 Critic 网络而不是采用奖励？**
Critic 网络用于判断当前状态在未来中可能获得的价值（累积奖励），奖励通常是衡量当前状态下的及时奖励，无法判断当前状态在未来中是否是好的，因此通过 Critic 网络评判状态的好坏会更加合理。

优势函数是正的就增大概率，如果是负的就减小概率。


Policy：
$$
L\left(s, a, \theta_{k}, \theta\right)=\min \left(\frac{\pi_{\theta}(a \mid s)}{\pi_{\theta_{k}}(a \mid s)} A^{\pi_{\theta_{k}}}(s, a), \quad \operatorname{clip}\left(\frac{\pi_{\theta}(a \mid s)}{\pi_{\theta_{k}}(a \mid s)}, 1-\epsilon, 1+\epsilon\right) A^{\pi_{\theta_{k}}}(s, a)\right)
$$
Value ：
