基于组划分和组信息的多智能体一致性系统
## 简述

研究背景：在一致性问题中，每个智能体通常利用智能体间的相对状态去更新自身的状态，但是在某些场景中，由于隐私保护、系统测量误差，智能体间的相对状态通常无法获得，在此情况下需要提出一种新的策略实现一致性策略。

问题解决：本文通过组划分和组间信息替代智能体间相对状态来实现一致性算法。

整体过程：首先，在每一时刻通过一定的概率对所有智能体划分为2个或2个以上的子组，每个子组内的智能体在通过与之对应的子组信息更新自身的状态。


## Gossip 算法

Gossip 是一个最终一致性算法。虽然无法保证在某个时刻所有节点状态一致，但可以保证在”最终“所有节点一致"。

因为Gossip不要求节点知道所有其他节点，因此又具有去中心化的特点，节点之间完全对等，不需要任何的中心节点
>   Gossip is a final consistency algorithm. Although the state of all nodes cannot be guaranteed to be consistent at a certain time, it can be guaranteed that all nodes are consistent at "final".
> Gossip does not require nodes to know all other nodes, so it has the characteristics of decentralization. Nodes are completely equivalent, and no central node is required


https://blog.csdn.net/chen77716/article/details/6275762


## 演讲稿

P1
Hello everyone，My name is Mapeixin，I am from Zhejiang sci-tech university . My paper title is "Group Partition and Group Information based Multi-agent Consensus(kənˈsensəs) Systems".


P2
In recent years, multi-agent systems have captured compelling attention, and been widely used in the fields // such as military(mɪləteri_), smart grid,  machine learning and so on. As a fundamental concern, consensus problem aims to achieve global agreement // with only local interaction among agents.

Consensus is a fundamental issue in distributed multi-agent systems, and utilizes(ˈjuːtəlaɪz) the inter-agent relative state // to update the state of each agent, so as to achieve global synchronization(sɪŋkrənəˈzeɪʃ(ə)n).

For the sake of privacy(ˈpraɪvəsi) requirement, all the workers perform encryption operation. As a result, the inter-agent relative state makes no sense any more.

Motivated by this concern, this paper considers the // distributed multi-agent consensus problem  // in the absence of inter-agent relative states // in discrete-time system.

>*and provide only the gradient data to other workers or a common server.*  
>This paper proposes a  distributed multi-agent  consensus algorithm by group partition and state updating via relative group information replacing of the inter-agent relative state in discrete-time system.

P3:

this paper considers the distributed multi-agent consensus problem // in the absence of inter-agent relative states in discrete-time system，Consider n agents with a discrete-time model . as follows

in which x denote the state of agent i // and  u denote  control input . k represents discrete time.

Since there contains no interaction between any two agents, the inter-agent relative state $x_i − x_j$ is unavailable. Therefore, the n agents are divided into m(≤ n) subgroups, and utilize the relative group information // replacing(rɪˈpleɪsɪŋ) of inter-agent information  // as the control input to update the agent state.

For example，At each time, all the agents are divided into // two subgroups by certain probability（prɑːbəˈbɪləti）, and the agents belonging to the same subgroup // share a common control input. and thus each agent updates its own state // via the relative group information 

P4

When subgroup number is 2, the relative group information is $f_{G_1}(k) − f_{G_2}(k)$ (ˈmaɪnəs) . The control law（lɔː） is given as

 where $\gamma$  is a weighting parameter（pəˈræmɪtər）and $\gamma$ > 0 ;  $f_{G_1}(k)$ denote the convex(ˈkɑːnveks) combination of states // for all the agents in subgroup $G_I$ ,
 
 where a is nonnegative coefficient satisfying（ˈsætɪsfaɪɪŋ） $\sum a =1$ , 


P5
Using formula $f_{G_1}(k)$ and system , we could obtain the overall system as：；

the  control input in W (k) is ：

P6
Now we give our first main result for the scenario（səˈnærioʊ） of two subgroups：
When all the agents are divided into two subgroups, the system (1) under control law (4) achieves consensus in mean // if and only if γ > 1.

I will briefly introduce its proof，Consider the following positive definite（ˈdefɪnət） function：
using the system and control law， We can infer this 
Suppose γ > 1,  $V_x(k+1)< V_x(k)$
when  k gose to infinity(ɪnˈfɪnəti)
the system reaches consensus in mean.

P7:
It can be known from experiments(ɪkˈsperɪmənts;)
The system reaches consensus with γ  > 1，the system reaches consensus in means.

and  $\gamma$ < 1,the system reaches diverges(daɪˈvɜːrdʒ)

P8
 we apply the proposed algorithm in // a smart-car hardware platform // to achieve velocity (vəˈlɑːsəti) synchronization. 
 the initial states are randomly// set as shown in Figure. 5.  Then each vehicle(viːək(ə)) receives // and uses the information of its neighbors // from the computer to control its own movement ,After the consensus algorithm via group information, all the velocities(vəˈlɑsəti) are synchronized(ˈsɪŋkrənaɪzd), indicating(ˈɪndɪkeɪtɪŋ) that all the vehicles(ˈviːək(ə)lz) move towards the same direction with the same speed as Fig. 6.


```
For simplicity(sɪmˈplɪsəti) of experiment, we use six Corey Mecanum wheel vehicles(viːək(ə)lz) to validate the algorithm in two subgroups case.

we utilize an OptiTrack system and a computer to track and record the states of the vehicles including the location information and velocity information.

Then each vehicle receives and uses the information of its neighbors from the computer to control its own movement, so
as to apply the distributed consensus algorithm. The actual speed of each vehicle is about 0.15m/s （metres(ˈmiːtər) per second）, and the initial states are randomly set as shown in Fig. 5. After the consensus algorithm via group information, all the velocities are synchronized, indicating that all the vehicles move towards the same direction with the same speed as Fig. 6.
```
 

P9
and then I will discusses the scenario(səˈnærioʊ_)  // of multiple subgroups. Similarly(ˈsɪmələrli), each agent at time k is partitioned into one of m subgroups//  with certain probability. However, only two subgroups $G_I(k)$ and $G_J(k)$ are chosen by Gossip algorithm to update the agents’ states inside. Other subgroups will not be updated and ui (k) = 0. 

In this case, we also consider// the following positive definite function. and only $\gamma$ > 1 ,the system reaches consensus in mean.

```
When all the agents are divided into mul-tiple subgroups, and only two subgroups chosen by Gossip algorithm update their agents’ states by , the system (1) achieves consensus in mean if γ > 1, and the partition probability for each subgroup is equal.

```



P10:

In the following, we consider the case of multiple subgroup //number m = 5. Similarly(ˈsɪmələrli), the weighting parameter γ  > 1 , the system reaches consensus in mean.

P11
Thank u for your attention



thanks for your question


Thank u for your attention

英文字符：
“2-1”读作：two minus one
“1+1”读作：one plus one
“2>1”读作：Two is greater than one.
“1<2”读作 One is less than two.
1/5 one fifth
https://language.chinadaily.com.cn/2016-02/24/content_23607321.htm