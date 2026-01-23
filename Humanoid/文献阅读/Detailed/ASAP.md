
## Overview
### Framework
为解决传统方法如系统辨识和域随机化存在参数调优复杂或策略过于保守的问题，该论文提出了名为 ASAP（**A**ligning **S**imulation and Re**a**l **P**hysics）的框架，该框架包含两个训练阶段：第一阶段为预训练阶段，通过人类视频数据重定向训练运动跟踪策略，将预训练策略部署到真机，记录真实世界轨迹。在第二阶段，训练一个**动作补偿模型**（Delta Action Model），补偿仿真和真机的动力学差异，最后在集成动作补偿的仿真中微调预训练策略，最终直接部署到真机中。
![[Pasted image 20250427112907.png]]

### Contribution

- Delta Action： 提出通过补偿动作去学习动力学不匹配的方法，避免了传统 SysID 对参数空间的依赖和 DR 的保守性，降低 Sim2Real Gap。
- 三阶段训练架构：从仿真到真机，再从真机到仿真，最后仿真到真机的训练架构，通过多次仿真和真机的切换，尽可能提升基准策略对真机的适配性。


## Pre-Training

***Pre-Training：Learning Agile Humanoid Skills***

### Data Generation
**A. Data Generation: Retargeting Human Video Data** -- 数据生成

1.  **Transforming Human Video to SMPL Motions**：通过 **TRAM**（Trajectory-Aware Motion Represent） 技术对从视频序列中中估计人体的**全局运动轨迹**（全局坐标系）和**基于 SMPL 模型的 3 D 姿态和形状**（局部坐标系）。其中 SMPL 是一个参数化的可微分的三维人体模型。
2.  **Simulation-based Data Cleaning:** 通过 MaskedMimic 方法进行动作跟踪模仿，在仿真中对不能被模仿的动作进行清洗。
3.  **Retargeting SMPL Motions to Robot Motions**: 将 SMPL 动作**重定向**到真机机器人上。

###  Policy Training
**B. Phase-based Motion Tracking Policy Training** -- 策略训练

-  **Asymmetric Actor-Critic Training**：采用非对称 AC 架构，Critic 利用特权信息（全局轨迹），Actor 仅依赖本体感知。
- **Termination Curriculum of Tracking Tolerance**： 采用课程学习的方法对回合结束的条件（动作误差容忍度）进行动态调整（1.5 m-0.3 m）
- **Reference State Initialization:** 采用 RSI（Reference State Initialization）架构对随机出实话动作相位，避免出现顺序学习的瓶颈。

## Post-Training


***Post-Training: Training Delta Action Model and Fine-Tuning Motion Tracking Policy***

- **Data Collection**： 利用第一阶段训练好的模型，在真机中进行动作的采集。
- **Training Delta Action Model**：基于仿真和真机的数据，通过 RL 方法学习一个补偿动作网络 Delta Action，补偿动力学差距。
- **Fine-tuning Motion Tracking Policy under New Dynamics**：对第一阶段的策略模型加入该动作补偿网络重新训练（策略微调），即所有新动作都加上 delta action，在真机部署时不需要部署动作补偿网络。

### Data Collection


### Delta Action


Delta Action 模型的目标是补偿**仿真与真实世界的动力学差异**，主要包括：
- **硬件特性差异**：如电机响应延迟、关节摩擦、连杆质量分布等。
- **固定环境参数偏差**：如平坦地面的摩擦系数、默认重力参数等[表VI]。
- **传感器噪声**：IMU、编码器等传感器的系统性误差[1]。
