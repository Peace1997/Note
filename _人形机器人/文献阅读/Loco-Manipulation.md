
# ResMimic：两阶段残差框架学习全身操作
[ResMimic: From General Motion Tracking to Humanoid Whole-body Loco-Manipulation via Residual Learning](https://www.arxiv.org/pdf/2510.05070) 
> Amazon FAR (Frontier AI & Robotics)  & USC  & Stanford University --2025.10

本文提出了一个名为 ResMimic 的两阶段残差学习框架，旨在提高机器人在面对全身操作类运动的精确控制。ResMimic 首先利用一个通用动作跟踪策略（GMT）生成类人动作，然后学习一个残差策略来精炼这些输出，以实现精确的运动和对象交互。

![[Pasted image 20251017093227.png]]
***contribution***
- 两阶段残差学习框架：
	- 第一阶段：通用运动跟踪策略 GMT，选用大规模人类动作数据集（AMASS & OMOMO）训练任务无关的基础运动策略。
	- 第二阶段：残差策略精炼，加入对象交互，提升 GMT 策略定位和操作的精度
- 高效训练的特定设计
	- **基于点云的对象跟踪奖励**：利用点云数据来判断当前位置和参考位置的跟踪差值，这样可以使奖励更平滑的优化。
	- **接触奖励**：鼓励人形机器人身体-物体更准确的交互（正确的部位、力度和时机）
	- **基于课程的虚拟力辅助控制：** 为弥补训练初期策略的不稳定性，引入了一个基于课程学习的虚拟力辅助控制机制，对需要交互的物体施加一个期望的力/力矩的方式，降低前期任务的难度，尤其是在重物和噪声较大的环境中。


