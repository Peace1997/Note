

- Amazon FAR + Berkeley/Stanford 这一大支：OmniRetarget / VisualMimic / ResMimic / HumanoidExo 一整套。
- Berkeley Hybrid Robotics + C. Karen Liu 线：BeyondMimic + General Motion Tracking + TWIST / TWIST2 等。
- NVIDIA / LeCAR / VIRAL 线：VIRAL + SoFTA / FALCON 这一批更偏「大规模视觉 sim2real」。
- 上海 AI Lab + CUHK 线：HOMIE + HoST + GRUtopia，把低成本 teleop 硬件和城市场景仿真绑在一起。
- 北大 Zongqing Lu + BeingBeyond：Being-0 + DemoHLM，已经开始认真搞「一次示教，机器人就能干活」。
- BIGAI / 智源这一支：CLONE + FSR-VLN，从闭环遥操作到语言驱动导航。


罗正宜：本科宾夕法尼亚、研究生 CMU -- Jason Peng

CMU ： 
- 罗正宜、何泰然：PHC、H2O、OmniH2O、ASAP、HOVER

 H2O 为 PHC 的衍生版本，主要为了实现 PHC2Real
关于 OmniH2O 和 H2O，它们之间的区别主要体现在输入形式与使用场景上。OmniH2O 项目的出发点是为了更好地实现遥操作。OmniH2O 还加入了一个蒸馏过程。


- https://mp.weixin.qq.com/s?__biz=MzkwNDMxMzg1NA==&mid=2247485653&idx=1&sn=d5c01fc21ce6f00f0898f9c118fdf9c6&scene=21&poc_token=HIB5XGmjwnuHBggNpm_DlepCqcxkKvrgZELxrICT


难点：机器人控制迄今为止仍未彻底解决的一个问题，就是如何在已有技能的基础上进行技能复用（skill reuse）。
# 技术路线总结

PHC：
PULSE：将技能嵌入隐空间，建立高效可用技能库
OmniGrasp
PDC：从基本动作控制转向构建行为基础模型，扩展到操作任务


## 重定向算法

### PHC
罗正宜（Zhengyi Luo）
> 博士毕业于 CMU，他的导师是 Kris Kitani 教授


>[! 策略衡量指标] 
> -  success rate（成功率）：指模型是否能从头到尾复现整个动作序列，如果中途失败，例如姿态偏差过大或跌倒，则计为失败。
> -  MPJPE（Mean Per Joint Position Error）：即每个关节点与参考动作之间的平均位置误差，用于衡量模仿精度
> - acceleration error：用于评估动作是否过于抖动或不稳定。

### GMR 

迮炎杰
> 坦福大学计算机科学系攻读博士，由 Jiajun Wu 和C. Karen Liu 指导。

在满足机器人运动学与实时性约束的前提下，通过分层、多目标 IK，最小化人体核心关键点（位姿）与机器人核心关键点的误差

**（1）数据格式标准化**
将不同的人体数据（SMPL-X、BVH、FBX、GVHMR 等格式）统一转换为标准世界坐标系下的全局位置和旋转表示（全局位姿），便于后续的映射和坐标系变换。

**（2）人体与机器人映射**
- 对人类骨骼数据进行尺度调整，减少不同身高/尺寸带来的足部穿透、关节冲突
- 将人体坐标系映射到机器人坐标系（包括位置偏移与旋转偏置调整）

**（3）多目标分层 IK 与迭代优化**

- 问题建模：将机器人的末端位置与映射后的目标位置尽可能接近（目标函数），同时避免不合理的姿态与物理不行的行为（约束）。
- 按任务重要性加权，逐层优化
	- 全局躯干不漂移不穿地
	- 主肢体步态合理，身体方向对齐
	- 在不破坏下肢和平衡的前提下拟合人体动作



GMR 与 PHC 更多的是在绝对坐标系下做一个点到点的简单映射

OmniRetarget


# model-based & learning-based 

model-based 优势：依赖明确的模型和数学原理，从原理层出发，可以更有效和更高质量的解决一些学习类方法难以处理的问题。但需要良好的状态估计与精准的建模（流体、可变形物体很难以数学的方式写出）

learning-based 方法：泛化能力更强，不依赖状态估计，但依赖数据的质量，定位问题更复杂


一种结合两种方法的思路是： 使用 model-based 的方法生成数据，在用 learning-based 的方法，对这些高质量数据进行学习。

