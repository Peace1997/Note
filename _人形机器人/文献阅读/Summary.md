

## Locomotion 架构

从人形机器人 Locomotion 主要分为两块：Perspective Locomotion 和 Locomotion Manipulation. 


Locomotion 的框架主要包含：Two-stage 和 End-to-End
- Teacher

方法基本框架：
- GAIL
- Teacher-Student
	- 两阶段学习：教师策略利用特权信息然后指导学生策略学习潜在编码空间或动作
	- 端到端学习：
- Asymmetric Actor-critic Architecture


## 评价标准


**成功率**（Success Rate）：完成例如 AMASS 轨迹的成功率，反映策略的稳定性。
**关键点的平均跟踪误差**（Mean Per Keypoint Position Error）：反映世界坐标系下关键点的跟踪能力。
**速度距离**（Velocity Distance）：衡量动作的动态准确性，位置误差衡量跟踪的性能，速度精确度速度跟踪对于生成流畅、自然的动作比较关键。
**关节加速度** （Acceleration Distance）：衡量动作动态的平滑性和精细控制的能力。
