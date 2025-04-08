
## Trinity

###  Introduction
Trinity 系统采用模块化层次结构设计，将人形机器人控制的复杂问题分解为不同层次，主要由三个核心技术模块组成：

1. **强化学习 (Reinforcement Learning, RL)模块**：负责低层次运动控制
2. **视觉语言模型 (Visual Language Model, VLM)模块**：负责环境感知和场景理解
3. **大型语言模型 (Large Language Model, LLM)模块**：负责任务理解和规划

![[Pasted image 20250322113004.png]]

这种模块化设计允许各组件独立优化，同时协同工作，提高系统整体性能和适应性。

```
用户指令（Human Prompt） +  VLM(场景理解) → LLM(任务理解) → 任务规划（Task Planner） → RL Policy 、 Arm Planner 、 Hand Controller → 机器人执行
```

通过这种集成方式，Trinity能够：
- 将高层次的自然语言指令转化为具体的运动计划
- 实时感知环境变化并调整运动策略
- 在复杂环境中执行多样化任务

### RL
采用了对抗性运动先验(Adversarial Motion Priors, AMP)技术，使机器人执行更自然、人类化的动作。

### VLM
采用 **ManipVQA** 作为 VLM 框架，结合了视觉编码器和语言编码器，实现视觉问答和环境理解：
- **输入**：相机捕获的图像/视频流（I）、用户语言查询（Q）
- **输出**：场景理解结果、物体属性描述、空间关系判断
$$
P(R|I,Q) = f_{VLM}(I,Q)
$$

1. **视觉编码器**：
    - 处理RGB图像和深度信息
    - 提取物体特征、空间关系和场景结构
2. **语言编码器**：
    - 处理文本查询和指令
    - 生成语义嵌入表示
3. **多模态融合结构**：
    - 交叉注意力机制融合视觉和语言特征
    - 生成综合环境认知表示

### LLM
LLM 模块作为 Trinity 系统的中央规划组件，负责将 VLM 的感知结果与用户指令相结合，生成完整的任务执行计划：
- **输入**：用户自然语言指令 (Human Prompt)、VLM 的环境感知结果（P）
- **输出**：具体的技能执行序列、高层任务规划 （T）



LLM 规划模块将高层任务指令翻译成具体的执行步骤：
1. **指令理解**：分析自然语言指令，识别核心目标和约束
2. **感知整合**：结合VLM提供的环境信息进行规划
3. **技能选择**：从技能库中选择合适的基本动作
4. **执行监控**：监控执行过程，必要时进行重规划

**【Human Prompt】**
- Task description
- Skill library
- Workspace limitations
- Safety constraints
- Prior kinematic knowledge (ref: [Kinematic-aware prompting for generalizable articulated object manipulation with llms](https://arxiv.org/abs/2311.02847))

（Kinematic-aware Prompting）：将关节物体的运动学先验信息（.URDF）融入到语言模型（LLM）的提示中，使模型在生成的动作符合机器人的运动学逻辑。



*技能库设计*：
- 手臂技能：抓取、放置、推拉等
- 手部技能：精细抓取、按压、旋转等
- 身体技能：行走、蹲下、转身等
*规划算法*：
- 基于技能库进行层次化任务分解
- 考虑工作空间约束和安全准则
