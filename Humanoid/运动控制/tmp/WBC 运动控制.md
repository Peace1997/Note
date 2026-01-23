
# WBC 起立

- 初始化状态估计
- 初始化接触力分布（平分重力）
- 起立阶段轨迹规划
	- 定义起始和目标状态
	- 固定脚部高度，防止起立过程中脚部打滑
	- 时间轨迹规划，以 0.11 m/s 的速度起来
	- 使用线性插值生成起立阶段的运动轨迹
	- WBC 优化求解过程
		- 约束条件构建
			- 浮动基体动力学约束
			- 关节力矩限制约束
			- 摩擦锥约束
		- 目标函数构建

## 优化问题构建
StandUpWbc 求解以下约束优化问题：

### 决策变量
决策变量：$\mathbf{x} = [\ddot{\mathbf{q}}, \mathbf{F}, \boldsymbol{\tau}]$
$\ddot{\mathbf{q}} \in \mathbb{R}^{18}$：基体 (6)+关节加速度 (12 个腿部关节) 
$\mathbf{F} \in \mathbb{R}^{12}$：4 个足端接触力（每个 3 维）
$\boldsymbol{\tau} \in \mathbb{R}^{12}$：关节力矩

### 约束条件

***1. 浮动基体动力学约束：*** 

$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{h}(\mathbf{q}, \dot{\mathbf{q}}) = \mathbf{J}_c^T \mathbf{F} + \mathbf{S}^T \boldsymbol{\tau}$$
其中：
$\mathbf{M}$：惯性矩阵
$\mathbf{h}$：科里奥利力+重力项
$\mathbf{J}$：接触雅可比矩阵
$\mathbf{S}$：关节选择矩阵


***2. 力矩限制约束***

$$-\boldsymbol{\tau}_{max} \leq \boldsymbol{\tau} \leq \boldsymbol{\tau}_{max}$$

转换为标准不等式约束：$\mathbf{D}\mathbf{x} \leq \mathbf{f}$ 
$$\begin{bmatrix} \mathbf{0} & \mathbf{0} & \mathbf{I} \\ \mathbf{0} & \mathbf{0} & -\mathbf{I} \end{bmatrix} \begin{bmatrix} \ddot{\mathbf{q}} \\ \mathbf{F} \\ \boldsymbol{\tau} \end{bmatrix} \leq \begin{bmatrix} \boldsymbol{\tau}_{max} \ \boldsymbol{\tau}_{max} \end{bmatrix}$$


***3.摩擦锥约束*** 

对于每个接触点$i$，摩擦锥约束为：
$$\begin{cases} F_{z,i} \geq 0 \\ |F_{x,i}| \leq \mu F_{z,i} \\ |F_{y,i}| \leq \mu F_{z,i} \end{cases}$$
金字塔近似： $$\mathbf{A}_{\text{friction}} \mathbf{F}_i \leq \mathbf{0}$$
其中： $$\mathbf{A}_{\text{friction}} = \begin{bmatrix} 0 & 0 & -1 \\ 1 & 0 & -\mu \\ -1 & 0 & -\mu \\ 0 & 1 & -\mu \\ 0 & -1 & -\mu \end{bmatrix}$$
库伦摩擦锥约束确保接触力在摩擦锥内：$$\sqrt{F_x^2 + F_y^2} \leq \mu F_z, \quad F_z \geq 0$$

### 目标函数

加权多任务目标： $$\min \sum_{i} w_i |\mathbf{A}_i\mathbf{x} - \mathbf{b}_i|^2$$
具体任务包括：

质心任务 ($w_{com}$)： 
$$\mathbf{A}_{com}\ddot{\mathbf{q}} = \ddot{\mathbf{r}}_{com,des} + \mathbf{Kp}(\mathbf{r}_{com,des} - \mathbf{r}_{com}) + \mathbf{Kd}(\dot{\mathbf{r}}_{com,des} - \dot{\mathbf{r}}_{com})$$

基体姿态任务 ($w_{base}$)： 

$$\mathbf{J}_{base}\ddot{\mathbf{q}} = \ddot{\mathbf{x}}_{base,des} + \mathbf{Kp}(\mathbf{x}_{base,des} - \mathbf{x}{base}) + \mathbf{Kd}(\dot{\mathbf{x}}_{base,des} - \dot{\mathbf{x}}_{base})$$


关节加速度任务 ($w_{joint}$)： 
$$\ddot{\mathbf{q}}_{joint} = \mathbf{Kp}(\mathbf{q}_{des} - \mathbf{q}) + \mathbf{Kd}(\dot{\mathbf{q}}_{des} - \dot{\mathbf{q}})$$

接触力跟踪 ($w_{force}$)： 

$$\mathbf{F} = \mathbf{F}_{des}$$
接触约束： $$\mathbf{J}_i\ddot{\mathbf{q}} + \dot{\mathbf{J}}_i\dot{\mathbf{q}} = \mathbf{0}$$ 
### QP 求解器
使用 qpOASES 求解器求解二次规划问题：

 
公式 6 - QP 标准形式： $$\begin{align} \min_{\mathbf{x}} \quad & \frac{1}{2}\mathbf{x}^T\mathbf{H}\mathbf{x} + \mathbf{g}^T\mathbf{x} \ \text{s.t.} \quad & \mathbf{A}\mathbf{x} = \mathbf{b} \ & \mathbf{D}\mathbf{x} \leq \mathbf{f} \end{align}$$

### 控制输出
优化求解得到：
关节力矩 $\boldsymbol{\tau}$
期望关节位置 $\mathbf{q}_{des}$
期望关节速度 $\dot{\mathbf{q}}_{des}$

## 目标函数

### 质心跟踪

- 坐标变换：世界坐标系到基体坐标系
- PD 控制生成期望加速度（任务层）
- 根据质心动力学方程生成目标

质心动力学方程： 质心的运动由质心动量变化率控制： $$m\ddot{\mathbf{r}}_{com} = \sum \mathbf{F}_{ext}$$
使用质心映射矩阵： $$\mathbf{A}_{com} = \mathbf{A}[0:3, :] \quad \text{(质心动量映射矩阵的线性部分)}$$
坐标变换 (世界坐标系到基体坐标系)
 $$\mathbf{x} = \begin{bmatrix} \mathbf{R}_{base}^T \mathbf{r}_{com} \\ \mathbf{R}_{base}^T \dot{\mathbf{r}}_{com} \end{bmatrix}$$
PD 控制律：
$$\ddot{\mathbf{r}}{com}^{des} = \mathbf{R}_{base} \mathbf{K}_{com} (\mathbf{x}_d - \mathbf{x})$$
其中 $\mathbf{K}_{com} \in \mathbb{R}^{3 \times 6}$ 是 PD 控制增益矩阵。

质心动量率项：
 $$\mathbf{A}_{com} \ddot{\mathbf{q}} = \dot{\mathbf{h}}_{lin}^{des} + m \ddot{\mathbf{r}}_{com}^{des} - \dot{\mathbf{A}}_{com} \dot{\mathbf{q}}$$

### 起立关节加速度任务
构建了选择矩阵：$\mathbf{A}_{joint} = \begin{bmatrix} \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} \end{bmatrix}$
从决策变量中提取关节加速度：$\ddot{\mathbf{q}}_{joint} = \mathbf{A}_{joint} \mathbf{x}$
期望关节加速度计算（PD 控制律）：
腿部关节： 
$$\ddot{\mathbf{q}}_{leg}^{des} = K_{p,leg} (\mathbf{q}_{leg}^{des} - \mathbf{q}_{leg}^{meas}) + K_{d,leg} (\dot{\mathbf{q}}_{leg}^{des} - \dot{\mathbf{q}}_{leg}^{meas})$$
手臂关节： 
$$\ddot{\mathbf{q}}_{arm}^{des} = K_{p,arm} (\mathbf{q}_{arm}^{des} - \mathbf{q}_{arm}^{meas}) + K_{d,arm} (\dot{\mathbf{q}}_{arm}^{des} - \dot{\mathbf{q}}_{arm}^{meas})$$
目标函数项：
$$J_{joint} = w_{joint} |\ddot{\mathbf{q}}_{joint} - \ddot{\mathbf{q}}_{joint}^{des}|^2$$

# WBC 太极

核心组件：
- 传感层：数据采集、滤波、状态估计
- 任务层：任务规划、动作序列生成
- 规划层：MPC 优化、策略生成
- 执行层：WBC 实时控制、关节命令生成



状态估计模块
将原始传感数据转换为控制系统可用的平滑状态信息

---
MPC：高级运动规划
1. 整体运动方向
2. 身体稳定性
3. 与环境交互
4. 大致身体配置

WBC：底层精确控制
1. 精确关节跟踪
2. 实时约束满足
3. 多任务协调
4. 安全保护


## MPC
MPC 模块在有限时域内求解最优控制问题，生成满足约束的参考轨迹和控制策略。在太极动作中，MPC 负责生成平滑、连续的全身运动轨迹，确保重心稳定性和动作流畅性。

### 1. 输入输出接口

MPC 输入：

|   输入向量    | 维度  |                                                              详细描述                                                               |
| :-------: | :-: | :-----------------------------------------------------------------------------------------------------------------------------: |
|    时间戳    |  1  |                                                                                                                                 |
| 质心动力学状态向量 | 24  | 质心线速度（3 dim -from 状态估计） <br>归一化角动量（3 dim - from 从 IMU 和关节状态计算）  <br>基体位置（3- 状态估计 ） <br>基体姿态（3 -- IMU 融合） <br>关节角度（12 -关节编码器+滤波） |
|  当前接触模式   |  1  |                                                            SS/SF/FS                                                             |


>[!NOTE]
>归一化角动量的计算：
>1. 计算各接触点对质心力矩的贡献
>2. 角动量率累加
>3. 归一化（除以机器人质量）

MPC 输出：

| 输出向量    | 维度     | 详细描述                                                     |
| ------- | ------ | -------------------------------------------------------- |
| 期望状态轨迹  | 24 x N |                                                          |
| 期望输入轨迹  | 36 x N | 脚部接触力（4 个接触点，4 x 3） </br> 手部六维力（2 x 6）</br> 关节速度（12 dim） |
| 规划的接触模式 | 1      |                                                          |

MPC 采用**滚动优化**的方式，内部维护完整 1s 的轨迹，但是只返回当前时刻的最优值，下个控制周期会重新优化整条轨迹。这样的好处是每次都能基于最新状态重新规划。



###  2. 质心动力学状态空间模型
机器人采用 [[Humanoid/运动控制/tmp/MPC#2. 质心动力学连续模型|质心动力学模型]]  (Centroidal Dynamics)进行 MPC 轨迹规划。该模型将复杂的多体系统简化为**质心运动**和**角动量守恒**的组合，大幅降低了计算复杂度。

质心动力学的核心方程： 
**动量方程：**$$m\ddot{\mathbf{r}}_{CoM} = \sum \mathbf{f}_{ext} + m\mathbf{g}$$
**角动量方程：**
$$\dot{\mathbf{L}} = \sum (\mathbf{r}_i - \mathbf{r}_{CoM}) \times \mathbf{f}_{ext}$$

质心动力学优势：
- 避免机器人的完整的状态配置
- 避免复杂的惯性装量计算
- 提供了系统的整体描述

>[! Tip] 状态向量与输入向量
>状态： **状态是优化的目标（跟踪期望轨迹）** ，观测到的系统的状态，需要从传感器观测
 输入：**输入是优化的手段（如何达到目标）**，施加给系统的控制，是 MPC 要寻找的最优解，即控制器的**决策变量**。
> 状态导数： 输入如何改变状态


MPC 的目标是基于当前状态，找到最优的未来输入轨迹。

#### 状态向量
$$\mathbf{x} = \begin{bmatrix} 
\mathbf{v}_{\text{CoM}} \\ 
\mathbf{L}/m \\ 
\mathbf{p}_{\text{base}} \\ 
\boldsymbol{\theta}_{\text{base}} \\ 
\mathbf{q}_{\text{joints}} 
\end{bmatrix} \in \mathbb{R}^{24}$$

其中：
- $\mathbf{v}_{\text{CoM}} \in \mathbb{R}^3$: 质心线速度 (归一化的线动量 $\mathbf{v}_{\text{CoM}} =\mathbf{h}_{\text{lin}}/m$) 
- $\mathbf{L}/m \in \mathbb{R}^3$: 归一化角动量  
- $\mathbf{p}_{\text{base}} \in \mathbb{R}^3$: 基体位置
- $\boldsymbol{\theta}_{\text{base}} \in \mathbb{R}^3$: 基体姿态 (ZYX 欧拉角)
- $\mathbf{q}_{\text{joints}} \in \mathbb{R}^{12}$: 关节角度 (腿部 12 DOF) 

>[! NOTE]
>在传统的状态向量中通常包括：$$\mathbf{x} = \begin{bmatrix} \mathbf{p}_{base} \\ \mathbf{\boldsymbol\theta}_{base} \\ \mathbf{q}_{joints} \\ \dot{\mathbf{p}}_{base} \\ \boldsymbol{\omega}_{base} \\ \dot{\mathbf{q}}_{joints} \end{bmatrix} \in \mathbb{R}^{n_x}$$
>1. 为什么不包含关节速度
>质心动力学将复杂的多体系统简化为质心运动+角动量守恒，因此关节速度信息被隐含编码在质心动量状态中。
>
>2. 传统角动量与角动量归一化
>传统角动量：
>$$\mathbf{L} = \mathbf{I}(\mathbf{q}) \boldsymbol{\omega}$$
>
>- $\mathbf{L} \in \mathbb{R}^3$: 角动量向量 (单位: $kg \cdot m^2/s$)
>- $\mathbf{I}(\mathbf{q}) \in \mathbb{R}^{3 \times 3}$: 惯性张量矩阵 (配置相关)
>- $\boldsymbol{\omega} \in \mathbb{R}^3$: 角速度向量 (单位: $rad/s$)
>归一化形式: 
>$$\mathbf{L}_{norm} = \frac{\mathbf{L}}{m} = \frac{\mathbf{I}(\mathbf{q}) \boldsymbol{\omega}}{m}$$
>- $\mathbf{L}_{norm} \in \mathbb{R}^3$: 归一化角动量 (单位: $m^2/s$)
>-  $m$: 机器人总质量 (单位: $kg$)
>**从量纲分析：**
>$$[\mathbf{L}_{norm}] = \frac{[kg \cdot m^2/s]}{[kg]} = [m^2/s]$$
>- 归一化角动量具有 $[长度^2 \times 角速度]$ 的量纲，避免不同质量造成的影响
>- 可以理解为等效回转半径的平方乘以等效角速度 $\mathbf{L}_{norm} = r_{gyration}^2 \cdot \boldsymbol{\omega}_{equivalent}$
>**从物理一致性角度分析：**
牛顿第二定律的直接应用: $$\frac{d}{dt}\left(\frac{\mathbf{h}_{lin}}{m}\right) = \frac{1}{m}\frac{d\mathbf{h}_{lin}}{dt} = \frac{1}{m}\sum_{i} \mathbf{f}_i^{ext}
$$ 欧拉动量方程的直接应用: $$\frac{d}{dt}\left(\frac{\mathbf{L}}{m}\right) = \frac{1}{m}\frac{d\mathbf{L}}{dt} = \frac{1}{m}\sum_{i} \boldsymbol{\tau}_i^{ext}$$
>3. 为什么使用归一化角动量而不用角速度
>同理，使用角速度的非线性动力学表示 $$\mathbf{I}(\mathbf{q})\dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times \mathbf{I}(\mathbf{q})\boldsymbol{\omega} = \boldsymbol{\tau}_{ext}$$ 
>这是一个**非线形微分方程**，它包含配置相关的惯性矩阵$\mathbf{I}(\mathbf{q})$和非线性的陀螺力矩项$\boldsymbol{\omega} \times \mathbf{I}(\mathbf{q})\boldsymbol{\omega}$。
>使用归一化角动量的线性动力学: $$\frac{d}{dt}\left(\frac{\mathbf{L}}{m}\right) = \frac{1}{m}\boldsymbol{\tau}_{ext}$$


#### 输入向量
$$\mathbf{u} = \begin{bmatrix} 
\mathbf{f}_1 \\ \mathbf{f}_2 \\ \mathbf{f}_3 \\ \mathbf{f}_4 \\ 
\mathbf{F}_{\text{hand1}} \\ \mathbf{F}_{\text{hand2}} \\ 
\dot{\mathbf{q}}_{\text{joints}} 
\end{bmatrix} \in \mathbb{R}^{36}$$

其中：
- $\mathbf{f}_i \in \mathbb{R}^3$: 第$i$个足端接触力 （4 个足端 (脚跟+脚尖)）- 12dim
- $\mathbf{F}_{\text{handi}} \in \mathbb{R}^6$: 第$i$个手部 6 DOF 力/力矩 (2 个手部)  - 12 dim
- $\dot{\mathbf{q}}_{\text{joints}} \in \mathbb{R}^{12}$: 关节速度 -12dim


>[! NOTE]
>1. 为什么使用关节速度不使用关节力矩
>
>在传统的人形机器人 MPC 中，输入向量通常是：
>$$
>\mathbf{u} = \begin{bmatrix} \mathbf{f}_{contacts} \\ \boldsymbol{\tau}_{joints} \end{bmatrix} \in \mathbb{R}^{n_u}
>$$
>而现在我们通过质心动力学模型实现，状态空间只关注质心运动，不直接包含关节动力学，内力（关节力矩）不出现在质心动力学方程中，这是因为根据牛顿第三定律，内力总是成对出现且相互抵消：
>$$\sum \mathbf{f}_{internal} = 0$$ 
>$$\sum \boldsymbol{\tau}_{internal} = 0$$
>关节力矩的作用被"隐藏"了，关节力矩确实影响机器人运动，但它们的作用是
>-  改变身体配置（关节角度）
>- 间接影响接触力分布. 
>- 不直接改变质心动力学
> 质心动力学的本质局限，质心动力学只能描述外力影响，关节力矩的作用被"隐藏"了
> 
> 2. **那为什么选择关节速度呢？**
>（1）状态空间方程的基本要求
对于任何动力学系统，状态空间方程必须满足： $$\dot{\mathbf{x}} = \mathbf{f}(\mathbf{x}, \mathbf{u}, t)$$
> 关键点： **状态的导数必须能够直接从当前状态和输入计算得出。**
（2）质心动力学的状态分解
  >在质心动力学模型中，状态向量被分解为两部分：
  >$$\mathbf{x} = [质心动量状态，配置状态] = [(\mathbf{v_{com}},\mathbf{L}/m),(\mathbf{p}_{base},\mathbf{\theta}_{base},\mathbf{q}_{joints})]$$
   对应的状态导数：
> $$\dot{\mathbf{x}} = [质心动量导数，配置导数] = [(\dot{\mathbf{v}}_{com},\dot{\mathbf{L}}/m),(\dot{\mathbf{p}}_{base},\dot{\mathbf{\theta}}_{base},\dot{\mathbf{q}}_{joints})]$$
>如下表所示，由于基体的角速度和关节速度不在状态里，则输入变量必须要包含关节角速度，而基体的角速度可以从归一化的角动量计算：
$$\mathbf{L} = \mathbf{I}_{composite}(\mathbf{q}) \boldsymbol{\omega}_{composite}$$ 
>其中 $\boldsymbol{\omega}_{composite}$ 包含基体角速度和关节对质心角动量的贡献。


| 状态导数                                      | 计算方式                                                 | 需要的信息  | 来源                              |
| ----------------------------------------- | ---------------------------------------------------- | ------ | ------------------------------- |
| $\dot{\mathbf{v}}_{\text{CoM}}$           | $\frac{1}{m}\sum \mathbf{f}_i + \mathbf{g}$          | 接触力    | 输入$\mathbf{u}$                  |
| $\dot{\mathbf{L}}/m$                      | $\frac{1}{m}\sum (\mathbf{r}_i \times \mathbf{f}_i)$ | 接触力+位置 | 输入 $\mathbf{u}$+状态 $\mathbf{x}$ |
| $\dot{\mathbf{p}}_{\text{base}}$          | $= \mathbf{v}_{\text{CoM}}$                          | 质心速度   | 状态$\mathbf{x}$                  |
| $\dot{\boldsymbol{\theta}}_{\text{base}}$ | $\mathbf{T}(\boldsymbol{\theta})\boldsymbol{\omega}$ | 基体角速度  | 需要来源！                           |
| $\dot{\mathbf{q}}_{\text{joints}}$        | $= ?$                                                | 关节速度   | 需要来源！                           |

基体姿态的时间导数： $$\dot{\boldsymbol{\theta}}_{\text{base}} = \mathbf{T}^{-1}(\boldsymbol{\theta}_{\text{base}}) \boldsymbol{\omega}_{\text{base}}$$ 其中 $\mathbf{T}$ 是从角速度到欧拉角速度的转换矩阵。
从归一化角动量推导基体角速度： 在质心动力学模型中，总角动量可以分解为： $$\mathbf{L} = \mathbf{L}_{\text{base}} + \mathbf{L}_{\text{joints}}$$

#### 质心动力学微分方程

**质心动力学微分方程 (牛顿-欧拉方程的简化形式):**

线动量守恒：
$$m\dot{\mathbf{v}}_{CoM} = \sum_{i=1}^{4} \mathbf{f}_i + \sum_{j=1}^{2} \mathbf{F}_{hand,j}^{lin} + m\mathbf{g}$$

角动量守恒：
$$\dot{\mathbf{L}} = \sum_{i=1}^{4} (\mathbf{r}_i - \mathbf{r}_{CoM}) \times \mathbf{f}_i + \sum_{j=1}^{2} [(\mathbf{r}_{hand,j} - \mathbf{r}_{CoM}) \times \mathbf{F}_{hand,j}^{lin} + \mathbf{F}_{hand,j}^{ang}]$$

基体运动学：
$$\dot{\mathbf{p}}_{\text{base}} = \mathbf{v}_{\text{CoM}}$$
$$\dot{\boldsymbol{\theta}}_{\text{base}} = \mathbf{R}_{\text{ZYX}}(\boldsymbol{\theta}_{\text{base}}) \boldsymbol{\omega}_{\text{base}}$$

关节运动学：$$\dot{\mathbf{q}}_{\text{joints}} = \dot{\mathbf{q}}_{\text{joints}}$$ (直接在输入向量中给定)

**归一化状态空间方程:**
$$\dot{\mathbf{x}} = \begin{bmatrix}
\frac{1}{m}\sum_{i=1}^{4} \mathbf{f}_i + \frac{1}{m}\sum_{j=1}^{2} \mathbf{F}_{\text{hand},j}^{\text{lin}} + \mathbf{g} \\
\frac{1}{m}\sum_{i=1}^{4} (\mathbf{r}_i - \mathbf{r}_{\text{CoM}}) \times \mathbf{f}_i + \frac{1}{m}\sum_{j=1}^{2} [(\mathbf{r}_{\text{hand},j} - \mathbf{r}_{\text{CoM}}) \times \mathbf{F}_{\text{hand},j}^{\text{lin}} + \mathbf{F}_{\text{hand},j}^{\text{ang}}] \\
\mathbf{v}_{\text{CoM}} \\
\mathbf{R}_{\text{ZYX}}(\boldsymbol{\theta}_{\text{base}}) \boldsymbol{\omega}_{\text{base}} \\
\dot{\mathbf{q}}_{\text{joints}}
\end{bmatrix}$$



### 3. 连续时间最优控制问题

**数学公式 (Bolza 型最优控制问题):**
$$\begin{align}
\min_{\mathbf{u}(\cdot)} \quad &\phi(\mathbf{x}(T)) + \int_0^T L(\mathbf{x}(t), \mathbf{u}(t), t) \, dt \\
\text{s.t.} \quad &\dot{\mathbf{x}}(t) = \mathbf{f}(\mathbf{x}(t), \mathbf{u}(t), t) \\
&\mathbf{g}(\mathbf{x}(t), \mathbf{u}(t), t) \leq \mathbf{0} \\
&\mathbf{h}(\mathbf{x}(t), \mathbf{u}(t), t) = \mathbf{0} \\
&\mathbf{x}(0) = \mathbf{x}_0
\end{align}$$

其中：
- $\phi(\mathbf{x}(T))$: 终端成本函数
- $T$：预测时域
- $L(\mathbf{x}, \mathbf{u}, t)$: 瞬时成本函数 (Lagrangian)
- $\mathbf{g}(\cdot) \leq 0$: 不等式约束 (摩擦锥、关节限制等)
- $\mathbf{h}(\cdot) = 0$: 等式约束 (运动学约束等)
- $x(t) \in \mathbb{R}^{24}$：状态向量（质心动力学）
- $u(t) \in \mathbb{R}^{36}$：输入向量（接触力+关节速度）
- $Q$：状态全重


**瞬时成本函数 (二次型):**
$$L(\mathbf{x}, \mathbf{u}, t) = \frac{1}{2}(\mathbf{x} - \mathbf{x}_{ref}(t))^T \mathbf{Q}(t) (\mathbf{x} - \mathbf{x}_{ref}(t)) + \frac{1}{2}(\mathbf{u} - \mathbf{u}_{ref}(t))^T \mathbf{R}(t) (\mathbf{u} - \mathbf{u}_{ref}(t))$$

**终端成本函数:**
$$\phi(\mathbf{x}(T)) = \frac{1}{2}(\mathbf{x}(T) - \mathbf{x}_{ref}(T))^T \mathbf{Q}_f (\mathbf{x}(T) - \mathbf{x}_{ref}(T))$$

###  4. 离散化与 DDP 求解

**时间离散化 (Euler 前向法):**
$$\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \cdot \mathbf{f}(\mathbf{x}_k, \mathbf{u}_k, t_k)$$

其中 $\Delta t = 0.015s$ (66.7 Hz)，预测步数 $N = T_{horizon}/\Delta t = 1.0/0.015 ≈ 67$

**离散时间最优控制问题:**
$$\begin{align}
\min_{\{\mathbf{u}_k\}_{k=0}^{N-1}} \quad &\phi_N(\mathbf{x}_N) + \sum_{k=0}^{N-1} L_k(\mathbf{x}_k, \mathbf{u}_k) \Delta t \\
\text{s.t.} \quad &\mathbf{x}_{k+1} = \mathbf{f}_k(\mathbf{x}_k, \mathbf{u}_k) \\
&\mathbf{g}_k(\mathbf{x}_k, \mathbf{u}_k) \leq \mathbf{0} \\
&\mathbf{h}_k(\mathbf{x}_k, \mathbf{u}_k) = \mathbf{0} \\
&\mathbf{x}_0 = \mathbf{x}_{measured}
\end{align}$$

**Gauss-Newton DDP 算法核心思想:**

DDP (Differential Dynamic Programming)是求解非线性最优控制的经典方法，通过反向递推构造二次逼近。

1. **前向传播 (Forward Pass):** 使用当前控制策略积分系统动力学
2. **后向传播 (Backward Pass):** 反向求解 Riccati 方程获得最优反馈增益
3. **线搜索 (Line Search):** 确定步长保证成本函数下降

**Hamilton-Jacobi-Bellman 方程的离散形式:**
$$V_k(\mathbf{x}_k) = \min_{\mathbf{u}_k} \left[ L_k(\mathbf{x}_k, \mathbf{u}_k) \Delta t + V_{k+1}(\mathbf{f}_k(\mathbf{x}_k, \mathbf{u}_k)) \right]$$

**最优性条件 (一阶必要条件):**
$$\nabla_{\mathbf{u}} H_k = \nabla_{\mathbf{u}} L_k + \nabla_{\mathbf{u}} \mathbf{f}_k^T \nabla_{\mathbf{x}} V_{k+1} = 0$$



## WBC

### 1. 输入输出接口

WBC 输入：

| 输入变量     | 维度    | 描述                                                                                                               |
| -------- | ----- | ---------------------------------------------------------------------------------------------------------------- |
| 期望状态     | 24+12 | 除 MPC 输出的期望状态（24）外，额外增加了期望的手臂关节角度。                                                                               |
| 期望输入     | 36+12 | 4 个脚接触点的力（MPC 直接输出）</br> 2 手的六维力（MPC 输出；坐标系转换）</br>腿部关节速度（MPC 输出+滤波）</br>手臂关节速度（轨迹规划+滤波）                         |
| 实测刚体状态   | 48    | 基体姿态（3 dim）</br> 基体位置（3 dim）</br> 完整关节角度（12+14 dim）</br> 基体角速度（3 dim ）</br> 基体线速度（3 dim）</br> 完整关节角速度（12+14 dim） |
| 接触模式     | 1     |                                                                                                                  |
| 时间步长     | 1     |                                                                                                                  |
| MPC 更新标志 | 1     |                                                                                                                  |

WBC 输出：

| 输入变量  | 维度  | 描述  |
| ----- | --- | --- |
| 关节力矩  | 24  |     |
| 关节加速度 | 24  |     |
| 基体加速度 | 6   |     |
| 接触力   | 24  |     |







3. 为什么选择关节速度？


    

关节运动学方程： $$\dot{\mathbf{q}}_{joints} = \mathbf{u}_{joints}$$

这里 $\mathbf{u}_{joints}$ 必须是速度量纲，不能是力矩！




**质心动力学微分方程 (牛顿-欧拉方程的简化形式):**

线动量守恒：
$$m\dot{\mathbf{v}}_{CoM} = \sum_{i=1}^{4} \mathbf{f}_i + \sum_{j=1}^{2} \mathbf{F}_{hand,j}^{lin} + m\mathbf{g}$$

角动量守恒：
$$\dot{\mathbf{L}} = \sum_{i=1}^{4} (\mathbf{r}_i - \mathbf{r}_{CoM}) \times \mathbf{f}_i + \sum_{j=1}^{2} [(\mathbf{r}_{hand,j} - \mathbf{r}_{CoM}) \times \mathbf{F}_{hand,j}^{lin} + \mathbf{F}_{hand,j}^{ang}]$$

基体运动学：
$$\dot{\mathbf{p}}_{\text{base}} = \mathbf{v}_{\text{CoM}}$$
$$\dot{\boldsymbol{\theta}}_{\text{base}} = \mathbf{R}_{\text{ZYX}}(\boldsymbol{\theta}_{\text{base}}) \boldsymbol{\omega}_{\text{base}}$$

关节运动学：

$$\dot{\mathbf{q}}_{\text{joints}} = \dot{\mathbf{q}}_{\text{joints}}$$ (直接给定)

**归一化状态空间方程:**
$$\dot{\mathbf{x}} = \begin{bmatrix}
\frac{1}{m}\sum_{i=1}^{4} \mathbf{f}_i + \frac{1}{m}\sum_{j=1}^{2} \mathbf{F}_{\text{hand},j}^{\text{lin}} + \mathbf{g} \\
\frac{1}{m}\sum_{i=1}^{4} (\mathbf{r}_i - \mathbf{r}_{\text{CoM}}) \times \mathbf{f}_i + \frac{1}{m}\sum_{j=1}^{2} [(\mathbf{r}_{\text{hand},j} - \mathbf{r}_{\text{CoM}}) \times \mathbf{F}_{\text{hand},j}^{\text{lin}} + \mathbf{F}_{\text{hand},j}^{\text{ang}}] \\
\mathbf{v}_{\text{CoM}} \\
\mathbf{R}_{\text{ZYX}}(\boldsymbol{\theta}_{\text{base}}) \boldsymbol{\omega}_{\text{base}} \\
\dot{\mathbf{q}}_{\text{joints}}
\end{bmatrix}$$
