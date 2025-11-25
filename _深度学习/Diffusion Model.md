
# 简述

***1. 什么是扩散模型***

扩散模型（Diffusion Model）是一类生成模型，适用于生成高质量的数据，例如机器人动作、图像视频等。它的核心思想是通过**逐步添加噪声**来破坏数据，再通过**反向去噪过程**逐步恢复数据，从而生成逼真的样本。


## 核心概念

1. **正向扩散过程（Forward Diffusion Process）**： 在正向过程中，扩散模型通过**向数据添加噪声**，使数据逐步变得更加随机。具体来说，给定一个真实样本 $x_0$，通过一步步向数据添加高斯噪声，逐渐将其破坏成接近纯噪声的状态。

	>正向扩散过程可以用一组随机变量 $x_0, x_1, \dots, x_T$​ 表示，其中 $T$ 是扩散过程的步数，最终 $x_T$ 是一个接近标准高斯分布的噪声


2. **反向扩散过程（Inverse Diffusion Process）**：通过去噪和条件信号的指引，逐步生成光滑、无噪声符合条件逻辑的目标样本。 
	- **去噪 （Denoising Step）**：根据当前带噪声的轨迹去识别噪声的分布
	- **条件信号（Conditioning）**：在去噪的过程中，根据指令信号的指引通过注意力机制生成指定的目标轨迹。


## DDPM

DDPM（Denoising Diffusion Probability Model）去噪扩散概率模型是扩散模型的基本框架，其核心思想是：
- 前向过程：通过一个前向“加噪”过程，把数据逐步破坏成纯高斯噪声
- 反向过程：然后训练一个神经网络来学习逆过程（逐步去噪），恢复原始分布

### 前向过程
 把干净的数据$x_0$逐渐加噪声直到变成各向同性的高斯噪声，
 定义一个马尔可夫链：
$$
q(x_t \mid x_{t-1}) = \mathcal{N}(x_t;\sqrt{1-\beta_t}x_{t-1}),\beta_tI)
$$
- $\sqrt{1-\beta_t}x_{t-1}$
    - 这是均值项，表示 $x_t$ 会接近于前一步的样本 $x_{t-1}$，但稍微被缩小。
    - 乘 $\sqrt{1-\beta_t}$ 是为了控制“保留多少原始信息”，确保信号逐渐衰减
    - 当 $\beta_t$ 小 → $\sqrt{1-\beta_t} \approx 1$，几乎保持原样。
    - 当 $\beta_t$ 大 → 衰减得更明显，数据逐步走向“纯噪声”。
- $\beta_t I$
    - 这是协方差矩阵，表示噪声强度
    - $\beta_t$ 为噪声强度，通常是一个小正数，随时间逐步增加，该值越大，随机扰动越强
    - $I$ 表示噪声在各个维度上独立同分布（各方向噪声一样）。

把这个高斯公式写成生成公式：
$$
x_t = \sqrt{1-\beta_t}x_{t-1}+\sqrt{\beta_t}\epsilon, \quad \epsilon\in\mathcal{N}(0,I)
$$
经过 T 步之后，$x_T \sim \mathcal{N}(0,I)$，即完全变为纯噪声

闭式展开过程：（直接从$x_0$采样$x_t$）：
$$
x_t = \sqrt{\overline\alpha_t}x_0 + \sqrt{1-\overline\alpha_t}\epsilon, \quad \epsilon \in \mathcal{N}(0,I)
$$
- $\alpha_t =1-\beta_t$
- $\bar{\alpha}_t=\prod_{s=1}^t \alpha_s$ ：累积衰减系数

### 逆向过程
从纯噪声一步步还原回$x_0$，目标是学到近似的反向分布
$$
p_\theta(x_{t-1}|x_t) \approx q(x_{t-1}|x_t,x_0)
$$

**理论反向分布：**
$$
p_\theta(x_{t-1},x_t) = \mathcal{N}(x_{t-1};\mu_\theta(x_t,t),\Sigma_\theta)
$$
在 DDPM 里简化为：
- 假设$\Sigma_\theta$固定（或简单可学习）
- 只训练网络预测均值部分$\mu_\theta$

**真实后验证分布：**
$$
 q(x_{t-1}|x_t,x_0)=\mathcal{N}(x_{t-1};\widetilde\mu_t(x_t,x_0),\widetilde\beta_tI)
$$
- $\widetilde\mu_t$：后验均值（依赖$x_t$和$x_0$）
- $\widetilde\beta_tI$是方差（有解析表达式）
训练时，$x_0$不可见，所以要训练一个神经网络$\epsilon_\theta(x_t,t)$来预测噪声$\epsilon$

### 重参数化
通过重参数化，把 $\mu_\theta$ 写成依赖于预测噪声 $\epsilon_\theta(x_t,t)$的形式：

$$
\mu_\theta(x_t,t) = \frac{1}{\sqrt{\alpha_t}}(x_t-\frac{1-\alpha_t}{\sqrt{1-\bar\alpha_t}}\epsilon_\theta(x_t,t))
$$
于是采样公式：
$$x_{t-1} = \mu_{\theta}(x_t,t) +\sigma_tz \quad z～\mathcal{N}(0,I)$$
- $\sigma_t$：控制采样时的方差

### 训练过程

由前向闭式：
$$
x_t = \sqrt{\overline\alpha_t}x_0 + \sqrt{1-\overline\alpha_t}\epsilon
$$
可以解出$\epsilon$：
$$\epsilon =\frac{x_t-\sqrt{\bar\alpha_t}x_0}{\sqrt{1-\bar\alpha_t}}$$
于是训练的目标就是让网络预测的噪声和真实噪声接近：
$$
L(\theta) = \mathbf{E}_{t,x_0,\epsilon}[||\epsilon-\epsilon_\theta(x_t,t)||^2]
$$
网络学习的目标就是给定$x_t$，恢复“加了多少噪声”
从高斯噪声$x_T \sim \mathcal{N}(0,I)$开始逐步去噪：
$$
x_{t-1} = \frac{1}{\sqrt{\alpha_t}}(x_t-\frac{1-\alpha_t}{\sqrt{1-\bar{\alpha_t}}}\epsilon_\theta(x_t,t)) + \sigma_t z
$$

训练流程：
***Step 1. 数据采样***
从真实数据分布中采样一个干净样本：
$$x_0 \sim q(x_0)$$
这里的 $q(x_0)$ 表示训练数据分布，比如图像分布、人类动作分布等。
***Step 2. 时间步采样***
随机选择一个扩散时间步：
$$t \sim \{1, 2, \dots, T\}$$
这样保证模型能在所有噪声水平下都学到去噪的能力，而不是只学某个固定程度的噪声。

***Step 3. 前向加噪***

利用闭式公式构造 $x_t$：
$$x_t = \sqrt{\bar{\alpha}_t} \, x_0 + \sqrt{1 - \bar{\alpha}_t}\, \epsilon, \quad \epsilon \sim \mathcal{N}(0, I)$$

其中：

- $\bar{\alpha}_t = \prod_{s=1}^t (1-\beta_s)$ 控制噪声累积；
- $\epsilon$ 是显式采样的高斯噪声；
- $x_t$ 就是“部分被破坏的数据”。
 这一步相当于告诉网络：**我在时间 $t$ 把干净数据加了噪声，你能不能学会把噪声找回来？**

***Step 4. 网络预测噪声***
把 $(x_t, t)$ 输入神经网络：
$$\hat{\epsilon}_\theta = \epsilon_\theta(x_t, t)$$
其中：
- $\epsilon_\theta(\cdot)$ 通常是 U-Net 或 Transformer；
- 它的任务不是直接预测 $x_0$，而是预测加在 $x_0$ 上的噪声 $\epsilon$。  
    这样做的原因是：**噪声在所有样本分布里都服从标准高斯，更容易学习和泛化。**

***Step 5. 定义训练目标***
比较真实噪声和网络预测噪声，最小化均方误差：
$$L(\theta) = \mathbb{E}_{t,x_0,\epsilon} \Big[ \lVert \epsilon - \epsilon_\theta(x_t, t) \rVert^2 \Big]$$
这个损失等价于最大化反向过程的变分下界 (ELBO)，只是用 MSE 实现更高效。

***Step 6. 参数更新***
使用随机梯度下降 (SGD/AdamW 等) 更新 $\theta$：
$$
\theta \leftarrow \theta - \eta \nabla_\theta L(\theta)
$$
