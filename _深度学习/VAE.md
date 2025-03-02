
# Abstract

**变分自编码器（VAE, Variational Autoencoder）** 是一种生成模型，它通过学习数据的概率分布来生成新的样本。相对于传统的自编码器（Autoencoder），VAE 对编码的潜在空间进行约束，使得该空间能够生成**更平滑、连续的分布**，从而提高生成样本的质量和多样性。VAE 的提出主要解决了传统 Autoencoder 在生成新数据时难以控制生成质量的问题。

![[Pasted image 20241111203040.png]]

作用：获取数据并将其压缩成低维表示，然后将其重建为原始形式。


# VAE 的实现过程

VAE 的实现与普通 Autoencoder 有一些相似之处，但在细节上有很大的不同，具体实现步骤如下：
## 编码器（Encoder）
   - 与传统 Autoencoder 不同，VAE 的编码器不仅仅是将输入数据压缩为一个固定的潜在表示（例如一个向量），而是生成一个**概率分布**的参数，即潜在空间的均值和方差。
   - 给定输入$x$，编码器生成潜在变量$z$ 的分布参数$\mu$（均值）和$\sigma$（标准差）。
   - 这些参数通常使用神经网络的全连接层来输出，因此，编码器输出的是$\mu (x)$ 和$\sigma (x)$ 两个向量。

## 重参数化（Reparameterization）
   - 为了从生成的分布中采样潜在变量$z$，VAE 引入了重参数化技巧。
   - 具体来说，从正态分布$N (\mu, \sigma^2)$ 采样$z$ ，即：
     $$
     Z = \mu (x) + \sigma (x) \cdot \epsilon
     $$
     其中$\epsilon \sim N (0, 1)$ 是标准正态分布的噪声。
   - 这个技巧的关键在于，使得采样$z$ 的过程变为可微，从而可以进行梯度传播，进而用反向传播法来优化模型。

## 解码器（Decoder）
   - 解码器接受潜在变量$z$ 作为输入，生成数据的重构$\hat{x}$，即解码器输出的是对输入$x$ 的重构。
   - 通过学习解码器的参数，VAE 试图使得重构数据$\hat{x}$ 与原始数据$x$ 尽可能相似。

## 损失函数（Loss Function）
VAE 通过最大化**变分下界（ELBO）** 来优化。VAE 的损失函数由两部分组成：
   - **重构损失**：衡量重构数据$\hat{x}$ 和输入数据$x$ 的差异，通常使用均方误差（MSE）或交叉熵损失。重构损失保证解码器能够准确地重构输入数据。
   - **KL 散度损失**：将编码器生成的潜在分布$q (z|x)$ 与先验分布$p (z)$（通常为标准正态分布）进行比较。通过最小化 KL 散度，使得潜在表示$z$ 接近标准正态分布。
     $$
     \text{KL}(q (z|x) \| p (z)) = \int q (z|x) \log \frac{q (z|x)}{p (z)} \, dz
     $$
   VAE 的总损失函数为：
   $$
   \text{Loss} = \text{重构损失} + \text{KL 散度损失}
   $$

训练过程：
   - 通过反向传播最小化总损失函数，优化编码器和解码器的参数。
   - 训练完成后，可以通过解码器从标准正态分布中采样潜在变量$z$，生成新的数据样本。

## VAE 相对于 Autoencoder 的改进

1. **潜在空间的结构化分布**：
   - VAE 通过 KL 散度将潜在空间**约束为标准正态分布**。这样编码的潜在空间是连续的，样本间的**过渡平滑**，使得采样新的$z$ 值生成的样本更有意义和一致性。
   - 传统 Autoencoder 的潜在空间可能没有明确的结构，导致采样生成新样本时，可能会生成无法识别的数据。

2. **生成数据的能力**：
   - VAE 是生成模型，能够生成符合训练数据分布的新样本。只需从标准正态分布中采样潜在变量$z$，然后通过解码器生成数据。
   - 传统 Autoencoder 没有约束**潜在空间的分布**，因此生成数据的效果较差，难以保证生成数据的质量。

3. **概率模型**：
   - VAE 是概率模型，它为输入数据生成了一个潜在分布（均值和方差）。这种分布化的表示不仅提高了生成数据的多样性，还允许在潜在空间中探索不同样本的潜在变量变化。
   - Autoencoder 是确定性的映射，没有使用概率模型，不具备 VAE 的生成优势。

# $\beta-VAE$

普通 VAE 可能会导致潜在变量之间的相关性较强，从而难以得到可解释的独立特征。

β-VAE 通过**引入一个超参数 $\beta$**，对 KL 散度进行加权，以控制信息瓶颈的强度。其目标函数变为：

$$\mathcal{L}_{\beta} = \mathbb{E}_{q(z|X)} [\log p(X|z)] - \beta D_{KL} (q(z|X) || p(z))$$

其中：

- **$\beta > 1$**：加大 KL 散度的权重，使得潜在变量更加接近于标准正态分布 $\mathcal{N}(0, I)$，从而增强了**特征解耦性**。
- 当 $\beta = 1$，β-VAE 退化为普通 VAE。
- **$\beta < 1$**（通常不用）：允许更自由的编码，但可能导致潜在变量分布与先验分布差异过大，影响生成效果。


# Basics

## 工作原理

### 1. 数学建模 
 **1. 潜在变量模型假设**
VAE 假设数据 $x$ 是由潜在变量 $z$ 生成的，生成的过程遵循：
1. 从先验分布 $p(z)$ 中采样潜在变量 $z$   --- Encoder
2. 根据 $z$ 的值生成数据 $x$ ，即 $p_\theta(x|z)$  --- Decoder

目标是学习到 $p(z)$ 和  $p_\theta(x|z)$ ，从而可以生成新的数据样本。

### 2. 目标函数

**1. 最大化边际似然估计** [[常用解释#最大化边际似然]]

根据潜在变量模型假设的目标，为了尽可能生成我们期望的数据样本，我们需要最小化我们期望的数据真实分布 $p_\mathrm{data}(x)$ 和预测的模型分布 $p_\theta(x)$ 的 KL 散度，即最大化 $\log p_\theta(x)$：
$$\log p_\theta(x) =\log\int p_\theta (x,z) dz= \log \int p_\theta(x|z)p(z) \, dz$$
但由于积分难以直接求解，VAE 使用变分推断，尝试逼近。


----
*目标公式推导*
最大化 $\log p_\theta(x)$ 等价于最小化数据真实分布 $p_\mathrm{data}(x)$ 和模型分布 $p_\theta(x)$ 的 KL 散度，公式为：
$$D_{\mathrm{KL}}(p_{\mathrm{data}}(x)\|p_{\theta}(x))=\mathbb{E}_{p_{\mathrm{data}}(x)}\left[\log p_{\mathrm{data}}(x)-\log p_{\theta}(x)\right]$$
由于$\mathbb{E}_{p_\mathrm{data}(x)}[\log p_\mathrm{data}(x)]$是一个常数 (与模型无关), 最小化$D_\mathrm{KL}$等价于最大化期望对数似然：$\mathbb{E}_{p_\text{data}(x)}[\log p_\theta(x)].$。



**2. 变分推理**

VAE 引入一个近似分布 $q_\phi(z|x)$ ，用于近似 $p_\theta(z|x)$ ，通过**证据下界**（Evidence Lower Bound, ELBO ）对 $logp_\theta(x)$ 进行下界逼近：
$$\log p_\theta(x) \geq \mathbb{E}_{q_\phi(z|x)}[\log p_\theta(x|z)] - D_{\text{KL}}(q_\phi(z|x) \| p(z))$$
等价于：
$$\mathcal{L}(\theta, \phi; x) = \mathbb{E}_{q_\phi(z|x)}[\log p_\theta(x|z)] - D_{\text{KL}}(q_\phi(z|x) \| p(z))$$
- **重构误差 $\log p_\theta(x|z)$ ：** 表示从潜在变量 z 生成数据 x 的准确性。最大化这一项能提升生成数据的质量。
- **KL 散度 $D_{\text{KL}}(q_\phi(z|x) \| p(z))$** ： 衡量近似分布 $q_\phi(z|x)$ 和先验分布$p(z)$之间的差异。通过最小化这一项，可以使潜在空间中的分布符合先验假设。

最大化 ELBO 同时在优化数据重构能力和正则化潜在空间结构，这正是 VAE 的设计目标。

---
*证据下界推导*
证据下界用于近似计算数据的边际对数似然$\log p_\theta(x)$，由于直接计算$\log p_\theta(x)$通常非常困难，ELBO 提供了一个可计算的下界，优化 ELBO 可以间接优化 $\log p_\theta(x)$。

为了简化计算，引入一个变分分布 $q_\phi(z|x)$，它是对后验分布 $p_\theta(z|x)$ 的一种近似。通过 **对数的性质**，将 $\log p_\theta(x)$ 重写为：
$$\log p_\theta(x) = \log \int q_\phi(z|x) \frac{p_\theta(x, z)}{q_\phi(z|x)} \, dz$$
由于log 是凹函数，可以使用 **Jensen 不等式** 将其向外移到积分外：
$$\log \int q_\phi(z|x) \frac{p_\theta(x, z)}{q_\phi(z|x)} \, dz \geq \int q_\phi(z|x) \log \frac{p_\theta(x, z)}{q_\phi(z|x)} \, dz$$

右侧的表达式即为$\log p_\theta(x)$的一个下界，这就是 **证据下界（ELBO）**：
$$\mathcal{L}(\theta, \phi) = \int q_\phi(z|x) \log \frac{p_\theta(x, z)}{q_\phi(z|x)} \, dz$$
或者更直观地写成期望形式：
$$\mathcal{L}(\theta, \phi) = \mathbb{E}_{z \sim q_\phi(z|x)} \left[ \log \frac{p_\theta(x, z)}{q_\phi(z|x)} \right]$$

将联合分布 $p_\theta(x, z) = p_\theta(x|z)p_\theta(z)$代入：
$$\mathcal{L}(\theta, \phi) = \int q_\phi(z|x) \log \frac{p_\theta(x|z)p_\theta(z)}{q_\phi(z|x)} \, dz$$
拆分成两部分：
$$\mathcal{L}(\theta, \phi) = \int q_\phi(z|x) \log p_\theta(x|z) \, dz + \int q_\phi(z|x) \log \frac{p_\theta(z)}{q_\phi(z|x)} \, dz$$

重写为期望形式：
$$\mathcal{L}(\theta, \phi) = \mathbb{E}_{z \sim q_\phi(z|x)} \left[ \log p_\theta(x|z) \right] - \text{KL}(q_\phi(z|x) \| p_\theta(z))$$

VAE 的优化目标是 **最大化 ELBO**，以更好地拟合数据分布并提高生成质量

[[常用解释#Jensen 不等式]]
### 3. 学习过程
通过先验分布 $p(z)$ 和解码器 $p_\theta(x|z)$ 的组合，能够生成尽可能逼近数据分布 $p(x)$ 的样本。

**1. 编码器：学习近似后验分布**
编码器网络将数据 x 映射到潜在变量的分布参数（均值 $\mu$ 和对数方差 $\log\sigma^2$），定义近似后验分布：
$$q_\phi(z|x) = \mathcal{N}(z; \mu_\phi(x), \text{diag}(\sigma_\phi^2(x)))$$

**2. 采样潜在变量**
从分布$q_\phi(z|x)$ 中采样 z。  
为了使梯度可传播，使用 **重参数化技巧**：
$$z = \mu_\phi(x) + \sigma_\phi(x) \cdot \epsilon, \quad \epsilon \sim \mathcal{N}(0, I)$$
将随机变量 $z$ 的采样过程转化为一个确定性可微的操作，解决了随机采样无法直接传播梯度的问题。

**3. 解码器：生成数据**
解码器网络根据采样的 z 生成数据分布 $p_\theta(x|z)$，通常假设为高斯分布：
$$p_\theta(x|z) = \mathcal{N}(x; f_\theta(z), \sigma^2 I)$$

**4. 损失函数**
VAE的目标是**最大化 ELBO**，对应的损失函数为：
$$
\mathcal{L}(\theta, \phi; x) = \text{重构损失} + \text{KL散度正则化}
$$
具体形式：
$$\mathcal{L} = \mathbb{E}_{q_\phi(z|x)}[\log p_\theta(x|z)] - D_{\text{KL}}(q_\phi(z|x) \| p(z))
$$

在 VAE 中，我们希望最大化数据的边际对数似然，直接优化边际对数似然很困难，因为需要计算复杂的积分$\int p_\theta(x, z) dz$。因此，VAE 引入了变分分布 $q_\phi(z|x)$，用它来近似后验分布 $p_\theta(z|x)$，并将目标重新表述为最大化 ELBO。

--- 
**先验分布**：
在 VAE 中，先验分布 $p(z)$ 是一个人为选择的分布，它是生成模型的一部分，用来描述潜在空间 $z$ 的结构。希望通过先验分布 $p(z)$ 和解码器 $p_\theta(x|z)$ 的组合，能够生成尽可能逼近数据分布 $p(x)$ 的样本。

[[常用解释#先验分布]]

## Autoencoder

Autoencoder（自编码器）是一种**无监督学习模型**，通过编码-解码结构学习数据的压缩和重构。
![[Pasted image 20241111201548.png|425]]

***编码器（Encoder）***：将输入数据压缩成低维的潜在空间（latent space，$z$）表示。
- **输入层**：接收原始数据（如图像的像素值或特征向量）。
- **隐藏层**：通过一系列非线性变换，将数据从高维空间映射到低维潜在空间，得到“编码”后的表示 $z$。
- **作用**：是提取数据的主要特征，因此潜在空间的维度通常比输入数据维度低得多。

***解码器（Decoder）***：将潜在表示还原为原始数据的近似。
- **隐藏层**：从编码器生成的潜在表示开始，通过非线性变换将其还原成输入数据的高维表示。
- **输出层**：解码器的输出尽可能接近输入数据，用于训练时的重构损失计算。
- **损失函数**：计算解码器的输出与输入之间的差异，通常使用均方误差（MSE）或交叉熵损失。
- **作用**：是尽量还原编码的输入数据。

***作用***：
- 数据降噪
- 数据生成
- 降维与特征提取
- 异常检测
- 图像超分辨率

***优点***：
- 有效的无监督学习，可以用于无标签数据的特征提取。
- 能够自适应地学习数据的低维表示，实现数据降维、去噪、生成等任务。

***缺点***：
- 通常没有标签监督，难以确保潜在表示具有明确的意义。
- 可能会出现过拟合，尤其是对小数据集，导致无法泛化到新数据。
- 重构效果不一定总是能达到理想效果，特别是在处理复杂数据时

***Bottleneck &&  Latent Space*** 
瓶颈层（Bottleneck）是自编码器模型结构中的具体层，是一个实现信息压缩的具体位置。
潜在空间（Latent Space）是数据在瓶颈层压缩后的表示，是一个数学概念和数据空间


## PCA

后验分布 （posterior distribution） p (x| z) ：对于给定图像 x ，生成潜在向量 z 的概率。-- 对应Decoder
似然分布（likelihood distribution） q (z|x) ：对于给定潜在向量 z ，重建图像 x 的概率。--- 对应Encoder

重参化（reparameterization）

# 总结


