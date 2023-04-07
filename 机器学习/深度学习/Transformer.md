#多头注意力机制 #残差结构

# 一、简述

## 1. 什么是Transformer？

Transformer 是一种深度学习模型，用于处理序列数据。Transformer 由 Encoder 和 Decoder 两个部分组成。
- Encoder 主要将输入序列进行编码，生成一个对应的上下文向量，该向量包含输入序列的语义信息
- Decoder 主要根据上下文向量和之前生成的输出序列，生成目标序列的预测。
>其中Encoder将输入序列编码为一个固定长度的向量，解码器再将向量解码为输出序列。


## 2. Transformer 优势是什么？

利用注意力机制（多头注意力机制）来提高训练速度的模型，支持并行化计算，由于增加了模型的复杂度，使得它在精度和性能上都要优于 RNN 循环神经网络

在 RNN 中，输入数据是有先后顺序的，之前前面的输入数据输入计算完成后才能计算后面的输入数据，因此 RNN 无法同时并行计算。

在Transformer中，对于输入数据是可以同时输入的，通过注意力机制，输入序列中的每个位置都会被赋予一个权重，这些权重指示了该位置在整个序列中的重要性，因此可以同时并行处理。

> 加强了对注意力的使用，使用多头注意力机制
> 它不像RNN一样每次都对整个句子加深理解，而是每次都注意到句中不同的部分，提炼对句子更深层的理解。


# 二、Transformer 的结构

![[Transformer1.png]]

主要包含一个编码层（Encoders） 和解码层（Decoders） ，在 Encoders、Decoders 中存在多个编码器 Encode模块、解码器 Decode模块。

- 【输入层】：输入一个序列数据，并将每个元素映射为一个词嵌入向量
- 【编码器】：每个模块由多头自注意力层（self-attention layer）和前馈层（feedforward layer）组成。自注意力层使用 [[注意力机制]]  。在每个注意力机制中，输入序列中的每个位置都会被赋予一个权重，这些权重指示了该位置在整个序列中的重要性。通过多个自注意力机制的组合，Encoder可以学习到输入序列不同层次的表示，从而生成上下文向量的表示。前馈层则使用全连接神经网络来提取高级特征，它的输入是前一个编码器（多头注意力机制）的输出。
- 【解码层】：每个模块由多头自注意力层、多头注意力层和前馈神经网络组成。在多头注意力机制中，Decoder会将
- 【输出层】：将解码器输出的向量转化为输出序列的每个元素。
对于每个解码器 decode，它的输入 包括两个部分：前一个解码器的输出+整个编码层 Encoders 的输出。

## 1. 输入层
在 Transformer 模型中，输入层（Encoder Input Embedding）是将输入的词嵌入向量映射到模型的隐藏层表示的关键组成部分之一。其结构主要包括以下几个部分：

### 1.1 Word Embedding
词嵌入层（Word Embedding Layer）：将输入的词转换成一个向量表示，以便于模型进行处理。在这个层中，每个输入的词都被映射为一个高维向量，这个向量包含了词的上下文信息和语义信息。这个向量通常是通过预训练的词嵌入模型（如 Word2Vec、GloVe 等）得到的，也可以是随机初始化的。

### 1.2 Positional Encoding
位置编码层（Positional Encoding Layer）：因为 Transformer 模型是基于注意力机制的，所以它**不能处理序列中的顺序信息**。为了解决这个问题，位置编码层会将输入序列中每个词的位置信息编码成一个**与词嵌入向量相同维度的位置向量**，然后将词嵌入向量和位置向量相加，以保留词序信息。在词嵌入层之后，位置编码层对输入的序列进行位置编码，以将序列中的词的位置信息引入模型。位置编码是一种类似于正弦函数的函数，可以在不增加模型参数的情况下引入位置信息。

具体来说，位置编码的基本思想是使用正弦和余弦函数来编码单词在序列中的位置，不同位置的单词将得到不同的位置编码向量，从而使得模型可以区分不同位置的单词。

$$
\begin{gathered}
P E_{(pos,2i)}=\sin \left(\frac{pos}{10000^{2i/d}}\right) \\
P E_{(pos,2i+1)}=\cos \left(\frac{pos}{10000^{2i/d}}\right)
\end{gathered}
$$
其中，$pos$ 表示单词在序列中的位置，$i$ 表示位置编码向量中的第 $i$ 个元素，$d$ 表示词嵌入向量的维度。位置编码向量的长度和词嵌入向量的长度相同。位置编码的基本思想是使用正弦和余弦函数来编码单词在序列中的位置，不同位置的单词将得到不同的位置编码向量，从而使得模型可以区分不同位置的单词。
> 2i 表示偶数的维度，2i+1 表示奇数维度。这样的公式计算使 PE 能够适应比训练集里面所有句子更长的句子，假设训练集里面最长的句子是有 20 个单词，突然来了一个长度为 21 的句子，则使用公式计算的方法可以计算出第 21 位的 Embedding。

最后，将词嵌入层与位置编码层相加即可得到Embedding向量**X**，该向量可作为Encoder的输入，例如：
![[Transformer1.png]]


## 2. 编码器层
Encoders 主要包含三个模块一个是自注意力层（self-attention）、前馈神经网络层（feed forward）、Add&Norm。

### 2.1 Self-Attention
1. 首先，在自注意力层会计算出三个新的向量，这三个向量分别为 Query（查询向量）、Key（键向量）、Value（值向量）。这三个向量是使用 Embedding 向量 X 分别与三个线性变换矩阵**WQ、WK、WV**相乘得到的结果：**注意 X, Q, K, V 的每一行都表示一个单词/输入 $x_i$）。**
![[Transformer3.png|375]]

> -   Q（Query）向量表示查询向量，是用来和 K 向量进行点积操作以得到注意力权重的向量。在编码器中，Query 向量是来自上一层的输出向量。
> - K（Key）向量表示键向量，用于与 Query 向量计算注意力权重。在编码器中，Key 向量是来自上一层的输出向量。
> - V（Value）向量表示值向量，用于与注意力权重相乘并求和得到上下文向量，最终用于生成下一层的输出。在编码器中，Value 向量也是来自上一层的输出向量。
> - 使用Q、K、V向量的自注意力机制可以对输入的每个元素进行加权聚合，并产生对应的输出表示。这种机制可以有效地捕获文本中的上下文关系

2. 得到矩阵 Q, K, V之后就可以计算出 Self-Attention 的输出了，计算的公式如下
$$\operatorname{Attention}(Q, K, V)=\operatorname{softmax}\left(\frac{Q K^T}{\sqrt{d_k}}\right) V$$
$d_k$ 是 $Q, K$ 矩阵的列数, 即向量维度, 除以 $d_k$ 的原因是防止 $Q K^T$ 的内积过大，然后把得到的结果做一次 Softmax 计算，得到每一个单词向量对于其当前位置的词的相关性大小（Attention 系数）。
![[Transformer4.png|500]]

#### Multi-Head Attention 
Multi-Head Attention 多头注意力机制包含多个 Self-Attention 层，即不仅仅初始化一组Q、K、V矩阵，而是初始化多组，Transformer中使用了8组，所以最后得到的结果是8个矩阵。

![[Transformer5.png|475]]

得到 8 个输出矩阵$Z_1$到$Z_8$之后，Multi-Head Attention 将它们拼接在一起 (Concat)，然后传入一个**Linear**层，得到 Multi-Head Attention 最终的输出**Z**。

![[Transformer6.png|475]]


Multi-Head Attention 输出的矩阵**Z**与其输入的矩阵**X**的维度是一样的。

### 2.2 Add & Norm
在 transformer 中，每一个子层（self-attetion，Feed Forward Neural Network）之后都会接一个残缺模块 (Add)，并且有一个 Layer normalization（Norm）。
- **Add** 是一种残差连接，通常用于解决多层网络训练的问题，可以让网络只关注当前差异的部分，在 ResNet 中经常用到；
- **Norm**指 Layer Normalization，通常用于 RNN 结构，Layer Normalization 会将每一层神经元的输入都转成均值方差都一样的，这样可以加快收敛。
![[Transformer7.png|425]]

> Normalization有很多种，但是它们都有一个共同的目的，那就是把输入转化成均值为0方差为1的数据。我们在把数据送入激活函数之前进行normalization（归一化），因为我们不希望输入数据落在激活函数的饱和区。

### 2.3 Feed Forward
Feed Forward 层比较简单，是一个两层的全连接层，第一层的激活函数为 Relu，第二层不使用激活函数，对应的公式如下。
$$
\max \left(0, X W_1+b_1\right) W_2+b_2
$$
**X**是输入，Feed Forward 最终得到的输出矩阵的维度与**X**一致。

### 2.4 总结

Encode 接受输入矩阵 $X_{n\times d}$，然后输出一个矩阵 $C_{n\times d}$，多个 Encoder 叠加就组成了 Encoder。其中第一个 Encoder 的输入为输入 X，后面的 Encoder 的输入是前一个 Encoder 的输出，最后一个 Encoder 输出的矩阵就是编码信息矩阵 **C**，这一矩阵后续会用到 Decoder 中。

>在 Transformer 中，Encoder 的输入为一个序列，经过多层自注意力机制和前馈神经网络处理后，会得到一个编码信息矩阵 C。该矩阵 C 可以看做是对输入序列的一种抽象表示，其中每个元素代表了对输入序列中对应位置的信息进行编码后得到的表示向量。
>与此同时，Encoder 还会输出一个最终的输出向量 Z，该向量是由 C 矩阵的加权平均值得到的。在计算加权平均值时，每个位置的权重由另外一个自注意力机制产生，该自注意力机制能够学习到不同位置的重要性，从而为加权平均提供权重。
>因此，C矩阵和Z向量都是Encoder的输出，它们之间的联系在于，Z是C的一种加权平均，其中权重由自注意力机制产生。Z可以被看做是对整个输入序列的一种汇总表示，相较于C矩阵更加简洁。在一些任务中，可以直接使用Z作为Encoder的输出，而无需考虑C矩阵。但在某些情况下，C矩阵的详细信息也可能是有用的，因此需要分别考虑它们的作用。


![[Transformer8.png]]

## 3. 解码器层
Decoder部分其实和Encoder部分大同小异，刚开始也是先添加一个位置向量Positional Encoding。接下来接的是Masked mutil-head attetion，这里的mask也是transformer一个很关键的技术，其余的层结构与Encoder一样。

### 3.1 Masked Multi-Head Attention
Decoder block 的第一个 Multi-Head Attention 采用了 Masked 操作，因为在翻译/输出的过程中是顺序输出的，即输出完第 i 个值，才可以输出第 i+1 个值。通过 Masked 操作可以防止第 i 个单词知道 i+1 个单词之后的信息。

Transformer 模型里面涉及两种 Mask，分别是 Padding mask 和 Sequence mask。
- **Padding Mask 是为了对输入序列进行对齐**。因为每个批次输入序列长度是不一样的也就是说，我们要对输入序列进行对齐。具体来说，就是给在较短的序列后面填充 0。但是如果输入的序列太长，则是截取左边的内容，把多余的直接舍弃。因为这些填充的位置，其实是没什么意义的，所以我们的 Attention 机制不应该把注意力放在这些位置上，所以我们需要进行一些处理。
>具体的做法是：把这些位置的值加上一个非常大的负数 (负无穷)，这样的话，经过 softmax，这些位置的概率就会接近 0！而我们的 padding mask 实际上是一个张量，每个值都是一个 Boolean，值为 false 的地方就是我们要进行处理的地方。

- **Sequence Mask 是为了使得 Decoder 不能看见未来的信息**。也就是对于一个序列，在 time_step 为 t 的时刻，我们的解码输出应该只能依赖于 t 时刻之前的输出，而不能依赖 t 之后的输出。因此我们需要想一个办法，把 t 之后的信息给隐藏起来。
>具体的做法是：产生一个上三角矩阵，上三角的值全为 0。把这个矩阵作用在每一个序列上，就可以达到我们的目的。

具体步骤为：

1. 初始化Mask矩阵。通过词嵌入层和位置编码层后获得输入矩阵X；
2. 接下来的操作和之前的 Self-Attention 一样，通过输入矩阵**X**计算得到**Q, K, V**矩阵。然后计算 $Q K^T$ ；
3. 在进行 Softmax 之前需要使用 Mask 矩阵遮住每一个单词之后的信息：
![[Transformer9.png|500]]
4. 在得到 **Mask  $Q K^T$** 之后，进行 **Softmax** 操作。每一行的和都为 1。但是单词 0 在单词 1, 2, 3, 4 上的 attention score 都为 0。
5. 使用**Mask  $Q K^T$** 与矩阵 **V** 相乘，得到输出 **$Z_1$**
![[Transformer10.png|500]]
7. 通过上述步骤就可以得到一个 Mask Self-Attention 的输出矩阵$Z_i$，然后和 Encoder 类似，通过 Multi-Head Attention 拼接多个输出$Z_i$，然后计算得到第一个 Multi-Head Attention 的输出**Z**，**Z**与输入**X**维度一样。

### 3.2 Encoder-Decoder Attention

Decoder block 第二个 Multi-Head Attention 变化不大， 主要的区别在于其中 Self-Attention 的 **K, V**矩阵不是使用 上一个 Decoder block 的输出/输入矩阵 X   计算的，而是使用 **Encoder 的编码信息矩阵 C** 计算的（因为 Encoder 的编码信息矩阵与输入矩阵X的维度相同）。

根据 Encoder 的输出 **C**计算得到 **K, V**，根据上一个 Decoder block 的输出 **Z** 计算 **Q** (如果是第一个 Decoder block 则使用输入矩阵 **X** 进行计算)，后续的计算方法与之前描述的一致。

这样做的好处是在 Decoder 的时候，每一位单词都可以利用到 Encoder 所有单词的信息 (这些信息无需 **Mask**)。

### 3.3 Linear and Softmax Layer

当多个Decoder模块执行完成后，需要将向量映射为我们想要的输出类型。这时仅需要在结尾添加一个全连接层和Softmax层即可，利用 Softmax 预测下一个单词，在之前的网络层我们可以得到一个最终的输出 Z，因为 Mask 的存在，使得单词 0 的输出 Z0 只包含单词 0 的信息，


![[Transformer11.png|475]]
![[Transformer12.png|475]]


## 4. 整体流程
编码器层通过处理序列数据开启工作，在输入层经过 Word Embedding 和 Position Encoding 操作后获取输入矩阵 X，经过每个 Encoder 中 Multi-Head Atttention、Add&Norm 以及 Feed Foward 后会输出编码信息矩阵 C/矩阵 C 的加权矩阵 Z。在这里，利用Encoder的信息矩阵C去计算 K、V 矩阵。其中K、V 矩阵会被输入到每个Decoder 模块中，帮助解码器层得到输入序列中哪些位置需要被得到关注。
在解码器层，利用Masked Multi-Head Attention掩盖未来信息，依据当前和之前的信息计算得到输出Z矩阵， 通过Z矩阵计算出Q值，与Encoder输出的K、V矩阵一同输入到Encoder-Decoder Attention模块，输出的矩阵经过Linear Softmax Layer后输入下一个Decoder模块。

解码阶段的每个步骤都会输出一个输出序列（在这个例子里，是英语翻译的句子）的元素。接下来的步骤重复了这个过程，直到到达一个特殊的终止符号，它表示transformer的解码器已经完成了它的输出。每个步骤的输出在下一个时间步被提供给底端Decoder，并且就像编码器之前做的那样，这些解码器会输出它们的解码结果 。

![[OverallTransformer.gif|475]]

![[transformer22.png|500]]









**Decode 模块**：自注意力机制 + 注意力 + 前馈神经网络
- 自注意力机制和前馈神经网络与 Encode 模型是相似的。
- Encoder-Decoder 注意力：利用 Encoder 的输出与 Decoder 的自注意力输出做一次注意力，这样 Decoder 就能参考 Encoder 自注意力的结果，这样自注意力才有意义。

预测模块：Softmax
- 将Decoder模块产生的向量映射到更高维度的向量（logits），用于单词的选择





















































简略版
![[机器学习/深度学习/img/transformer.png]]
完整版：
![[机器学习/深度学习/img/transformer22.png]]


>  注意力机制的核心：通过Query、Key、Value，快速准确地找到核心内容，换句话说：**用我的搜索（Query）找到关键内容（Key），在关键内容上花时间花功夫（Value）。**


**什么是多头注意力？**
Multi-head Attention
在同一层做注意力计算的时候，同时多做几次注意力（独立）。他扩展了模型关注不同位置的能力，提高自注意力机制层的性能。
>就是多个人帮我同时理解一句话，我在他们理解的基础上，给出自己的答案






Seq2Seq model 
特点：Self- Attention —— 取代 RNN，可同时计算

在self-attention中，序列数据顺序位置不重要；若要考虑位置顺序，则需要人工加入位置信息$e^i$



参考：http://jalammar.github.io/illustrated-transformer/
