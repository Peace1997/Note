深度学习框架
底层数据结构 tensor
类似于Numpy数据结构，相当于把Numpy放到GPU上跑


ipynb

**为什么需要 pytorch？**
- 使用率逐年上涨
- 方便Debug
- 方便并行数据计算

**Tensor 计算**
- 基本数据运算
- 支持向量、矩阵运算
- numpy（数据计算）和 tensor（模型） 转换
	- `torch.from_numpy` ： 共用一个内存，改 numpy，tensor 也变 ； --》深拷贝？？？
	- `torch.tensor()` ：tensor 独立
	深拷贝和浅拷贝学习

> 实际应用时，浅拷贝多一些


**Tensor常用操作：**
-  加法 ： `torch.add`
- 乘法 ： `torch.dot` 、 `torch.mv` 矩阵 * 向量 、 `torch.mm` 矩阵* 矩阵；矩阵乘法计算前通常需要 shape 对齐，而 Tensor Contractions  （`@`）可以完成 shape 对齐（仅对齐）
- 求和 `sum`、求平均 `mean`、求标准差 `std` 求最大 `max` 、求最小 `min` ；`axis` 指定维度（可指定多个维度）
- Transpose ：维度交换  `torch.arrange`  
- View & Reshape 维度转换，不确定某一维度时用 -1
	- View：`view` ， 不会进行拷贝tensor
	- Reshape：`reshape` ，会进行拷贝
- Squeeze & Unsqueeze 维度降低、增加
	- Squeeze：`squeze`  省略维度中的所有一维
	- Unsqueeze：`unsqueeze` 在某处额外增加一个一维
- Expand & Repeat 数据复制
	- Expand ：`expand` ；共享内存的数据复制
	- Repeat：`repeat`； 非共享内存的数据复制
- Indexing： 定位 `5：`，同 numpy 
- Cat & Stack  数据拼接
	- Cat：`cat`  
	- Stack：`stack`
- Gather ：`gather(x,dim,index)` 在矩阵取元素
- Broadcasting： 可以完成部分不对齐 shape 相乘（只允许有一维对不齐）
- Sparse Tensors `torch.sparse` 优化解决大稀疏矩阵的乘法


**Pytorch Variables:**
用于封装 Tensor 类型的 数据、梯度、梯度所用的方程 （后俩个用于反向传播）的变量，常用于前向、反向传播的梯度计算。
使用方法：`requires_grad =True`，即：
`torch.tensor (data,requires_grad =True)`
![[pytorch.png]]
补：
`from torchviz import make_dot` 打印计算流程


**Pytorh AuotGrad —— Gradient、BackPropagation**

`loss.backward()`

**Pytorch Modules 模组**
父类：`torch.nn.Module`


参数设置：`nn.Parameter`
定义单层接层：`torch.nn.Linear()`
定义多层：`nn.Sequential(nn,Linear() nn.ReLU  ... )`
优化器：`torch.optim.SGD` ；
损失函数：`torch.nn.MSELoss()`

优化器优化是模型的参数（梯度下降、上升）

y = wx+b ；数据 x 是固定的，无梯度，w、b 带梯度的

1. 定义模型，初始化参数$\theta$
2. 采样数据，前向传播计算损失
3. 反向传播计算损失函数梯度，
4. 优化器优化参数


定位损失爆炸：



**DataLoader**
构建“数据库” 

`import torch.utils.data import Data`
把数据传入Dataset进行包装
`torch_dataset = Data.TensorDataset()`
`loader = Data.DataLoader(dataset = torch_dataset, batch_size = ...`


**Tensorboard**
`from torch.utils.tensorboard import SummaryWriter`

**Sava and Load models**
保存整个网络模型：`torch.save(net`)`
只保存参数 `torch.save(net.state_dict)`

调用：`torch.load(net.pkl)`


**RNN**
`nn.RNN()  nn.GRU  nn.LSTM`



[RLchina 暑期课2022：pytorch入门](https://www.bilibili.com/video/BV1Q14y1b7yb?share_source=copy_web&vd_source=4208bd08a8472d32da44b0506a34efb8)




