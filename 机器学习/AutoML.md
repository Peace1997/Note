# NNI 
https://nni.readthedocs.io/zh/stable/index.html#id1

## NNI 简介
**什么是 NNI？**
NNI (Neural Network Intelligence) 是一个轻量而强大的工具，可以帮助用户**自动化**的进行：
-   [超参调优](https://nni.readthedocs.io/zh/stable/hpo/overview.html)
-   [架构搜索](https://nni.readthedocs.io/zh/stable/nas/overview.html)
-   [模型压缩](https://nni.readthedocs.io/zh/stable/compression/overview.html)
-   [特征工程](https://nni.readthedocs.io/zh/stable/feature_engineering/overview.html)

**什么是超参调优？**
为一种机器学习算法选择最优超参组合的问题被称为“超参调优”。

## NNI 超参调优
NNI 超参调优的主要功能：调优算法、训练平台、网页控制台
### 调优算法
通过**调优算法**来更快地找到最优超参组合

#### 整体流程：
- **准备模型**
	- 准备待调优的模型，被调优的模型会被独立地运行多次
- **定义搜索空间**
	- 定义的超参数的“搜索空间”：指定它们的取值范围和分布规律
- **配置实验**
	- 定义了如何训练模型、如何遍历搜索空间
- **运行试验&可视化显示**
	- 指定一个端口来运行它，并查看运行结果
https://nni.readthedocs.io/zh/stable/tutorials/hpo_quickstart_pytorch/main.html

### 训练平台
可以利用分布式平台进行训练
### 网页控制台
可以使用 NNI 的网页控制台来监控超参调优实验