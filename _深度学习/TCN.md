# Introduction

**TCN 是一种基于卷积的序列建模网络**，它完全使用一维卷积代替 RNN/LSTM 来处理时间序列任务，具有并行性强、稳定性好、长依赖捕捉能力强等优势。

TCN 模型以 CNN 模型为基础，并做了如下改进：
1. 适用序列模型：因果卷积（Causal Convolution）
2. 记忆历史：空洞卷积/膨胀卷积（Dilated Convolution），残差模块（Residual block）


# TCN 

## Causal Convolution
- 只允许当前时刻访问**当前及过去的信息**，防止“信息泄露未来”。
- 保证模型对时间的单向性建模。

## Dilated Convolution
设置跳跃间隔，让模型**在不增加参数量的情况下**看到更远的过去。

## Residual Block


