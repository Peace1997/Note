

- 好用的开源推理框架：Caffe、NCNN、MNN、TVM、OpenVino
- 好用的半开源推理框架：TensorRT

# 常用框架
## Caffe

Caffe（Convolutional Architecture for Fast Feature Embedding）是一种流行的深度学习框架。Caffe 以其高效性和易用性而闻名，特别适合计算机视觉任务。Caffe的核心是一个基于数据流的计算模型，使用图形表示层次化的神经网络结构。它支持多种常见的神经网络层类型，如卷积层、池化层、全连接层等，并提供了丰富的层组件库。

Caffe 不仅提供了训练深度学习模型的功能，还具备了快速推理的能力。它可以通过 GPU 加速计算，实现高效的模型推理和预测。

## LibTorch

Pytorch 训练出来的模型经过 `torch.jit.trace` 或者 `torch.jit.scrpit` 可以导出为.pt 格式，随后可以通过 Libtorch 中的 API 加载然后运行，因为 Libtorch 是纯 C++实现的，因此 Libtorch 可以集成在各种生产环境中，也就实现了部署。在真实使用场景中，一般都是结合 TensorRT 来部署，TensorRT 负责简单卷积层等操作部分，libtorch 复杂后处理等细小复杂 op 部分。


## TensorRT

TensorRT 是可以在 NVIDIA 各种 GPU 硬件平台下运行的一个 C++推理框架。我们利用 Pytorch、TF 或者其他框架训练好的模型，可以转化为 TensorRT 的格式，然后利用 TensorRT 推理引擎去运行我们这个模型，从而提升这个模型在英伟达 GPU 上运行的速度。速度提升的比例是比较可观的。

在 GPU 服务器上部署的话，TensorRT 是首选。

## OpenVINO
在英特尔CPU端(也就是我们常用的x86处理器)部署首选它！开源且速度很快，文档也很丰富，更新很频繁，代码风格也不错，很值得学习

## NCNN

NCNN 易用性比较高，很容易上手，用了会让你感觉没有那么卷。而且相对于其他框架来说，NCNN 的设计比较直观明了，与 Caffe 和 OpenCV 有很多相似之处，使用起来也很简单。可以比较快速地编译链接和集成到我们的项目中。

# AI 部署提速

## 模型结构
模型结构当然就是探索更快更强的网络结构

## 剪枝
大模型的基础上，对模型通道或者模型结构进行有目的地修剪，剪掉对模型推理贡献不是很重要的地方。

## 蒸馏

大网络教小网络，之后小网络会有接近大网络的精度，同时也有小网络的速度。具体来说，两个网络分别可以称之为老师网络和学生网络，老师网络通常比较大(ResNet50)，学生网络通常比较小(ResNet18)。训练好的老师网络利用 soft label 去教学生网络，可使小网络达到接近大网络的精度

## 稀疏化

稀疏化就是随机将 Tensor 的部分元素置为0，类似于我们常见的 dropout，附带正则化作用的同时也减少了模型的容量，从而加快了模型的推理速度。


# 部署流程

训练好的模型通过以下几种方式转换：

- Pytorch->ONNX->trt onnx 2 trt
- Pytorch->trt torch 2 trt
- Pytorch->torchscipt->trt trtorch

常见的服务部署搭配：

- Triton server + TensorRT/libtorch
- Flask + Pytorch
- Tensorflow Server

