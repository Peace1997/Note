
# Abstract
获取数据，将其压缩成低维表示，然后将其重建为原始形式。


# Basics

潜在空间（latent space）


后验分布 （posterior distribution） p (x| z) ：对于给定图像 x ，生成潜在向量 z 的概率。
似然分布（likelihood distribution） p (z|x) ：对于给定潜在向量 z ，重建图像 x 的概率。

重参化（reparameterization）