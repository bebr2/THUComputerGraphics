# THUComputerGraphics
### 2022春季学期清华大学计算机图形学大作业

代码、报告和最终效果在master分支上

先完成并理解smallpt，在pt基础上可以实现加速、景深、运动模糊等一系列东西，同时用smallpt的思路改成pm/ppm/sppm。

obj模型建议在网络上各种寻找。

而且强推**Rhino**这个软件！！！！可以帮你处理obj模型会遇到的各种问题，例如你想要的是三角网格，但网上下载了一个四角网格的模型，可以拿Rhino直接一键改（自己傻乎乎地写了一个复杂度巨高的change.py）。而且Rhino还能算顶点法向量、能补全空缺网格、能随心所欲简化网格数目……。它真的，我哭死。

最终效果之一（因为使用了ppm，所以焦散效果很好；水面模型是自己拿随机数生成一个obj三角网格，然后法向插值了一波就很光滑啦）：

![](https://s3.bmp.ovh/imgs/2022/07/07/3e498fabad04043c.bmp)
