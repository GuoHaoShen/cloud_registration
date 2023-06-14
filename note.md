## 点云配准笔记

### icp匹配参数

ICP 算法（迭代最近点算法）是 PCL 库中的一个重要功能，它用于将两个3D点云间的相对姿态对齐。ICP 算法涉及许多参数，这些参数可以调整来控制算法行为和输出结果的质量。

下面我们将详细介绍 PCL 中 ICP 相关的常见参数及其含义：


setMaxCorrespondenceDistance - 最大匹配距离
要将两个点云进行匹配需要找到相关的点对。max_correspondence_distance 定义了在距离空间中查找最近邻居的最大距离。默认情况下，该值是 0.1 米。

应根据所提供数据的大小和性质合理设置此参数。如果点云数据集较稀疏，则可能需要增加此参数以查找更远的邻居。

setTransformationEpsilon - 收敛阈值
该参数定义了算法收敛时变换矩阵的最大变化量或误差阈值。默认情况下，阈值设置为 1e-6。

当算法迭代次数过多或点云间的平移/旋转较小时，建议适当调整此参数来获得更好的结果。

setMaximumIterations - 最大迭代次数
ICP 算法是一个基于迭代优化的算法，因此需要设置最大迭代次数以限制循环次数并避免死循环。通常，该值由用户自行设置。默认情况下， icp.setMaxiterations (100)。

调整此参数可以确保在不超过计算资源预算的情况下实现最佳匹配效果。

setTransformationCalculation - 转换计算方式
该参数定义了如何计算点对之间的距离。默认情况下，使用 pcl::registration::TransformationEstimationSVD 计算方法（一种可靠但运算速度较慢的方法）。如果你需要更快的运算速度，则可以使用另一些较新的转换估计方法，如 FPCS 算法，摄影测量学算法等。

setEuclideanFitnessEpsilon - 模型收敛阈值
该参数定义了算法的收敛标准。当均方误差小于这个值时，可以认为 ICP 匹配是有效的。默认情况下，该值设置为 1e-5。

建议仅在计算机资源中限制时使用此参数进行微调。

以上是 PCL 中 ICP 相关的常见参数。通过合理选择并调整这些参数，可以获得最佳的匹配效果。


该算法有几个终止标准：

    迭代次数已达到用户施加的最大迭代次数(通过 setMaximumIterations)
    上一个转换和当前估计转换之间的 epsilon（差值）小于用户强加的值(通过 setTransformationEpsilon)
    欧几里得平方误差的总和小于用户定义的阈值(通过 setEuclideanFitnessEpsilon)


### ndt参数
NDT（Normal Distribution Transform）是一种基于点云配准的方法，是 PCL 中著名的算法之一。NDT 算法建立在概率分布变换的基础上，用均值方差等参数描述点云的局部特征，从而达到高精度的配准效果。下面我来介绍一下 NDT 配准算法中的几个关键参数以及如何设置这些参数。

pcl::NormalDistributionsTransform::setStepSize(double step_size)
setStepSize() 函数表示 NDT 戳使用的搜索步长。实际上就是在坐标轴上设计一个单位步进数值。接受 double 类型的数据，其默认值为 0.1。


pcl::NormalDistributionsTransform::setResolution(double resolution)
NDT 最近邻搜索的体素大小（voxel size）。接受 double 类型的数据，其默认值为 1.0。此参数代表了匹配时候的精度，需要根据场景进行调整。


pcl::NormalDistributionsTransform::setTransformationEpsilon(double epsilon)
在最小均方误差（MSE）达到某一阈值后可以终止迭代。设置 epsilon 可以控制精度。接受 double 类型的数据，其默认值为 0.01。


pcl::NormalDistributionsTransform::setMaximumIterations(int max_iter)
设置最大的配准迭代次数。避免无限迭代而导致系统性能问题。接受 int 类型的数据，其默认值为 35。


### sacia参数

#### 法线计算
pcl::NormalEstimation是PCL中的一个类，用于计算点云中每个点的法向量信息。该算法会对输入的点云数据进行搜索，并在每一个点周围估算法向量。

除此之外，NormalEstimation还有其他一些方法和设置参数：

* setInputCloud：设置输入点云
* setSearchMethod：设置搜索方式（例如KdTree或Octree）
* setRadiusSearch：设置搜索半径
* setKSearch：设置最近邻点的数目。请注意，如果同时使用了 setRadiusSearch 方法，则将被忽略。
* compute：执行法向量计算

因为NormalEstimation提取局部特征，所以需要将搜索半径或最近邻点（k）设置为相对合适的大小才能得到正确的结果。不同应用程序可能需要不同的观察特征大小。

默认值
* kSearch或radiusSearch：0.03米
* setEpsAngle：15度
* setMaxDepthChangeFactor：0.02
* setNormalSmoothingSize：10点（k-search），或者为半径0.05米（radius-search）
* setSearchMethod：KdTree

调参建议
setSearchPoint：类型为 PointCloud<PointXYZ> 普及 Pointer，表示输入云点;

setSearchMethod: Kdtree或octree查找方法(Pointer)

设置KdTreeSP或OcTreeSP有效, 通过setRadiusSearch()控制用于估计查询点法线的球体半径
setNormalsOutput指定它是否同时生成法线和曲率. ;

越大的范围和更多的邻居可以产生更好的法线估计，但代价是性能的巨大下降，并且在噪声数据中可能会导致错误估计。所以，在实际使用的时候还要根据你的点云数据进行调整。

#### FPFH特征
pcl::FPFHEstimation 是PCL中的一个类，常用于描述特定点周围的表面几何形状，特别是在三维目标识别和场景重建中非常有用。该算法计算点云中每个点的快速点特征直方图（FPFH）信息

除此之外，也有其他一些方法和设置参数：

* setInputCloud：设置输入点云
* setInputNormals：设置输入点法向量
* setSearchMethod：设置搜索方式
* setRadiusSearch：设置搜索半径
* setKSearch：设置最近邻数目
* compute：执行FPFH计算
与NormalEstimation类似，FPFH也需要调整一些参数以获取最佳性能和效果。

调参建议
FPFH的参数设置通常需要根据具体应用场景进行调整。以下是PCL中建议使用的一些默认值：

* setSearchMethod：KdTree
* setRadiusSearch：0.05
* setNormalSearchRadius：setRadiusSearch 的两倍
* setNumberOfThreads: NCPU

在实际使用中，可以将这些参数设置为适合您的数据集的大小和复杂度的任何值。有两个主要的参数需要注意调整：

搜索半径（或最近邻数目）：与NormalEstimation类似，FPFH也依赖于搜索半径或最近邻数，以获得最精确的结果。通常情况下，应将其设置为相对较小的值，以克服数据过于稠密的问题，但还不至于遗漏重要信息。

参考点数目：PCL的官方文档指出，为了平衡准确性和计算成本，参考点数目(即每个点选择多少个最近邻作为参考) 应该设置为大约50%左右的输入最近邻数量(默认为30)。