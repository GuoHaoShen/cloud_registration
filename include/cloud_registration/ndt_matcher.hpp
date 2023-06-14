#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

struct NDTParameters
{
    /* data */
    int max_iterations;   //  最大迭代次数
    double transformation_epsilon;  // 变换矩阵更新阈值
    double StepSize;  // 步长
    double Resolution; // 分辨率

    NDTParameters()
        : max_iterations(10),
          transformation_epsilon(0.1),
          StepSize(1.0),
          Resolution(0.01) {}
};

// 定义 NDTMatcher 类，继承自 pcl::NormalDistributionsTransform
class NDTMatcher : public pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> {
public:
    inline explicit NDTMatcher()
        : pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>(){}

    // 更新 NDT 算法参数函数
    inline void updateParams(NDTParameters& params) {
        setMaximumIterations(params.max_iterations);
        setStepSize(params.StepSize);
        setTransformationEpsilon(params.transformation_epsilon);
        setResolution(params.Resolution);
    }  

    // 执行 NDT 匹配操作，并返回匹配后的点云指针
    inline void match(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source_cloud,
                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud) {
        setInputSource(source_cloud);
        setInputTarget(target_cloud);
        align(*output_cloud);
    }
};
