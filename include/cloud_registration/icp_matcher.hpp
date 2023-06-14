#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

struct ICPParameters
{
    /* data */
    int max_iterations;   // ICP 最大迭代次数
    double transformation_epsilon;  // 变换矩阵更新阈值
    double euclideanFitness_Epsilon;  // 欧氏距离差 epsilon阈值
    double max_Correspondence_Distance; //最大匹配距离

    ICPParameters()
        : max_iterations(100),
          transformation_epsilon(1e-8),
          euclideanFitness_Epsilon(1e-5),
          max_Correspondence_Distance(5) {}
};

// 定义 ICPMatcher 类，继承自 pcl::IterativeClosestPoint
class ICPMatcher : public pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> {
public:
    inline explicit ICPMatcher()
        : pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>(){}

    // 更新 ICP 算法参数函数
    inline void updateParams(ICPParameters& params) {
        setMaxCorrespondenceDistance(params.max_Correspondence_Distance);
        setTransformationEpsilon(params.transformation_epsilon);
        setEuclideanFitnessEpsilon(params.euclideanFitness_Epsilon);
        setMaximumIterations(params.max_iterations);
    }

    // 执行 ICP 匹配操作，并返回匹配后的点云指针
    inline void match(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& source_cloud,
                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& target_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud) {
        setInputSource(source_cloud);
        setInputTarget(target_cloud);
        align(*output_cloud);
        std::cout << " score: " << getFitnessScore() << std::endl;
    }

};
