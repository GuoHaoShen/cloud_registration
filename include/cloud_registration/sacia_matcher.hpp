#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/fpfh.h>          // 使用快速点特征直方图 (FPFH) 特征
#include <pcl/features/normal_3d.h>

struct SACIAParameters
{
    /* data */
    int max_iterations;   // SACIA 最大迭代次数
    double max_Correspondence_Distance;  // 变换矩阵更新阈值
    double MinSampleDistance;  // 最小样本距离
    double RANSACRefinementDistance; // RANSAC 优化距离
    double NoemalRadiusSearch;  // 法线计算搜索半径
    double FPFHRadiusSearch;    // FPFH特征计算搜索半径

    SACIAParameters()
        : max_iterations(10),
          max_Correspondence_Distance(0.1),
          MinSampleDistance(0.05),
          RANSACRefinementDistance(0.02),
          NoemalRadiusSearch(0.05),
          FPFHRadiusSearch(0.1) {}
};

// 定义 SACIAMatcher 类，继承自 pcl::SampleConsensusInitialAlignment
class SACIAMatcher : public pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> {
public:
    inline explicit SACIAMatcher()
        : pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33>(){}

    // 更新 SACIA 算法参数函数
    inline void updateParams(SACIAParameters& params) {
        setMaximumIterations(params.max_iterations);
        setMinSampleDistance(params.MinSampleDistance);
        setMaxCorrespondenceDistance(params.max_Correspondence_Distance);
        // setRANSACRefinementDistance(params.RANSACRefinementDistance);

        normal_estimator_.setRadiusSearch(params.NoemalRadiusSearch);
        fpfh_.setRadiusSearch(params.FPFHRadiusSearch);
    }  

    // 执行 SACIA 匹配操作，并返回匹配后的点云指针
    inline void match(const PointCloud::Ptr& source_cloud,
                      const PointCloud::Ptr& target_cloud,
                            PointCloud::Ptr& output_cloud) {

        FeatureCloud::Ptr source_fpfh_features = calculateFPFHSimple(source_cloud);
        FeatureCloud::Ptr target_fpfh_features = calculateFPFHSimple(target_cloud);

        setInputSource(source_cloud);
        setSourceFeatures(source_fpfh_features);      // 设置源点云的 FPFH 特征
        setInputTarget(target_cloud);
        setTargetFeatures(target_fpfh_features);      // 设置目标点云 FPFH 特征
        align(*output_cloud);
        std::cout << " score: " << getFitnessScore() << std::endl;
    }

private:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> NormalCloud;
    typedef pcl::PointCloud<pcl::FPFHSignature33> FeatureCloud;

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator_;
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_;

FeatureCloud::Ptr calculateFPFHSimple(const PointCloud::Ptr &cloud) {
    // 计算法线
    NormalCloud::Ptr normals(new NormalCloud);
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);
    normal_estimator_.setSearchMethod(tree);
    normal_estimator_.setInputCloud(cloud);
    // normal_estimator_.setRadiusSearch(0.05);
    normal_estimator_.compute(*normals);

    // 计算FPFH特征
    FeatureCloud::Ptr fpfhs(new FeatureCloud);
    // pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh_.setInputCloud(cloud);
    fpfh_.setInputNormals(normals);
    fpfh_.setSearchMethod(tree);
    // fpfh_.setRadiusSearch(0.1);
    fpfh_.compute(*fpfhs);
    
    return fpfhs;
}
};
