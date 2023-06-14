#include <chrono>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <dynamic_reconfigure/server.h>
#include <cloud_registration/MatcherParamsConfig.h>
#include <pcl/common/transforms.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "cloud_registration/point_cloud_publisher.hpp"
#include "cloud_registration/icp_matcher.hpp"
#include "cloud_registration/ndt_matcher.hpp"
#include "cloud_registration/sacia_matcher.hpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);

// 创建 ICPMatcher 对象
ICPMatcher icp_matcher;
NDTMatcher ndt_matcher;
SACIAMatcher sacia_matcher;


// 初始变换参数
std::string registration_type;
float sourceTranfrom_x;
float sourceTranfrom_y;
float sourceTranfrom_yaw;

void configCallback(point_cloud_matcher::MatcherParamsConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request:");
    ROS_INFO("-> maxCorrespondenceDistance: %f", config.icp_maxCorrespondenceDistance);
    ROS_INFO("-> transformationEpsilon: %f", config.icp_transformationEpsilon);
    ROS_INFO("-> euclideanFitnessEpsilon: %f", config.icp_euclideanFitnessEpsilon);
    ROS_INFO("-> maxIterations: %d", config.icp_maxIterations);

    registration_type = config.registration_type;
    sourceTranfrom_x = config.sourceTranfrom_x;
    sourceTranfrom_y = config.sourceTranfrom_y;
    sourceTranfrom_yaw = config.sourceTranfrom_yaw;

    // 定义 ICP 参数结构体
    ICPParameters icp_params;
    icp_params.max_iterations = config.icp_maxIterations;
    icp_params.transformation_epsilon = config.icp_transformationEpsilon;
    icp_params.euclideanFitness_Epsilon = config.icp_euclideanFitnessEpsilon;
    icp_params.max_Correspondence_Distance = config.icp_maxCorrespondenceDistance;

    // 更新 ICP 参数
    icp_matcher.updateParams(icp_params);

    // 定义 NDT 参数结构体
    NDTParameters ndt_params;
    ndt_params.max_iterations = config.ndt_maxIterations;
    ndt_params.transformation_epsilon =config.ndt_transformationEpsilon;
    ndt_params.StepSize = config.ndt_StepSize;
    ndt_params.Resolution = config.ndt_Resolution;

    // 更新 NDT 参数
    ndt_matcher.updateParams(ndt_params);

    // 定义 SACIA 参数结构体
    SACIAParameters sacia_params;
    sacia_params.max_iterations = config.sacia_maxIterations;
    sacia_params.max_Correspondence_Distance = config.sacia_maxCorrespondenceDistance;
    sacia_params.MinSampleDistance = config.sacia_MinSampleDistance;
    sacia_params.RANSACRefinementDistance = config.sacia_RANSACRefinementDistance;
    sacia_params.NoemalRadiusSearch = config.sacia_NoemalRadiusSearch;
    sacia_params.FPFHRadiusSearch = config.sacia_FPFHRadiusSearch;

    // 更新 SACIA 参数
    sacia_matcher.updateParams(sacia_params);

}

void transformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,  
                        float trans_x, float trans_y, float rot_yaw)
{
    // 定义变换矩阵
    Eigen::Affine3f transformation =
        Eigen::Translation3f(Eigen::Vector3f(trans_x, trans_y, 0.0)) *
        Eigen::AngleAxisf(rot_yaw, Eigen::Vector3f::UnitZ());

    // 对每个点云进行变换
    pcl::transformPointCloud(*cloud_in, *cloud_out, transformation);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "point_cloud_matcher");
    ros::NodeHandle nh;

    // 创建点云发布者对象
    PointCloudPublisher source_pub(nh, "/source_cloud");
    PointCloudPublisher target_pub(nh, "/target_cloud");
    PointCloudPublisher aligned_pub(nh, "/aligned_cloud");

    // Load input file
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gh_luck/catkin_ws/src/cloud_registration/resources/map_cloud_2.pcd", *cloud_target) == -1)
    {
        ROS_ERROR("Could not read input cloud1 file");
        return (-1);
    }
    std::cout << "Loaded point cloud1 with " << cloud_target->size() << " points" << std::endl;

    // Load output file
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/gh_luck/catkin_ws/src/cloud_registration/resources/scan_cloud_2.pcd", *cloud_source) == -1)
    {
        ROS_ERROR("Could not read input cloud2 file");
        return (-1);
    }
    std::cout << "Loaded point cloud2 with " << cloud_source->size() << " points" << std::endl;

    // Set up dynamic reconfiguration callback
    dynamic_reconfigure::Server<point_cloud_matcher::MatcherParamsConfig> server;
    dynamic_reconfigure::Server<point_cloud_matcher::MatcherParamsConfig>::CallbackType f;
    f = boost::bind(&configCallback, _1, _2);
    server.setCallback(f);

    ros::Rate loop_rate(10); // 执行频率为 10H

    while (ros::ok())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        transformPointCloud(cloud_source, cloud_source_transformed, 
                            sourceTranfrom_x, sourceTranfrom_y, sourceTranfrom_yaw);

        ROS_INFO_STREAM("Match Started...");
        auto start_time = std::chrono::high_resolution_clock::now(); // 获取开始时间

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZ>);
        if(registration_type == "ICP")
        {
            // 执行匹配操作，并得到匹配后的点云结果
            icp_matcher.match(cloud_source_transformed, cloud_target, cloud_output);   
        }
        else if (registration_type == "NDT")
        {
            ndt_matcher.match(cloud_source_transformed, cloud_target, cloud_output);   
        }
        else if (registration_type == "SACIA")
        {
            sacia_matcher.match(cloud_source_transformed, cloud_target, cloud_output);   
        }
        else{
            ROS_INFO_STREAM("unknown registration_type , please retry!");
        }

        ROS_INFO_STREAM("Match Finished...");
        auto end_time = std::chrono::high_resolution_clock::now();   // 获取结束时间
        auto duration_sec = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);   // 计算耗时（秒）
        ROS_INFO_STREAM("Matcher takes " << duration_sec.count() << " seconds or ");

        // 发布结果进行可视化
        source_pub.Publish(cloud_source_transformed);
        target_pub.Publish(cloud_target);
        aligned_pub.Publish(cloud_output);

        loop_rate.sleep();  // 控制过程执行时间，保持固定的循环频率
        ros::spinOnce();
    }

    return (0);
}
