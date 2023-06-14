#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

class PointCloudPublisher
{
public:
    PointCloudPublisher(ros::NodeHandle& nh, std::string topic) : nh_(nh)
    {
        // 创建ROS发布器
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic, 1);
    }

    void Publish(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        // 填充ROS消息头部信息
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "map";

        // 发布点云消息
        pub_.publish(cloud_msg);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_visualization_node");
    ros::NodeHandle nh;

    // 创建点云发布者对象
    PointCloudPublisher source_pub(nh, "/source_cloud");
    PointCloudPublisher filtered_pub(nh, "/filtered_cloud");
    PointCloudPublisher aligned_pub(nh, "/aligned_cloud");

    // 读取源点云并发布
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/gh_luck/catkin_ws/src/cloud_registration/resources/map_cloud_1.pcd", *source_cloud);

    while (ros::ok())
    {
        source_pub.Publish(source_cloud);

        // 对源点云进行下采样并发布
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setInputCloud(source_cloud);
        filter.setLeafSize(0.1f, 0.1f, 0.1f);
        filter.filter(*filtered_cloud);
        filtered_pub.Publish(filtered_cloud);

        // 对滤波后的点云进行配准，再发布
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(filtered_cloud);
        icp.setInputTarget(source_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*aligned_cloud);

        aligned_pub.Publish(aligned_cloud);

        ros::spinOnce(); // 保持ROS运行
    }
    return 0;
    
}
