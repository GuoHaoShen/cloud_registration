#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

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