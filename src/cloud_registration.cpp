// #include <iostream>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/registration/icp.h>
// #include <pcl/registration/ndt.h>
// #include <pcl/registration/ia_ransac.h>
// #include <pcl/visualization/pcl_visualizer.h>

// using namespace std;
// using namespace pcl;

// PointCloud<PointXYZ>::Ptr LoadPointCloud(string path)
// {
//     PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
//     if(io::loadPCDFile(path, *cloud) == -1)
//     {
//         cerr << "Failed to read PCD file: " << path << endl;
//         exit(-1);
//     }
//     cout << "Successfully loaded point cloud: " << path << endl;
//     return cloud;
// }

// void FilterPointCloud(PointCloud<PointXYZ>::Ptr& cloud, float voxel_size)
// {
//     VoxelGrid<PointXYZ> filter;
//     filter.setLeafSize(voxel_size, voxel_size, voxel_size);
//     filter.setInputCloud(cloud);
//     PointCloud<PointXYZ>::Ptr filtered_cloud(new PointCloud<PointXYZ>);
//     filter.filter(*filtered_cloud);
//     cloud = filtered_cloud;
//     cout << "Point cloud downsampled with a voxel size of " << voxel_size << endl;
// }

// void IcpRegistration(PointCloud<PointXYZ>::Ptr& source_cloud, PointCloud<PointXYZ>::Ptr& target_cloud, float icp_epsilon, int icp_max_iterations)
// {
//     IterativeClosestPoint<PointXYZ, PointXYZ> icp;
//     icp.setTransformationEpsilon(icp_epsilon);
//     icp.setMaximumIterations(icp_max_iterations);
//     icp.setInputCloud(source_cloud);
//     icp.setInputTarget(target_cloud);
//     PointCloud<PointXYZ>::Ptr registered_cloud(new PointCloud<PointXYZ>);
//     icp.align(*registered_cloud);
//     source_cloud = registered_cloud;
//     cout << "ICP registration finished with a fitness score of " << icp.getFitnessScore() << endl;
// }

// void NdtRegistration(PointCloud<PointXYZ>::Ptr& source_cloud, PointCloud<PointXYZ>::Ptr& target_cloud, float ndt_epsilon, float ndt_resolution)
// {
//     NormalDistributionsTransform<PointXYZ, PointXYZ> ndt;
//     ndt.setTransformationEpsilon(ndt_epsilon);
//     ndt.setResolution(ndt_resolution);
//     ndt.setInputCloud(source_cloud);
//     ndt.setInputTarget(target_cloud);
//     PointCloud<PointXYZ>::Ptr registered_cloud(new PointCloud<PointXYZ>);
//     ndt.align(*registered_cloud);
//     source_cloud = registered_cloud;
//     cout << "NDT registration finished with a fitness score of " << ndt.getFitnessScore() << endl;
// }

// void ScaIaRegistration(PointCloud<PointXYZ>::Ptr& source_cloud, PointCloud<PointXYZ>::Ptr& target_cloud, float scaia_min_sample_distance, float scaia_max_correspondence_distance, int scaia_maximum_iterations)
// {
//     SampleConsensusInitialAlignment<PointXYZ, PointXYZ, Normal> sac_ia;
//     sac_ia.setMinSampleDistance(scaia_min_sample_distance);
//     sac_ia.setMaxCorrespondenceDistance(scaia_max_correspondence_distance);
//     sac_ia.setMaximumIterations(scaia_maximum_iterations);
//     sac_ia.setInputTarget(target_cloud);
//     sac_ia.setSourceNormals(pcl::NormalEstimation<PointXYZ, pcl::Normal>::Ptr(new pcl::NormalEstimation<PointXYZ, pcl::Normal>));
//     sac_ia.setInputSource(source_cloud);
//     PointCloud<PointXYZ>::Ptr registered_cloud(new PointCloud<PointXYZ>);
//     sac_ia.align(*registered_cloud);
//     source_cloud = registered_cloud;
//     cout << "SCA-IA registration finished with a fitness score of " << sac_ia.getFitnessScore() << endl;
// }

// void VisualizePointClouds(PointCloud<PointXYZ>::Ptr& source_cloud, PointCloud<PointXYZ>::Ptr& target_cloud)
// {
//     visualization::PCLVisualizer viewer("3D Viewer");
//     viewer.setBackgroundColor(0.0, 0.0, 0.0);

//     visualization::PointCloudColorHandlerCustom<PointXYZ> source_color(source_cloud, 255, 0, 0);
//     viewer.addPointCloud<PointXYZ>(source_cloud, source_color, "source cloud");

//     visualization::PointCloudColorHandlerCustom<PointXYZ> target_color(target_cloud, 0, 255, 0);
//     viewer.addPointCloud<PointXYZ>(target_cloud, target_color, "target cloud");

//     while(!viewer.wasStopped())
//     {
//         viewer.spinOnce(100);
//         boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//     }
// }

// int main()
// {
//     // 加载点云
//     PointCloud<PointXYZ>::Ptr source_cloud = LoadPointCloud("source.pcd");
//     PointCloud<PointXYZ>::Ptr target_cloud = LoadPointCloud("target.pcd");

//     // 下采样
//     FilterPointCloud(source_cloud, 0.01);     // 注意这里的采样步长需要根据点云数据进行调整

//     // ICP 配准
//     IcpRegistration(source_cloud, target_cloud, 1e-8f, 150);

//     // NDT 配准
//     NdtRegistration(source_cloud, target_cloud, 1e-6f, 0.01f);

//     // SCA-IA 配准
//     ScaIaRegistration(source_cloud, target_cloud, 0.05f, 5.0f, 50);

//     // 可视化点云
//     VisualizePointClouds(source_cloud, target_cloud);
// }
