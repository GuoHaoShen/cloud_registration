cmake_minimum_required(VERSION 3.0.2)
project(cloud_registration)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  dynamic_reconfigure
)


generate_dynamic_reconfigure_options(cfg/MatcherParams.cfg) # 添加动态参数配置文件

# 寻找PCL的库
find_package(PCL REQUIRED COMPONENT common io visualization filters)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp dynamic_reconfigure
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "/usr/include/eigen3"
  include  # 添加自己的头文件目录
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)


# point_cloud_visualization_node
add_executable(point_cloud_visualization_node 
              src/point_cloud_visualization.cpp)

target_link_libraries(point_cloud_visualization_node 
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
)

# add_executable(cloud_registration_node src/cloud_registration.cpp)

# target_link_libraries(cloud_registration_node 
#     ${catkin_LIBRARIES} 
#     ${PCL_LIBRARIES}
# )

add_executable(point_cloud_matcher src/point_cloud_matcher.cpp)

target_link_libraries(point_cloud_matcher 
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
)

