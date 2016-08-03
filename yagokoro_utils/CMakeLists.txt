cmake_minimum_required(VERSION 2.8.3)
project(yagokoro_utils)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure cv_bridge image_transport pcl_ros std_msgs sensor_msgs geometry_msgs visualization_msgs tf message_generation)

find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/PointCloudToImage.cfg
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS roscpp dynamic_reconfigure cv_bridge image_transport pcl_ros std_msgs sensor_msgs geometry_msgs visualization_msgs tf
   DEPENDS pcl
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

add_executable(pointcloud_to_image_node src/pointcloud_to_image_node.cpp)
target_link_libraries(pointcloud_to_image_node ${catkin_LIBRARIES})
add_dependencies(pointcloud_to_image_node ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
