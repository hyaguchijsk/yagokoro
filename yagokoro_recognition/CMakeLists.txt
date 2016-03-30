cmake_minimum_required(VERSION 2.8.3)
project(yagokoro_recognition)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport std_msgs sensor_msgs geometry_msgs visualization_msgs tf message_generation)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS roscpp cv_bridge image_transport std_msgs sensor_msgs geometry_msgs visualization_msgs tf
   DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(feature_extraction src/feature_extraction.cpp)
target_link_libraries(feature_extraction ${catkin_LIBRARIES})
