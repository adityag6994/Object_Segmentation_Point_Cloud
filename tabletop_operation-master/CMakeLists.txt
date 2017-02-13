cmake_minimum_required(VERSION 2.8.3)
project(tabletop_operation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  pcl_conversions
  pcl_ros
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tabletop_operation
#  CATKIN_DEPENDS genmsg pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cluster_euclid src/cluster_euclid.cpp)
target_link_libraries(cluster_euclid ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(planar src/planar.cpp)
target_link_libraries(planar ${catkin_LIBRARIES} ${PCL_LIBRARIES})
