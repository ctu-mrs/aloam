cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(aloam_slam VERSION 666)

# we really need to replace the general flags because
# we cannot build this with march=native
set(CMAKE_CXX_FLAGS "")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_definitions(-g)
add_definitions(-O3)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
  mrs_lib
  mrs_msgs
  eigen_conversions
  tf_conversions
  tf2_eigen
  nodelet
  pcl_ros
  ouster_ros
)

find_package(PCL REQUIRED COMPONENTS)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs mrs_lib mrs_msgs eigen_conversions tf_conversions pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(
  ${PCL_DEFINITIONS}
)

add_library(AloamSlam src/aloam.cpp src/odometry.cpp src/mapping.cpp src/feature_extractor.cpp)

add_dependencies(AloamSlam
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(AloamSlam
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${CERES_LIBRARIES}
  )
