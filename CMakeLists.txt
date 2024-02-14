cmake_minimum_required(VERSION 2.8.3)
# cmake_policy(SET CMP0048 NEW)
project(aloam_slam)

# Override CXX flags inherited from workspace, if precompiled PCL binaries from debian repos are used
# if (DEFINED ENV{PCL_CROSS_COMPILATION})
#   set(PCL_CROSS_COMPILATION $ENV{PCL_CROSS_COMPILATION})
# else()
#   set(PCL_CROSS_COMPILATION "false")
# endif()
# if(${PCL_CROSS_COMPILATION} STREQUAL "false")
#   message("Using precompiled PCL binaries from debian repos. Overriding catkin workspace CMAKE_CXX_FLAGS.")
#   set(CMAKE_CXX_FLAGS "-std=c++17")
# else()
#   message("Using custom-built PCL binaries. Inheriting all CMAKE_CXX_FLAGS from catkin workspace.")
# endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# add_definitions(-Werror)
# add_definitions(-Wall)
# add_definitions(-Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  tf
  mrs_lib
  mrs_msgs
  mrs_modules_msgs
  eigen_conversions
  tf_conversions
  tf2_eigen
  nodelet
  pcl_ros
  pcl_conversions
  pcl_msgs
  ouster_ros
  feature_extraction
  feature_selection
  )

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

add_message_files(DIRECTORY msg FILES
  AloamDiagnostics.msg
  OdometryDiagnostics.msg
  MappingDiagnostics.msg
  Signature.msg
  FeaturesSignature.msg
)

generate_messages(DEPENDENCIES
  geometry_msgs nav_msgs mrs_msgs mrs_modules_msgs pcl_msgs feature_extraction feature_selection ouster_ros sensor_msgs
)

set(LIBRARIES
  AloamSlam AloamSlamRosbag
  )

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs nav_msgs roscpp rospy mrs_lib mrs_msgs mrs_modules_msgs eigen_conversions tf_conversions pcl_ros pcl_conversions pcl_msgs feature_extraction feature_selection ouster_ros
  LIBRARIES ${LIBRARIES}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

link_directories(
  ${PCL_LIBRARY_DIRS}
  )

add_definitions(
  ${PCL_DEFINITIONS}
  )

add_library(AloamSlam
  src/aloam.cpp
  src/odometry.cpp
  src/mapping.cpp
  src/feature_extractor.cpp
  )

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

add_library(AloamSlamRosbag
  src/aloam_rosbag.cpp
  src/odometry.cpp
  src/mapping.cpp
  src/feature_extractor.cpp
)
add_dependencies(AloamSlamRosbag
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(AloamSlamRosbag
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${CERES_LIBRARIES}
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
