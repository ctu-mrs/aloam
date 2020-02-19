cmake_minimum_required(VERSION 2.8.3)
project(aloam_velodyne)


set(CMAKE_CXX_FLAGS "-g -O0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DDISABLE_PARALLELIZATION -DDEBUG_PARAMS")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DDISABLE_PARALLELIZATION -DDEBUG_PARAMS")
set(CMAKE_CXX_FLAGS_RELEASE "-Werror -O3 -g -O0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DDISABLE_PARALLELIZATION -DDEBUG_PARAMS")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -O3")
# set(CMAKE_CXX_FLAGS "-g -Og -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DDISABLE_PARALLELIZATION -DDEBUG_PARAMS")
# set(CMAKE_CXX_FLAGS_DEBUG "-g -Og -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DDISABLE_PARALLELIZATION -DDEBUG_PARAMS")
# set(CMAKE_CXX_FLAGS_RELEASE "-g -Og -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DDISABLE_PARALLELIZATION -DDEBUG_PARAMS")
# set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -O0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN -DDISABLE_PARALLELIZATION -DDEBUG_PARAMS")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options("-Wall")
add_compile_options("-Wextra")

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
  eigen_conversions
  tf_conversions
)

find_package(PCL REQUIRED COMPONENTS)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs mrs_lib eigen_conversions tf_conversions
  LIBRARIES aloamOdometry aloamMapping
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
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

# add_executable(ascanRegistration src/scanRegistration.cpp)
# target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(aloamOdometry src/laserOdometry.cpp src/lidarFactor.hpp)
target_link_libraries(aloamOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES} ${CERES_LIBRARIES})
add_dependencies(aloamOdometry
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(aloamOdometry SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})

# add_executable(alaserMapping src/laserMapping.cpp)
# target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

