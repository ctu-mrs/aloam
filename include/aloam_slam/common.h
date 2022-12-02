// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <math.h>
#include <cmath>
#include <vector>
#include <string>
#include <thread>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <queue>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <ceres/ceres.h>
/* #include <opencv/cv.h> */

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <std_msgs/String.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/PointCloud2.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_msgs/Float64ArrayStamped.h>
#include <mrs_msgs/PclToolsDiagnostics.h>

#include <pcl/point_types.h>

#include <feature_extraction/lidar_extraction_edge_plane.h>

typedef ouster_ros::Point PointTypeOS;
typedef pcl::PointXYZI    PointType;

inline double rad2deg(double radians) {
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees) {
  return degrees * M_PI / 180.0;
}

struct CommonHandlers_t
{
  ros::NodeHandle                       nh;
  std::shared_ptr<mrs_lib::ParamLoader> param_loader;
  std::shared_ptr<mrs_lib::Profiler>    profiler;

  bool offline_run = false;

  std::string uav_name;
  std::string frame_fcu;
  std::string frame_lidar;
  std::string frame_odom;
  std::string frame_map;

  tf::Transform tf_lidar_in_fcu_frame;

  // Sensor parameters
  int   scan_lines;
  int   samples_per_row;
  float vfov;
  float frequency;

  bool                                       enable_scope_timer;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger = nullptr;
};
