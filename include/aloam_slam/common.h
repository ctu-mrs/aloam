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
#define PCL_NO_PRECOMPILE  // !! BEFORE ANY PCL INCLUDE!!

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <feature_selection/data_structures.h>
#include <feature_selection/feature_selection.h>

#include <feature_extraction/lidar_extraction_edge_plane.h>

#include <math.h>
#include <cmath>
#include <thread>
#include <iostream>
#include <mutex>
#include <condition_variable>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <ceres/ceres.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/transformer.h>
#include <mrs_msgs/Float64ArrayStamped.h>
#include <mrs_msgs/PclToolsDiagnostics.h>

typedef pcl::PointXYZI pt_I_t;

struct CommonHandlers_t
{
  ros::NodeHandle                       nh;
  std::shared_ptr<mrs_lib::ParamLoader> param_loader;
  std::shared_ptr<mrs_lib::Profiler>    profiler;

  std::shared_ptr<feature_selection::FeatureSelection> feature_selection;

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

  // Scope timer
  bool                                       enable_scope_timer;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger = nullptr;

  // Feature extraction
  bool overwrite_intensity_with_curvature = false;
};

/*//{ cloudPointOStoCloudPoint() */
inline pcl::PointCloud<pt_I_t>::Ptr cloudPointOStoCloudPoint(const PC_ptr cloud_unordered_in, const feature_selection::IndicesPtr_t indices,
                                                             const size_t cloud_width, const float scan_period_sec) {

  if (!cloud_unordered_in || !indices) {
    ROS_ERROR("[cloudPointOStoCloudPoint]: Null pointer to input cloud or input indices given, returning nullptr.");
    return nullptr;
  } else if (!cloud_unordered_in->is_dense) {
    ROS_ERROR("[cloudPointOStoCloudPoint]: Input cloud is NOT dense (is organized), returning nullptr.");
    return nullptr;
  } else if (cloud_unordered_in->size() != indices->size()) {
    ROS_ERROR("[cloudPointOStoCloudPoint]: Size of input dense cloud and respective point indices do not match, returning nullptr.");
    return nullptr;
  }

  const float cloud_width_flt = cloud_width;

  const pcl::PointCloud<pt_I_t>::Ptr cloud_out = boost::make_shared<pcl::PointCloud<pt_I_t>>();
  cloud_out->header                            = cloud_unordered_in->header;
  cloud_out->width                             = cloud_unordered_in->width;
  cloud_out->height                            = 1;
  cloud_out->is_dense                          = true;

  cloud_out->reserve(cloud_out->width);
  for (size_t i = 0; i < cloud_unordered_in->size(); i++) {
    const auto &point_os = cloud_unordered_in->at(i);
    const auto &idx      = indices->at(i);

    pt_I_t point;
    point.x         = point_os.x;
    point.y         = point_os.y;
    point.z         = point_os.z;
    point.intensity = float(point_os.ring) + scan_period_sec * (float(idx.col) / cloud_width_flt);

    cloud_out->push_back(point);
  }

  return cloud_out;
}
/*//}*/

/*//{ publishCloud() */

inline void publishCloud(const ros::Publisher &pub, const feature_selection::FSCloudManagerPtr cloud_manager) {
  if (cloud_manager && pub.getNumSubscribers() > 0) {

    const auto cloud = cloud_manager->getUnorderedCloudPtr();

    const sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloud, *msg);

    try {
      pub.publish(msg);
    }
    catch (...) {
    }
  }
}

inline void publishCloud(const ros::Publisher &pub, const PC_ptr cloud, const feature_extraction::indices_ptr_t indices) {
  if (cloud && pub.getNumSubscribers() > 0) {

    const auto manager = std::make_shared<feature_selection::FSCloudManager>(cloud, indices);
    publishCloud(pub, manager);
  }
}

/*//}*/
