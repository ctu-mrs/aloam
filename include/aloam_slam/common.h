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

  /* std::shared_ptr<feature_selection::FeatureSelection> feature_selection; */

  bool offline_run = false;

  std::string uav_name;
  std::string frame_fcu;
  std::string frame_lidar;
  std::string frame_odom;
  std::string frame_map;

  tf::Transform tf_lidar_in_fcu_frame;

  // Sensor parameters
  int   samples_per_row;
  float frequency;

  // Scope timer
  bool                                       enable_scope_timer;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger = nullptr;

  // Feature extraction
  bool overwrite_intensity_with_curvature = false;
};

/*//{ cloudPointTtoCloudIPoint() */
template <typename T_pt>
inline pcl::PointCloud<pt_I_t>::Ptr cloudPointTtoCloudIPoint(const boost::shared_ptr<pcl::PointCloud<T_pt>> cloud_in,
                                                             const feature_selection::IndicesPtr_t indices, const float scan_period_sec) {

  if (!cloud_in || !indices) {
    ROS_ERROR("[cloudPointTtoCloudIPoint]: Null pointer to input cloud or input indices given, returning nullptr.");
    return nullptr;
  }

  const pcl::PointCloud<pt_I_t>::Ptr cloud_out = boost::make_shared<pcl::PointCloud<pt_I_t>>();
  cloud_out->header                            = cloud_in->header;
  cloud_out->width                             = indices->size();
  cloud_out->height                            = 1;
  cloud_out->is_dense                          = true;

  cloud_out->reserve(cloud_out->width);
  for (const auto idx : *indices) {
    const auto &point = cloud_in->at(idx.val);

    pt_I_t point_I;
    point_I.x         = point.x;
    point_I.y         = point.y;
    point_I.z         = point.z;
    point_I.intensity = float(point.ring) + scan_period_sec * (idx.azimuth / (2.0 * M_PI));

    cloud_out->push_back(point_I);
  }

  return cloud_out;
}
/*//}*/

/*//{ publishCloud() */

template <typename T_pt>
inline void publishCloud(const ros::Publisher &pub, const feature_selection::FSCloudManagerPtr<T_pt> cloud_manager) {
  if (cloud_manager && pub.getNumSubscribers() > 0) {

    boost::shared_ptr<pcl::PointCloud<T_pt>> cloud;
    if (cloud_manager->getIndicesPtr()) {
      cloud = cloud_manager->extractCloudByIndices();
    } else {
      cloud = cloud_manager->getCloudPtr();
    }

    const sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloud, *msg);

    try {
      pub.publish(msg);
    }
    catch (...) {
    }
  }
}

template <typename T_pt>
inline void publishCloud(const ros::Publisher &pub, const boost::shared_ptr<pcl::PointCloud<T_pt>> cloud,
                         const feature_extraction::indices_ptr_t indices = nullptr) {
  if (cloud && pub.getNumSubscribers() > 0) {

    const auto manager = std::make_shared<feature_selection::FSCloudManager<T_pt>>(cloud, indices);
    publishCloud(pub, manager);
  }
}

/*//}*/
