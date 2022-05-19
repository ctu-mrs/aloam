#ifndef ALOAM_MAPPING_H
#define ALOAM_MAPPING_H

/* includes //{ */

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

#include "aloam_slam/common.h"
#include "aloam_slam/tic_toc.h"
#include "aloam_slam/lidarFactor.hpp"

#include <feature_selection/feature_selection.h>

//}

using IndicesPtr = std::shared_ptr<feature_selection::Indices_t>;

namespace aloam_slam
{

/*//{ class CloudManager */
class CloudManager {

public:
  CloudManager(const pcl::PointCloud<PointType>::Ptr cloud) {
    std::scoped_lock lock(_mutex_cloud);
    _cloud_unordered = cloud;
    _indices         = nullptr;
  }

  CloudManager(const pcl::PointCloud<PointType>::Ptr cloud, const IndicesPtr feature_indices) {
    std::scoped_lock lock(_mutex_cloud);
    _cloud_ordered = cloud;
    _indices       = feature_indices;
  }

  pcl::PointCloud<PointType>::Ptr getUnorderedCloudPtr() {

    std::scoped_lock lock(_mutex_cloud);

    if (!_cloud_unordered && _indices) {

      _cloud_unordered           = boost::make_shared<pcl::PointCloud<PointType>>();
      _cloud_unordered->header   = _cloud_ordered->header;
      _cloud_unordered->height   = 1;
      _cloud_unordered->width    = _indices->size();
      _cloud_unordered->is_dense = true;

      _cloud_unordered->reserve(_cloud_unordered->width);
      for (const auto &idx : *_indices) {
        _cloud_unordered->push_back(_cloud_ordered->at(idx.col, idx.row));
      }
    }

    return _cloud_unordered;
  }

  std::size_t getSize() {
    if (_indices) {
      return _indices->size();
    }

    if (_cloud_unordered) {
      return _cloud_unordered->size();
    }

    return 0;
  }

private:
  std::mutex                      _mutex_cloud;
  IndicesPtr                      _indices         = nullptr;
  pcl::PointCloud<PointType>::Ptr _cloud_ordered   = nullptr;
  pcl::PointCloud<PointType>::Ptr _cloud_unordered = nullptr;
};
using CloudManagerPtr = std::shared_ptr<CloudManager>;
/*//}*/

struct MappingData
{

  std::chrono::milliseconds::rep time_feature_extraction;
  std::chrono::milliseconds::rep time_odometry;

  ros::Time                       stamp_ros;
  tf::Transform                   odometry;
  pcl::PointCloud<PointType>::Ptr cloud_corners_last;
  pcl::PointCloud<PointType>::Ptr cloud_surfs_last;
};

class AloamMapping {

public:
  AloamMapping(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, const std::shared_ptr<mrs_lib::Profiler> profiler,
               const std::string &frame_fcu, const std::string &frame_map, const tf::Transform &tf_lidar_to_fcu, const bool enable_scope_timer,
               const std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger);

  std::atomic<bool> is_initialized = false;

  void setData(const std::shared_ptr<MappingData> data);

  void setTransform(const Eigen::Vector3d &t, const Eigen::Quaterniond &q, const ros::Time &stamp);

private:
  bool _enable_scope_timer;

  // member objects
  std::shared_ptr<mrs_lib::Profiler>             _profiler;
  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  std::shared_ptr<mrs_lib::ScopeTimerLogger>     _scope_timer_logger;

  ros::Timer _timer_mapping_loop;
  ros::Time  _time_last_map_publish;

  std::mutex                                   _mutex_cloud_features;
  std::vector<pcl::PointCloud<PointType>::Ptr> _cloud_corners;
  std::vector<pcl::PointCloud<PointType>::Ptr> _cloud_surfs;

  // Feature extractor newest data
  std::condition_variable      _cv_mapping_data;
  std::mutex                   _mutex_mapping_data;
  bool                         _has_new_data = false;
  std::shared_ptr<MappingData> _mapping_data = nullptr;

  // publishers and subscribers
  ros::Publisher _pub_laser_cloud_map;
  ros::Publisher _pub_odom_global;
  ros::Publisher _pub_path;
  ros::Publisher _pub_eigenvalue;

  // services
  ros::ServiceServer _srv_reset_mapping;

  // ROS messages
  nav_msgs::Path::Ptr _laser_path_msg      = boost::make_shared<nav_msgs::Path>();
  Eigen::Vector3d     _path_last_added_pos = Eigen::Vector3d::Identity();

  // member variables
  std::string _frame_fcu;
  std::string _frame_map;

  float _mapping_frequency;
  float _map_publish_period;
  bool  _remap_tf;

  tf::Transform _tf_lidar_to_fcu;

  double                         _parameters[7] = {0, 0, 0, 1, 0, 0, 0};
  Eigen::Map<Eigen::Quaterniond> _q_w_curr;
  Eigen::Map<Eigen::Vector3d>    _t_w_curr;

  // wmap_T_odom * odom_T_curr = wmap_T_curr;
  // transformation between odom's world and map's world frame
  Eigen::Quaterniond _q_wmap_wodom;
  Eigen::Vector3d    _t_wmap_wodom;

  Eigen::Quaterniond _q_wodom_curr;
  Eigen::Vector3d    _t_wodom_curr;

  long int _frame_count = 0;

  int       _cloud_center_width  = 10;
  int       _cloud_center_height = 10;
  int       _cloud_center_depth  = 5;
  const int _cloud_width         = 21;
  const int _cloud_height        = 21;
  const int _cloud_depth         = 11;
  const int _cloud_volume        = _cloud_width * _cloud_height * _cloud_depth;  // 4851

  float _resolution_line;
  float _resolution_plane;

  // member methods
  void timerMapping([[maybe_unused]] const ros::TimerEvent &event);
  bool callbackResetMapping(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  void transformAssociateToMap();
  void transformUpdate();
  void pointAssociateToMap(PointType const *const pi, PointType *const po);
};
}  // namespace aloam_slam
#endif
