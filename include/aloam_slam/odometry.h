#ifndef ALOAM_ODOMETRY_H
#define ALOAM_ODOMETRY_H

#include "aloam_slam/mapping.h"

#include <mrs_lib/subscribe_handler.h>

namespace aloam_slam
{
class AloamOdometry {

public:
  AloamOdometry(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_lib::Profiler> profiler, std::shared_ptr<AloamMapping> aloam_mapping, std::string frame_fcu,
                std::string frame_lidar, std::string frame_odom, float scan_period_sec, tf::Transform tf_lidar_to_fcu);

  bool is_initialized = false;

  void setData(pcl::PointCloud<PointType>::Ptr corner_points_sharp, pcl::PointCloud<PointType>::Ptr corner_points_less_sharp,
               pcl::PointCloud<PointType>::Ptr surf_points_flat, pcl::PointCloud<PointType>::Ptr surf_points_less_flat,
               pcl::PointCloud<PointType>::Ptr laser_cloud_full_res);

private:
  // member objects
  std::shared_ptr<mrs_lib::Profiler>             _profiler;
  std::shared_ptr<AloamMapping>                  _aloam_mapping;
  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  ros::Timer                                     _timer_odometry_loop;

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> _sub_handler_orientation;

  std::mutex                      _mutex_odometry_process;
  pcl::PointCloud<PointType>::Ptr _features_corners_last;
  pcl::PointCloud<PointType>::Ptr _features_surfs_last;

  Eigen::Quaterniond _q_w_curr;
  Eigen::Vector3d    _t_w_curr;

  // Feature extractor newest data
  std::mutex                      _mutex_extracted_features;
  bool                            _has_new_data = false;
  pcl::PointCloud<PointType>::Ptr _corner_points_sharp;
  pcl::PointCloud<PointType>::Ptr _corner_points_less_sharp;
  pcl::PointCloud<PointType>::Ptr _surf_points_flat;
  pcl::PointCloud<PointType>::Ptr _surf_points_less_flat;
  pcl::PointCloud<PointType>::Ptr _cloud_full_ress;

  // publishers and subscribers
  ros::Publisher _pub_odometry_local;

  // member variables
  std::string _frame_fcu;
  std::string _frame_lidar;
  std::string _frame_odom;

  float _scan_period_sec;

  long int _frame_count = 0;

  tf::Transform _tf_lidar_to_fcu;

  double                         _para_q[4] = {0, 0, 0, 1};
  double                         _para_t[3] = {0, 0, 0};
  Eigen::Map<Eigen::Quaterniond> _q_last_curr;
  Eigen::Map<Eigen::Vector3d>    _t_last_curr;

  // constants
  const double DISTANCE_SQ_THRESHOLD = 25.0;
  const double NEARBY_SCAN           = 2.5;
  const bool   DISTORTION            = false;

  // member methods
  void timerOdometry([[maybe_unused]] const ros::TimerEvent &event);

  void TransformToStart(PointType const *const pi, PointType *const po);
  void TransformToEnd(PointType const *const pi, PointType *const po);
};
}  // namespace aloam_slam
#endif
