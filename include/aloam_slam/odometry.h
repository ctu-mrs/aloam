#ifndef ALOAM_ODOMETRY_H
#define ALOAM_ODOMETRY_H

#include "aloam_slam/mapping.h"

#include <feature_selection/feature_selection.h>

namespace aloam_slam
{
class AloamOdometry {

public:
  AloamOdometry(const ros::NodeHandle &parent_nh, const std::string uav_name, const std::shared_ptr<mrs_lib::Profiler> profiler,
                const std::shared_ptr<AloamMapping> aloam_mapping, const std::string &frame_fcu, const std::string &frame_lidar, const std::string &frame_odom,
                const float scan_period_sec, const tf::Transform &tf_lidar_to_fcu, const bool enable_scope_timer,
                const std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger);

  std::atomic<bool> is_initialized = false;

  void setData(pcl::PointCloud<PointType>::Ptr corner_points_sharp, pcl::PointCloud<PointType>::Ptr corner_points_less_sharp,
               pcl::PointCloud<PointType>::Ptr surf_points_flat, pcl::PointCloud<PointType>::Ptr surf_points_less_flat,
               pcl::PointCloud<PointType>::Ptr laser_cloud_full_res);

  void setTransform(const Eigen::Vector3d &t, const Eigen::Quaterniond &q, const ros::Time &stamp);

private:
  bool _enable_scope_timer;

  // member objects
  std::shared_ptr<mrs_lib::Profiler>             _profiler;
  std::shared_ptr<AloamMapping>                  _aloam_mapping;
  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  ros::Timer                                     _timer_odometry_loop;

  std::shared_ptr<mrs_lib::Transformer>      _transformer;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> _scope_timer_logger;
  /* mrs_lib::SubscribeHandler<nav_msgs::Odometry> _sub_handler_orientation; */

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
