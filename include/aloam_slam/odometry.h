#ifndef ALOAM_ODOMETRY_H
#define ALOAM_ODOMETRY_H

#include "aloam_slam/mapping.h"

namespace aloam_slam
{

struct OdometryData
{
  std::chrono::milliseconds::rep time_feature_extraction;

  ros::Time     stamp_ros;
  std::uint64_t stamp_pcl;

  pcl::PointCloud<PointType>::Ptr cloud_raw;
  CloudManagerPtr                 manager_finite_points;
  CloudManagerPtr                 manager_corners_sharp;
  CloudManagerPtr                 manager_corners_less_sharp;
  CloudManagerPtr                 manager_surfs_flat;
  CloudManagerPtr                 manager_surfs_less_flat;
};

class AloamOdometry {

public:
  AloamOdometry(const std::shared_ptr<CommonHandlers_t> handlers, const std::shared_ptr<AloamMapping> aloam_mapping);

  std::atomic<bool> is_initialized = false;

  void setData(const std::shared_ptr<OdometryData> odometry_data);
  void setTransform(const Eigen::Vector3d &t, const Eigen::Quaterniond &q, const ros::Time &stamp);
  void setFrequency(const float frequency);

private:
  std::shared_ptr<CommonHandlers_t>                    _handlers;
  std::shared_ptr<feature_selection::FeatureSelection> _feature_selection;

  // member objects
  std::shared_ptr<AloamMapping>                  _aloam_mapping;
  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  ros::Timer                                     _timer_odometry_loop;

  std::shared_ptr<mrs_lib::Transformer> _transformer;
  /* mrs_lib::SubscribeHandler<nav_msgs::Odometry> _sub_handler_orientation; */

  std::mutex                      _mutex_odometry_process;
  pcl::PointCloud<PointType>::Ptr _features_corners_last;
  pcl::PointCloud<PointType>::Ptr _features_surfs_last;

  Eigen::Quaterniond _q_w_curr;
  Eigen::Vector3d    _t_w_curr;

  // Feature extractor newest data
  std::mutex                    _mutex_odometry_data;
  bool                          _has_new_data = false;
  std::shared_ptr<OdometryData> _odometry_data;

  // publishers and subscribers
  ros::Publisher _pub_odometry_local;

  // member variables
  float _scan_period_sec;

  long int _frame_count = 0;

  double                         _para_q[4] = {0, 0, 0, 1};
  double                         _para_t[3] = {0, 0, 0};
  Eigen::Map<Eigen::Quaterniond> _q_last_curr;
  Eigen::Map<Eigen::Vector3d>    _t_last_curr;

  // constants
  const double DISTANCE_SQ_THRESHOLD = 25.0;
  const double NEARBY_SCAN           = 2.5;
  const bool   DISTORTION            = false;

  // member methods
  void timerOdometry(const ros::TimerEvent &event);

  void TransformToStart(PointType const *const pi, PointType *const po);
  void TransformToEnd(PointType const *const pi, PointType *const po);
};
}  // namespace aloam_slam
#endif
