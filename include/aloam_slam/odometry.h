#ifndef ALOAM_ODOMETRY_H
#define ALOAM_ODOMETRY_H

#include "aloam_slam/mapping.h"

namespace aloam_slam
{
class AloamOdometry {

public:
  AloamOdometry(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::shared_ptr<AloamMapping> mapper, std::string frame_lidar,
                std::string frame_map, float scan_period_sec, tf::Transform tf_lidar_to_fcu);

  bool is_initialized = false;

  void compute_local_odometry(double time_feature_extraction, pcl::PointCloud<PointType>::Ptr cornerPointsSharp, pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp,
                              pcl::PointCloud<PointType>::Ptr surfPointsFlat, pcl::PointCloud<PointType>::Ptr surfPointsLessFlat,
                              pcl::PointCloud<PointType>::Ptr laserCloudFullRes);

private:
  // member objects
  std::shared_ptr<AloamMapping> _mapper;
  ros::Subscriber               _sub_orientation_meas;

  // TODO: make these objects local
  pcl::PointCloud<PointType>::Ptr       laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr       laserCloudSurfLast;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast;

  Eigen::Quaterniond q_w_curr;
  Eigen::Vector3d    t_w_curr;

  // publishers and subscribers
  ros::Publisher _pub_odometry_local;  // o_pubLaserOdometry

  // mutexes
  std::mutex _mutex_orientation_meas;

  // ROS msgs
  nav_msgs::Odometry _orientation_meas;

  // member variables
  std::string _frame_lidar;
  std::string _frame_map;

  float _scan_period_sec;

  double timeCornerPointsSharp     = 0;
  double timeCornerPointsLessSharp = 0;
  double timeSurfPointsFlat        = 0;
  double timeSurfPointsLessFlat    = 0;
  double timeLaserCloudFullRes     = 0;

  long int _frame_count = 0;

  int _skip_mapping_frame_num;

  tf::Transform _tf_lidar_to_fcu;

  bool _got_orientation_meas = false;

  double                         para_q[4] = {0, 0, 0, 1};
  double                         para_t[3] = {0, 0, 0};
  Eigen::Map<Eigen::Quaterniond> q_last_curr;
  Eigen::Map<Eigen::Vector3d>    t_last_curr;

  // constants
  const double DISTANCE_SQ_THRESHOLD = 25.0;
  const double NEARBY_SCAN           = 2.5;
  const bool   DISTORTION            = false;

  // member methods
  void TransformToStart(PointType const *const pi, PointType *const po);
  void TransformToEnd(PointType const *const pi, PointType *const po);

  // callbacks
  void callbackOrientationMeas(const nav_msgs::OdometryConstPtr &msg);
};
}  // namespace aloam_slam
#endif
