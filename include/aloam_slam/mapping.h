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
#include <queue>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <ceres/ceres.h>
#include <opencv/cv.h>

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

#include "aloam_slam/common.h"
#include "aloam_slam/tic_toc.h"
#include "aloam_slam/lidarFactor.hpp"

//}

namespace aloam_slam
{
class AloamMapping {

public:
  AloamMapping(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::shared_ptr<mrs_lib::Profiler> profiler, std::string frame_fcu,
               std::string frame_map, float scan_frequency, tf::Transform tf_fcu_to_lidar);

  bool is_initialized = false;

  void setData(nav_msgs::Odometry aloam_odometry, pcl::PointCloud<PointType>::Ptr laserCloudCornerLast, pcl::PointCloud<PointType>::Ptr laserCloudSurfLast,
               pcl::PointCloud<PointType>::Ptr laserCloudFullRes);

private:
  // member objects
  std::shared_ptr<mrs_lib::Profiler> _profiler;

  ros::Timer _timer_mapping_loop;
  ros::Time  _time_last_map_publish;

  std::vector<pcl::PointCloud<PointType>::Ptr> _cloud_corners;
  std::vector<pcl::PointCloud<PointType>::Ptr> _cloud_surfs;

  // Feature extractor newest data
  bool                            _has_new_data = false;
  nav_msgs::Odometry              _aloam_odometry;
  pcl::PointCloud<PointType>::Ptr _features_corners_last;
  pcl::PointCloud<PointType>::Ptr _features_surfs_last;
  pcl::PointCloud<PointType>::Ptr _cloud_full_res;

  // publishers and subscribers
  ros::Publisher _pub_laser_cloud_map;
  ros::Publisher _pub_laser_cloud_registered;
  ros::Publisher _pub_odom_global;
  ros::Publisher _pub_path;

  // mutexes
  std::mutex _mutex_odometry_data;

  // ROS messages
  nav_msgs::Path _laser_path_msg;

  // member variables
  std::string _frame_fcu;
  std::string _frame_map;

  float _scan_frequency;
  float _mapping_frequency;
  float _map_publish_period;

  tf::Transform _tf_fcu_to_lidar;

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

  void transformAssociateToMap();
  void transformUpdate();
  void pointAssociateToMap(PointType const *const pi, PointType *const po);
  void pointAssociateTobeMapped(PointType const *const pi, PointType *const po);
};
}  // namespace aloam_slam
#endif