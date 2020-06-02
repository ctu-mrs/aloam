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
  AloamMapping(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::string frame_fcu, std::string frame_map,
               tf::Transform tf_fcu_to_lidar);

  bool is_initialized = false;

  void compute_mapping(nav_msgs::Odometry odometry, pcl::PointCloud<PointType>::Ptr laserCloudCornerLast, pcl::PointCloud<PointType>::Ptr laserCloudSurfLast,
                       pcl::PointCloud<PointType>::Ptr laserCloudFullRes);

private:
  // member objects
  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudCornerArray;
  std::vector<pcl::PointCloud<PointType>::Ptr> laserCloudSurfArray;

  PointType _pointOri;
  PointType _pointSel;

  std::vector<int>   _pointSearchInd;
  std::vector<float> m_pointSearchSqDis;

  // publishers and subscribers
  ros::Publisher _pub_laser_cloud_map;         // m_pubLaserCloudMap
  ros::Publisher _pub_laser_cloud_registered;  // m_pubLaserCloudFullRes
  ros::Publisher _pub_odom_global;             // m_pubOdomAftMapped
  ros::Publisher _pub_path;                    // m_pubLaserAfterMappedPath

  // ROS messages
  nav_msgs::Path laserAfterMappedPath;

  // member variables
  std::string _frame_fcu;
  std::string _frame_map;
  
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

  int       laserCloudValidInd[125];
  int       laserCloudCenWidth  = 10;
  int       laserCloudCenHeight = 10;
  int       laserCloudCenDepth  = 5;
  const int laserCloudWidth     = 21;
  const int laserCloudHeight    = 21;
  const int laserCloudDepth     = 11;
  const int laserCloudNum       = laserCloudWidth * laserCloudHeight * laserCloudDepth;  // 4851

  // member methods
  void transformAssociateToMap();
  void transformUpdate();
  void pointAssociateToMap(PointType const *const pi, PointType *const po);
  void pointAssociateTobeMapped(PointType const *const pi, PointType *const po);
};
}  // namespace aloam_slam
#endif
