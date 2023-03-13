#pragma once

/* includes //{ */

#include <random>
#include <algorithm>

#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>

#include "aloam_slam/common.h"
#include "aloam_slam/lidarFactor.hpp"

#include "aloam_slam/AloamDiagnostics.h"
#include "aloam_slam/OdometryDiagnostics.h"
#include "aloam_slam/MappingDiagnostics.h"

//}

namespace aloam_slam
{

class VoxelFilter {

public:
  VoxelFilter(){};
  VoxelFilter(const PC_ptr cloud, const feature_extraction::indices_ptr_t indices, const float resolution);
  PC_ptr filter() const;

private:
  bool _enabled = true;
  bool _empty   = false;

  bool _return_centroid = true;

  PC_ptr                            _cloud   = nullptr;
  feature_extraction::indices_ptr_t _indices = nullptr;

  Eigen::Array3f  _inverse_leaf_size;
  Eigen::Vector3i _div_b_mul;
  Eigen::Vector3f _min_b_flt;
};

struct MappingCloudManagersOut_t
{
  feature_selection::FSCloudManagerPtr man_edges_selected  = nullptr;
  feature_selection::FSCloudManagerPtr man_planes_selected = nullptr;
};

struct MappingData
{
  feature_extraction::FEDiagnostics diagnostics_fe;
  aloam_slam::OdometryDiagnostics   diagnostics_odometry;

  ros::Time                            stamp_ros;
  tf::Transform                        odometry;
  feature_selection::FSCloudManagerPtr manager_corners;
  feature_selection::FSCloudManagerPtr manager_surfs;
};

class AloamMapping {

public:
  AloamMapping(const std::shared_ptr<CommonHandlers_t> handlers);

  bool computeMapping(geometry_msgs::TransformStamped &tf_msg_out, aloam_slam::AloamDiagnostics &diag_msg_out,
                      const std::shared_ptr<MappingCloudManagersOut_t> managers_out = nullptr);

  std::atomic<bool> is_initialized = false;

  void setData(const std::shared_ptr<MappingData> data);
  void setTransform(const Eigen::Vector3d &t, const Eigen::Quaterniond &q, const ros::Time &stamp);

private:
  std::shared_ptr<CommonHandlers_t> _handlers;

  // member objects
  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

  ros::Timer _timer_mapping_loop;
  ros::Time  _time_last_map_publish;

  std::mutex          _mutex_cloud_features;
  std::vector<PC_ptr> _cloud_corners;
  std::vector<PC_ptr> _cloud_surfs;

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
  ros::Publisher _pub_diag;
  ros::Publisher _pub_features_edges_selected_;
  ros::Publisher _pub_features_planes_selected_;

  // services
  ros::ServiceServer _srv_reset_mapping;

  // ROS messages
  nav_msgs::Path::Ptr _laser_path_msg      = boost::make_shared<nav_msgs::Path>();
  Eigen::Vector3d     _path_last_added_pos = Eigen::Vector3d::Identity();

  // member variables
  float _scan_period_sec;
  float _mapping_frequency;
  float _map_publish_period;
  bool  _remap_tf;

  double                         _parameters[7] = {0, 0, 0, 1, 0, 0, 0};
  Eigen::Map<Eigen::Quaterniond> _q_w_curr;
  Eigen::Map<Eigen::Vector3d>    _t_w_curr;

  // wmap_T_odom * odom_T_curr = wmap_T_curr;
  // transformation between odom's world and map's world frame
  Eigen::Quaterniond _q_wmap_wodom;
  Eigen::Vector3d    _t_wmap_wodom;

  Eigen::Quaterniond _q_wodom_curr;
  Eigen::Vector3d    _t_wodom_curr;

  uint32_t _frame_count = 0;

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
  pt_t pointAssociateToMap(const pt_t &pi) const;


  std::pair<std::vector<LidarEdgeFactor *>, std::vector<LidarPlaneFactor *>> findFactors(const std::shared_ptr<pcl::KdTreeFLANN<pt_t>> kdtree_map_corners,
                                                                                         const std::shared_ptr<pcl::KdTreeFLANN<pt_t>> kdtree_map_surfs,
                                                                                         const PC_ptr features_corners_stack, const PC_ptr features_surfs_stack,
                                                                                         const PC_ptr map_features_corners, const PC_ptr map_features_surfs);

  std::pair<std::vector<LidarEdgeFactor *>, std::vector<LidarPlaneFactor *>> selectFactorsGreedy(
      const std::pair<std::vector<LidarEdgeFactor *>, std::vector<LidarPlaneFactor *>> &factors);


  // | -------------------- Feature selection ------------------- |
  struct LidarFeatureSelectionEdgePlaneIO_t
  {
    PC_ptr                            cloud;
    feature_extraction::indices_ptr_t indices_edges_extracted;   // All edge features
    feature_extraction::indices_ptr_t indices_planes_extracted;  // All plane features

    bool                              selection_enabled;
    bool                              selection_success = false;
    feature_extraction::indices_ptr_t indices_edges_selected;
    feature_extraction::indices_ptr_t indices_planes_selected;
  };
  void selectFeatures(LidarFeatureSelectionEdgePlaneIO_t &selection_io, feature_selection::FSDiagnostics &msg_diag);
};
}  // namespace aloam_slam
