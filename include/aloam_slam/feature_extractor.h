#pragma once

#include "aloam_slam/odometry.h"

namespace aloam_slam
{

struct FECloudManagersOut_t
{
  feature_selection::FSCloudManagerPtr man_edges_extracted  = nullptr;
  feature_selection::FSCloudManagerPtr man_edges_salient    = nullptr;
  feature_selection::FSCloudManagerPtr man_planes_extracted = nullptr;
  feature_selection::FSCloudManagerPtr man_planes_salient   = nullptr;
};


class FeatureExtractor {

public:
  FeatureExtractor(const std::shared_ptr<CommonHandlers_t> handlers, const std::shared_ptr<AloamOdometry> aloam_odometry);

  bool extractFeatures(const sensor_msgs::PointCloud2::ConstPtr msg, const std::shared_ptr<FECloudManagersOut_t> managers = nullptr);

  std::atomic<bool> is_initialized = false;

private:
  std::shared_ptr<CommonHandlers_t>                             _handlers;
  std::unique_ptr<feature_extraction::LidarExtractionEdgePlane> _fe_edge_plane_;

  bool _has_required_parameters = false;

  ros::Subscriber _sub_points_;
  void            callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr msg);

  ros::Subscriber _sub_input_data_processing_diag;
  void            callbackInputDataProcDiag(const mrs_msgs::PclToolsDiagnosticsConstPtr &msg);

  std::shared_ptr<AloamOdometry> _aloam_odometry;

  // member variables
  float    _scan_period_sec;
  long int _frame_count = 0;
  int      _initialization_frames_delay;

  bool hasField(const std::string field, const sensor_msgs::PointCloud2::ConstPtr &msg);

private:
  ros::Publisher _pub_diagnostics_;
  ros::Publisher _pub_points_finite_;
  ros::Publisher _pub_features_edges_;
  ros::Publisher _pub_features_planes_;
  ros::Publisher _pub_features_edges_salient_;
  ros::Publisher _pub_features_planes_salient_;
};
}  // namespace aloam_slam
