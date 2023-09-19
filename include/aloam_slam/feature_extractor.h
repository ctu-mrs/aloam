#pragma once

#include "aloam_slam/odometry.h"
#include <tf2_ros/transform_listener.h>

namespace aloam_slam
{

/*//{ getFieldValue() */
template <typename T_pt, typename T_out>
T_out getFieldValue(const T_pt &point, std::size_t field_offset) {
  const std::uint8_t *pt_data     = reinterpret_cast<const std::uint8_t *>(&(point));
  T_out               field_value = 0;
  memcpy(&field_value, pt_data + field_offset, sizeof(T_out));
  return field_value;
}
/*//}*/

template <typename T_pt>
struct FECloudManagersOut_t
{
  feature_selection::FSCloudManagerPtr<T_pt> man_edges_extracted  = nullptr;
  feature_selection::FSCloudManagerPtr<T_pt> man_edges_salient    = nullptr;
  feature_selection::FSCloudManagerPtr<T_pt> man_planes_extracted = nullptr;
  feature_selection::FSCloudManagerPtr<T_pt> man_planes_salient   = nullptr;
};

template <class T_pt>
class FeatureExtractor {

public:
  FeatureExtractor(const std::shared_ptr<CommonHandlers_t> handlers, const std::shared_ptr<AloamOdometry<T_pt>> aloam_odometry);

  bool extractFeatures(const sensor_msgs::PointCloud2::ConstPtr msg, const std::shared_ptr<FECloudManagersOut_t<T_pt>> managers = nullptr);

  std::atomic<bool> is_initialized = false;

private:
  std::shared_ptr<CommonHandlers_t>                             _handlers       = nullptr;
  std::shared_ptr<AloamOdometry<T_pt>>                          _aloam_odometry = nullptr;
  std::unique_ptr<feature_extraction::LidarExtractionEdgePlane> _fe_edge_plane_ = nullptr;

  std::shared_ptr<tf2_ros::Buffer>            _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

  bool _has_required_parameters = false;

  ros::Subscriber _sub_points_;
  void            callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr msg);

  ros::Subscriber _sub_input_data_processing_diag;
  void            callbackInputDataProcDiag(const mrs_msgs::PclToolsDiagnosticsConstPtr &msg);

  void publishRangeDiff(const boost::shared_ptr<pcl::PointCloud<T_pt>> cloud, const ros::Time &stamp);

  // member variables
  float    _scan_period_sec;
  long int _frame_count = 0;
  int      _initialization_frames_delay;

  bool                                     _publish_cloud_diff = false;
  boost::shared_ptr<pcl::PointCloud<T_pt>> _cloud_prev         = nullptr;
  Eigen::Matrix3d                          _R_prev = Eigen::Matrix3d::Identity();

  /* template <typename T_out> */
  /* T_out getFieldValue(const T_pt &point, std::size_t field_offset); */

private:
  ros::Publisher _pub_points_finite_;
  ros::Publisher _pub_features_edges_;
  ros::Publisher _pub_features_planes_;
  ros::Publisher _pub_features_edges_salient_;
  ros::Publisher _pub_features_planes_salient_;
  ros::Publisher _pub_range_diff_;
};
}  // namespace aloam_slam
