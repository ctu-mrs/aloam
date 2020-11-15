#pragma once

#include "aloam_slam/mapping.h"

namespace aloam_slam
{
class FeatureSelection {

public:
  FeatureSelection(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader);

  std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr> selectFeatures(const pcl::PointCloud<PointType>::Ptr &  corner_points,
                                                                                             const pcl::PointCloud<PointType>::Ptr &  surf_points,
                                                                                             aloam_slam::FeatureSelectionDiagnostics &diag_msg);

  bool is_initialized = false;

private:
  ros::Publisher _pub_features_corners_selected;
  ros::Publisher _pub_features_surfs_selected;

  bool  _features_selection_enabled;
  float _features_corners_resolution;
  float _features_surfs_resolution;
  float _features_corners_gradient_limit_upper;
  float _features_corners_gradient_limit_bottom;
  float _features_surfs_gradient_limit_upper;
  float _features_surfs_gradient_limit_bottom;

  std::tuple<pcl::PointCloud<PointType>::Ptr, std::vector<float>, float, float> selectFeaturesFromCloudByGradient(const pcl::PointCloud<PointType>::Ptr cloud,
                                                                                                                  const float search_radius,
                                                                                                                  const float grad_min, const float grad_max);

  void publishCloud(ros::Publisher publisher, const pcl::PointCloud<PointType>::Ptr cloud);

  const size_t POINT_SIZE = sizeof(PointType);
};
}  // namespace aloam_slam
