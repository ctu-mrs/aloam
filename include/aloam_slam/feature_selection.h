#pragma once

#include "aloam_slam/mapping.h"

namespace aloam_slam
{
class FeatureSelection {

public:
  FeatureSelection(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader);

  std::tuple<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr, float, float> selectFeatures(
      const pcl::PointCloud<PointType>::Ptr &corner_points, const pcl::PointCloud<PointType>::Ptr &surf_points,
      aloam_slam::FeatureSelectionDiagnostics &diag_msg);

  bool is_initialized = false;

private:
  ros::Publisher _pub_features_corners_selected;
  ros::Publisher _pub_features_surfs_selected;

  /* float _resolution_grad_prc_scale; */
  float _resolution_corners;
  float _resolution_surfs;
  float _resolution_corners_min;
  float _resolution_corners_max;
  float _resolution_surfs_min;
  float _resolution_surfs_max;

  bool  _features_selection_enabled;
  float _features_min_count_percent;
  /* float _features_corners_gradient_limit_upper; */
  /* float _features_corners_gradient_limit_bottom; */
  /* float _features_surfs_gradient_limit_upper; */
  /* float _features_surfs_gradient_limit_bottom; */

  std::tuple<pcl::PointCloud<PointType>::Ptr, std::vector<float>, float, float> selectFeaturesFromCloudByGradient(const pcl::PointCloud<PointType>::Ptr cloud,
                                                                                                                  const float search_radius);

  std::tuple<std::vector<int>, std::vector<std::pair<int, float>>, float, float> estimateGradients(const pcl::PointCloud<PointType>::Ptr cloud,
                                                                                                   const float                           search_radius);

  float estimateResolution(const float &percent, const std::vector<float> &fnc_sorted, const float &min_res, const float &max_res);

  void publishCloud(ros::Publisher publisher, const pcl::PointCloud<PointType>::Ptr cloud);

  const size_t POINT_SIZE = sizeof(PointType);
};
}  // namespace aloam_slam
