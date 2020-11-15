#include "aloam_slam/feature_selection.h"

namespace aloam_slam
{

/*//{ FeatureSelection() */
FeatureSelection::FeatureSelection(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader) {

  ros::NodeHandle nh_(parent_nh);
  ros::Time::waitForValid();

  /* param_loader.loadParam("mapping_resolution/grad_percent", _resolution_grad_prc_scale, 0.5f); */
  param_loader.loadParam("mapping_resolution/corner/min", _resolution_corners_min, 0.05f);
  param_loader.loadParam("mapping_resolution/corner/max", _resolution_corners_max, 0.6f);
  param_loader.loadParam("mapping_resolution/surfs/min", _resolution_surfs_min, 0.1f);
  param_loader.loadParam("mapping_resolution/surfs/max", _resolution_surfs_max, 1.0f);

  param_loader.loadParam("feature_selection/enable", _features_selection_enabled, false);
  param_loader.loadParam("feature_selection/min_count_percent", _features_min_count_percent, 0.3f);
  /* param_loader.loadParam("feature_selection/corners/gradient/limit/upper", _features_corners_gradient_limit_upper, 1.0f); */
  /* param_loader.loadParam("feature_selection/corners/gradient/limit/bottom", _features_corners_gradient_limit_bottom, 0.0f); */
  /* param_loader.loadParam("feature_selection/surfs/gradient/limit/upper", _features_surfs_gradient_limit_upper, 1.0f); */
  /* param_loader.loadParam("feature_selection/surfs/gradient/limit/bottom", _features_surfs_gradient_limit_bottom, 0.0f); */

  _resolution_corners = (_resolution_corners_max - _resolution_corners_min) / 2.0f;
  _resolution_surfs   = (_resolution_surfs_max - _resolution_surfs_min) / 2.0f;

  _pub_features_corners_selected = nh_.advertise<sensor_msgs::PointCloud2>("features_corners_selected_out", 1);
  _pub_features_surfs_selected   = nh_.advertise<sensor_msgs::PointCloud2>("features_surfs_selected_out", 1);
}
/*//}*/

/*//{ selectFeatures() */
std::tuple<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr, float, float> FeatureSelection::selectFeatures(
    const pcl::PointCloud<PointType>::Ptr &corner_points, const pcl::PointCloud<PointType>::Ptr &surf_points,
    aloam_slam::FeatureSelectionDiagnostics &diag_msg) {

  diag_msg.enabled                       = _features_selection_enabled;
  diag_msg.sizeof_features_corners_kb_in = (corner_points->size() * POINT_SIZE) / 1024.0f;
  diag_msg.sizeof_features_surfs_kb_in   = (surf_points->size() * POINT_SIZE) / 1024.0f;

  if (!_features_selection_enabled) {
    return std::make_tuple(corner_points, surf_points, _resolution_corners, _resolution_surfs);
  }

  TicToc t_corners;
  const auto [selected_corners, corner_gradients, corner_gradients_mean, corner_gradients_std] =
      selectFeaturesFromCloudByGradient(corner_points, _resolution_corners);
  diag_msg.corners_time_ms = t_corners.toc();

  TicToc t_surfs;
  const auto [selected_surfs, surf_gradients, surf_gradients_mean, surf_gradients_std] = selectFeaturesFromCloudByGradient(surf_points, _resolution_surfs);
  diag_msg.surfs_time_ms                                                               = t_surfs.toc();

  _resolution_corners = estimateResolution(corner_gradients_mean, corner_gradients, _resolution_corners_min, _resolution_corners_max);
  _resolution_surfs   = estimateResolution(surf_gradients_mean, surf_gradients, _resolution_surfs_min, _resolution_surfs_max);

  diag_msg.number_of_features_in          = corner_points->size() + surf_points->size();
  diag_msg.number_of_corners_in           = corner_points->size();
  diag_msg.number_of_surfs_in             = surf_points->size();
  diag_msg.number_of_features_out         = selected_corners->size() + selected_surfs->size();
  diag_msg.number_of_corners_out          = selected_corners->size();
  diag_msg.number_of_surfs_out            = selected_surfs->size();
  diag_msg.corners_resolution             = _resolution_corners;
  diag_msg.surfs_resolution               = _resolution_surfs;
  diag_msg.corners_cutoff_thrd            = corner_gradients_mean;
  diag_msg.surfs_cutoff_thrd              = surf_gradients_mean;
  diag_msg.corners_gradient_mean          = corner_gradients_mean;
  diag_msg.surfs_gradient_mean            = surf_gradients_mean;
  diag_msg.corners_gradient_stddev        = corner_gradients_std;
  diag_msg.surfs_gradient_stddev          = surf_gradients_std;
  diag_msg.corners_gradient_sorted        = corner_gradients;
  diag_msg.surfs_gradient_sorted          = surf_gradients;
  diag_msg.sizeof_features_corners_kb_out = (selected_corners->size() * POINT_SIZE) / 1024.0f;
  diag_msg.sizeof_features_surfs_kb_out   = (selected_surfs->size() * POINT_SIZE) / 1024.0f;

  // Publish selected features
  publishCloud(_pub_features_corners_selected, selected_corners);
  publishCloud(_pub_features_surfs_selected, selected_surfs);

  ROS_WARN_THROTTLE(0.5, "[FeatureSelection] feature selection of corners and surfs: %0.1f ms", diag_msg.corners_time_ms + diag_msg.surfs_time_ms);

  return std::make_tuple(selected_corners, selected_surfs, _resolution_corners, _resolution_surfs);
}
/*//}*/

/*//{ selectFeaturesFromCloudByGradient() */
std::tuple<pcl::PointCloud<PointType>::Ptr, std::vector<float>, float, float> FeatureSelection::selectFeaturesFromCloudByGradient(
    const pcl::PointCloud<PointType>::Ptr cloud, const float search_radius) {

  pcl::PointCloud<PointType>::Ptr selected_features = boost::make_shared<pcl::PointCloud<PointType>>();

  if (cloud->size() == 0) {
    return std::make_tuple(selected_features, std::vector<float>(), -1.0f, -1.0f);
  }

  const unsigned int cloud_size = cloud->size();
  std::vector<float> gradient_norms(cloud_size);
  std::vector<float> gradient_norms_sorted(cloud_size);
  float              grad_norm_max  = 0.0;
  float              grad_norm_mean = 0.0;
  float              grad_norm_std  = 0.0;

  selected_features->resize(cloud_size);

  // Create kd tree for fast search, TODO: find neighbors by indexing
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_features;
  kdtree_features.setInputCloud(cloud);

  for (unsigned int i = 0; i < cloud_size; i++) {

    const pcl::PointXYZI point = cloud->at(i);
    std::vector<int>     indices;
    std::vector<float>   squared_distances;

    // Find all features in given radius, TODO: find neighbors by indexing
    kdtree_features.radiusSearch(point, search_radius, indices, squared_distances);

    Eigen::Vector3f gradient  = Eigen::Vector3f::Zero();
    float           grad_norm = 0.0;

    // Compute gradient
    if (indices.size() > 0) {

      Eigen::Vector3f point_xyz = Eigen::Vector3f(point.x, point.y, point.z);
      for (const int &ind : indices) {
        const pcl::PointXYZI neighbor = cloud->at(ind);
        gradient += Eigen::Vector3f(neighbor.x, neighbor.y, neighbor.z) - point_xyz;
      }

      gradient /= indices.size();
      grad_norm = gradient.norm();
    }

    gradient_norms.at(i) = grad_norm;

    grad_norm_max = std::max(grad_norm_max, grad_norm);
  }

  // Sort gradients in non-ascending order by indices
  std::vector<int> indices(cloud_size);
  std::iota(indices.begin(), indices.end(), 0);
  std::stable_sort(indices.begin(), indices.end(), [&gradient_norms](int a, int b) { return gradient_norms[a] > gradient_norms[b]; });

  // Estimate mean of gradient norms
  for (unsigned int i = 0; i < cloud_size; i++) {

    const int idx = indices.at(i);

    // Normalize gradients to range 0-1
    gradient_norms.at(idx) /= grad_norm_max;

    // Sum gradients for mean estimation
    grad_norm_mean += gradient_norms.at(idx);
  }
  grad_norm_mean /= float(cloud_size);

  // Compute standard deviation, fill vector of sorted gradients and select features with gradient above mean
  size_t             k                 = 0;
  const unsigned int min_feature_count = std::floor(_features_min_count_percent * cloud_size);
  for (unsigned int i = 0; i < cloud_size; i++) {

    const int idx               = indices.at(i);
    gradient_norms_sorted.at(i) = gradient_norms.at(idx);

    // Compute standard deviation
    grad_norm_std += std::pow(gradient_norms_sorted.at(i) - grad_norm_mean, 2);

    // Select points with grad norm above mean only
    if (gradient_norms_sorted.at(i) >= grad_norm_mean || i < min_feature_count) {
      pcl::PointXYZI point_grad  = cloud->at(idx);
      point_grad.intensity       = gradient_norms_sorted.at(i);
      selected_features->at(k++) = point_grad;
    }
  }
  grad_norm_std = std::sqrt(grad_norm_std / float(cloud_size));

  if (k != cloud_size) {
    selected_features->resize(k);
  }

  selected_features->header   = cloud->header;
  selected_features->width    = 1;
  selected_features->height   = selected_features->points.size();
  selected_features->is_dense = cloud->is_dense;

  return std::make_tuple(selected_features, gradient_norms_sorted, grad_norm_mean, grad_norm_std);
}
/*//}*/

/*//{ estimateResolution() */
float FeatureSelection::estimateResolution(const float &percent, const std::vector<float> &fnc_sorted, const float &min_res, const float &max_res) {
  if (fnc_sorted.size() == 0) {
    return min_res;
  }

  const unsigned int idx = std::round(percent * float(fnc_sorted.size()));
  return min_res + fnc_sorted.at(idx) * (max_res - min_res);
}
/*//}*/

/*//{ publishCloud() */
void FeatureSelection::publishCloud(ros::Publisher publisher, const pcl::PointCloud<PointType>::Ptr cloud) {
  if (publisher.getNumSubscribers() > 0) {

    sensor_msgs::PointCloud2::Ptr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloud, *cloud_msg);

    try {
      publisher.publish(cloud_msg);
    }
    catch (...) {
      ROS_ERROR("exception caught during publishing on topic: %s", publisher.getTopic().c_str());
    }
  }
}
/*//}*/

}  // namespace aloam_slam
