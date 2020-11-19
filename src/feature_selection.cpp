#include "aloam_slam/feature_selection.h"

namespace aloam_slam
{

/*//{ FeatureSelection() */
FeatureSelection::FeatureSelection(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader) {

  ros::NodeHandle nh_(parent_nh);
  ros::Time::waitForValid();

  param_loader.loadParam("mapping_resolution/corners/min", _resolution_corners_min, 0.05f);
  param_loader.loadParam("mapping_resolution/corners/max", _resolution_corners_max, 0.6f);
  param_loader.loadParam("mapping_resolution/surfs/min", _resolution_surfs_min, 0.1f);
  param_loader.loadParam("mapping_resolution/surfs/max", _resolution_surfs_max, 1.0f);

  param_loader.loadParam("feature_selection/enable", _features_selection_enabled, false);
  param_loader.loadParam("feature_selection/min_count_percent", _features_min_count_percent, 0.3f);
  param_loader.loadParam("feature_selection/corners/keep_percentile", _corners_keep_percentile, 0.5f);
  param_loader.loadParam("feature_selection/surfs/keep_percentile", _surfs_keep_percentile, 0.5f);
  /* param_loader.loadParam("feature_selection/corners/gradient/limit/upper", _features_corners_gradient_limit_upper, 1.0f); */
  /* param_loader.loadParam("feature_selection/corners/gradient/limit/bottom", _features_corners_gradient_limit_bottom, 0.0f); */
  /* param_loader.loadParam("feature_selection/surfs/gradient/limit/upper", _features_surfs_gradient_limit_upper, 1.0f); */
  /* param_loader.loadParam("feature_selection/surfs/gradient/limit/bottom", _features_surfs_gradient_limit_bottom, 0.0f); */

  _resolution_corners = (_resolution_corners_max + _resolution_corners_min) / 2.0f;
  _resolution_surfs   = (_resolution_surfs_max + _resolution_surfs_min) / 2.0f;

  _pub_features_corners_selected = nh_.advertise<sensor_msgs::PointCloud2>("features_corners_selected_out", 1);
  _pub_features_surfs_selected   = nh_.advertise<sensor_msgs::PointCloud2>("features_surfs_selected_out", 1);
}
/*//}*/

/*//{ selectFeatures() */
std::tuple<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr, float, float> FeatureSelection::selectFeatures(
    const std::shared_ptr<ExtractedFeatures> &extracted_features) {

  aloam_slam::FeatureSelectionDiagnostics diag_msg;
  diag_msg.enabled               = _features_selection_enabled;
  diag_msg.number_of_features_in = extracted_features->aloam_diag_msg->feature_extraction.corner_points_less_sharp_count +
                                   extracted_features->aloam_diag_msg->feature_extraction.surf_points_less_flat_count;
  diag_msg.number_of_corners_in          = extracted_features->aloam_diag_msg->feature_extraction.corner_points_less_sharp_count;
  diag_msg.number_of_surfs_in            = extracted_features->aloam_diag_msg->feature_extraction.surf_points_less_flat_count;
  diag_msg.sizeof_features_corners_kb_in = (extracted_features->aloam_diag_msg->feature_extraction.corner_points_less_sharp_count * POINT_SIZE) / 1024.0f;
  diag_msg.sizeof_features_surfs_kb_in   = (extracted_features->aloam_diag_msg->feature_extraction.surf_points_less_flat_count * POINT_SIZE) / 1024.0f;

  if (!_features_selection_enabled) {
    extracted_features->aloam_diag_msg->feature_selection = diag_msg;
    return std::make_tuple(extracted_features->getLessSharpCorners(), extracted_features->getLessFlatSurfs(), _resolution_corners, _resolution_surfs);
  }

  TicToc t_corners;
  const auto [selected_corners, corner_gradients, corner_gradients_mean, corner_gradients_std, corner_grad_cutoff] =
      selectFeaturesFromCloudByGradient(extracted_features->getLessSharpCorners(), extracted_features->cloud_full_res,
                                        extracted_features->indices_corners_less_sharp, _resolution_corners, _corners_keep_percentile);
  diag_msg.corners_time_ms = t_corners.toc();

  TicToc t_surfs;
  const auto [selected_surfs, surf_gradients, surf_gradients_mean, surf_gradients_std, surf_grad_cutoff] =
      selectFeaturesFromCloudByGradient(extracted_features->getLessFlatSurfs(), extracted_features->cloud_full_res, extracted_features->indices_surfs_less_flat,
                                        _resolution_surfs, _surfs_keep_percentile);
  diag_msg.surfs_time_ms = t_surfs.toc();

  _resolution_corners = estimateResolution(corner_grad_cutoff, corner_gradients, _resolution_corners_min, _resolution_corners_max);
  _resolution_surfs   = estimateResolution(surf_grad_cutoff, surf_gradients, _resolution_surfs_min, _resolution_surfs_max);
  /* _resolution_corners = 0.4f; */
  /* _resolution_surfs   = 0.8f; */

  diag_msg.number_of_features_out         = selected_corners->size() + selected_surfs->size();
  diag_msg.number_of_corners_out          = selected_corners->size();
  diag_msg.number_of_surfs_out            = selected_surfs->size();
  diag_msg.corners_resolution             = _resolution_corners;
  diag_msg.surfs_resolution               = _resolution_surfs;
  diag_msg.corners_keep_percentile        = _corners_keep_percentile;
  diag_msg.surfs_keep_percentile          = _surfs_keep_percentile;
  diag_msg.corners_cutoff_thrd            = corner_grad_cutoff;
  diag_msg.surfs_cutoff_thrd              = surf_grad_cutoff;
  diag_msg.corners_gradient_mean          = corner_gradients_mean;
  diag_msg.surfs_gradient_mean            = surf_gradients_mean;
  diag_msg.corners_gradient_stddev        = corner_gradients_std;
  diag_msg.surfs_gradient_stddev          = surf_gradients_std;
  diag_msg.corners_gradient_sorted        = corner_gradients;
  diag_msg.surfs_gradient_sorted          = surf_gradients;
  diag_msg.sizeof_features_corners_kb_out = (selected_corners->size() * POINT_SIZE) / 1024.0f;
  diag_msg.sizeof_features_surfs_kb_out   = (selected_surfs->size() * POINT_SIZE) / 1024.0f;

  extracted_features->aloam_diag_msg->feature_selection = diag_msg;

  // Publish selected features
  publishCloud(_pub_features_corners_selected, selected_corners);
  publishCloud(_pub_features_surfs_selected, selected_surfs);

  ROS_WARN_THROTTLE(0.5, "[FeatureSelection] feature selection of corners and surfs: %0.1f ms", diag_msg.corners_time_ms + diag_msg.surfs_time_ms);

  return std::make_tuple(selected_corners, selected_surfs, _resolution_corners, _resolution_surfs);
}
/*//}*/

/*//{ selectFeaturesFromCloudByGradient() */
std::tuple<pcl::PointCloud<PointType>::Ptr, std::vector<float>, float, float, float> FeatureSelection::selectFeaturesFromCloudByGradient(
    const pcl::PointCloud<PointType>::Ptr &features, const pcl::PointCloud<PointType>::Ptr &cloud_full_res,
    const std::vector<std::vector<unsigned int>> &indices_in_full_res, const float &search_radius, const float &percentile) {

  pcl::PointCloud<PointType>::Ptr selected_features = boost::make_shared<pcl::PointCloud<PointType>>();
  std::vector<float>              gradient_norms_all;

  if (features->size() == 0) {
    return std::make_tuple(selected_features, gradient_norms_all, -1.0f, -1.0f, -1.0f);
  }

  selected_features->resize(features->size());
  gradient_norms_all.resize(features->size());

  const auto [standalone_points_indices, gradients, grad_norm_mean, grad_norm_std] =
      estimateGradients(features, cloud_full_res, indices_in_full_res, search_radius);

  const float grad_cutoff_thrd = gradients.at(std::floor(percentile * gradients.size())).second;

  /* const unsigned int min_feature_count = std::floor(_features_min_count_percent * (features->size() - standalone_points.size())); */

  // Fill with features, which have no neighbors (standalone)
  for (unsigned int i = 0; i < standalone_points_indices.size(); i++) {
    selected_features->at(i) = features->at(standalone_points_indices.at(i));
    gradient_norms_all.at(i) = 1.0;
  }

  // Select features with gradient above mean
  size_t k = standalone_points_indices.size();
  for (unsigned int i = 0; i < gradients.size(); i++) {

    const float grad                                            = gradients.at(i).second;
    gradient_norms_all.at(standalone_points_indices.size() + i) = grad;

    // Select points with grad norm above mean only
    /* if (grad >= grad_norm_mean || i < min_feature_count) { */
    // TODO: keep atleast min_feature_count
    if (grad >= grad_cutoff_thrd) {
      const int            cloud_idx  = gradients.at(i).first;
      const pcl::PointXYZI point_grad = features->at(cloud_idx);

      /* point_grad.intensity       = grad; */

      selected_features->at(k++) = point_grad;
    }
  }

  if (k != features->size()) {
    selected_features->resize(k);
  }

  selected_features->header   = features->header;
  selected_features->width    = 1;
  selected_features->height   = selected_features->points.size();
  selected_features->is_dense = features->is_dense;

  return std::make_tuple(selected_features, gradient_norms_all, grad_norm_mean, grad_norm_std, grad_cutoff_thrd);
}
/*//}*/

/*//{ estimateGradients() */
std::tuple<std::vector<int>, std::vector<std::pair<int, float>>, float, float> FeatureSelection::estimateGradients(
    const pcl::PointCloud<PointType>::Ptr &features, const pcl::PointCloud<PointType>::Ptr &cloud_full_res,
    const std::vector<std::vector<unsigned int>> &indices_in_full_res, const float &search_radius) {

  // DEBUG
  /* for (unsigned int i = 0; i < indices_in_full_res.size(); i++) { */
  /*   ROS_ERROR("RING: %d, start index: %d, features: %ld", i, indices_in_full_res.at(i).first, indices_in_full_res.at(i).second.size()); */
  /*   for (int j = 0; j < 10; j++) { */
  /*     ROS_WARN("- %d", indices_in_full_res.at(i).second.at(j)); */
  /*   } */
  /* } */

  const unsigned int cloud_size                = features->size();
  size_t             grouped_features_count    = 0;
  size_t             standalone_features_count = 0;
  float              grad_norm_max             = 0.0f;
  float              grad_norm_mean            = 0.0f;
  float              grad_norm_std             = 0.0f;

  std::vector<int>                   standalone_points_indices(cloud_size);
  std::vector<std::pair<int, float>> gradient_norms(cloud_size);

  TicToc t_tmp;

  // Iterate through input cloud
  for (unsigned int i = 0; i < indices_in_full_res.size(); i++) {
    for (const auto &idx : indices_in_full_res.at(i)) {

      // TODO: pass also ranges to speed-up, inner indexes might be the same for entire row
      const std::vector<PointType> neighbors = getNearestNeighbors(cloud_full_res, indices_in_full_res.at(i), idx, search_radius * 1.5f);

      // Compute gradient
      if (neighbors.size() > 0) {

        const Eigen::Vector3f point_xyz = Eigen::Vector3f(cloud_full_res->at(idx).x, cloud_full_res->at(idx).y, cloud_full_res->at(idx).z);
        Eigen::Vector3f       gradient  = Eigen::Vector3f::Zero();
        for (const auto &neighbor : neighbors) {
          gradient += Eigen::Vector3f(neighbor.x, neighbor.y, neighbor.z) - point_xyz;
        }

        // Sum gradients for mean estimation
        const float grad_norm                       = (gradient / float(neighbors.size())).norm();
        gradient_norms.at(grouped_features_count++) = std::make_pair(i, grad_norm);

        grad_norm_max = std::fmax(grad_norm_max, grad_norm);
      } else {
        standalone_points_indices.at(standalone_features_count++) = i;
      }
    }
  }

  /*//{ old NN method, ineffective but working */
  // Create kd tree for fast search, TODO: find neighbors by indexing
  /* pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_features; */
  /* kdtree_features.setInputCloud(cloud); */
  /* const float time_build_tree = t_tmp.toc(); */
  const float time_build_tree = 0.0f;

  // Iterate through input cloud
  /* for (unsigned int i = 0; i < cloud_size; i++) { */

  /*   const pcl::PointXYZI point = cloud->at(i); */
  /*   std::vector<int>     indices; */
  /*   std::vector<float>   squared_distances; */

  /*   // Find all features in given radius, TODO: find neighbors by indexing */
  /*   kdtree_features.radiusSearch(point, search_radius, indices, squared_distances, 9); */

  /*   // Compute gradient */
  /*   if (indices.size() > 1) { */
  /*     const Eigen::Vector3f point_xyz = Eigen::Vector3f(point.x, point.y, point.z); */
  /*     Eigen::Vector3f       gradient  = Eigen::Vector3f::Zero(); */
  /*     for (const int &ind : indices) { */
  /*       const pcl::PointXYZI neighbor = cloud->at(ind); */
  /*       gradient += Eigen::Vector3f(neighbor.x, neighbor.y, neighbor.z) - point_xyz; */
  /*     } */

  /*     // Sum gradients for mean estimation */
  /*     const float grad_norm                       = (gradient / indices.size()).norm(); */
  /*     gradient_norms.at(grouped_features_count++) = std::make_pair(i, grad_norm); */

  /*     grad_norm_max = std::fmax(grad_norm_max, grad_norm); */
  /*   } else { */
  /*     standalone_points_indices.at(standalone_features_count++) = i; */
  /*   } */
  /* } */
  /*//}*/

  const float time_first_iteration = t_tmp.toc() - time_build_tree;

  if (grouped_features_count != cloud_size) {
    gradient_norms.resize(grouped_features_count);
  }
  if (standalone_features_count != cloud_size) {
    standalone_points_indices.resize(standalone_features_count);
  }

  for (unsigned int i = 0; i < grouped_features_count; i++) {
    gradient_norms.at(i).second /= grad_norm_max;
    grad_norm_mean += gradient_norms.at(i).second;
  }

  // Estimate mean
  grad_norm_mean /= float(grouped_features_count);

  // Estimate standard deviation
  for (unsigned int i = 0; i < grouped_features_count; i++) {
    grad_norm_std += std::pow(gradient_norms.at(i).second - grad_norm_mean, 2);
  }
  grad_norm_std = std::sqrt(grad_norm_std / float(cloud_size));

  // Sort gradients in non-ascending order
  std::sort(gradient_norms.begin(), gradient_norms.end(), [&](const std::pair<int, float> &i, const std::pair<int, float> &j) { return i.second > j.second; });

  const float time_mean_std_estimation = t_tmp.toc() - time_first_iteration;

  ROS_WARN("[FeatureExtractor]: Select features timing -> finding neighbors: %0.1f ms, finding gradients: %0.1f ms, mean/std estimation: %0.1f ms",
           time_build_tree, time_first_iteration, time_mean_std_estimation);

  return std::make_tuple(standalone_points_indices, gradient_norms, grad_norm_mean, grad_norm_std);
}
/*//}*/

/*//{ estimateResolution() */
float FeatureSelection::estimateResolution(const float &percent, const std::vector<float> &fnc_sorted, const float &min_res, const float &max_res) {
  /* if (fnc_sorted.size() == 0) { */
  /*   return min_res; */
  /* } */

  /* const unsigned int idx = std::round(percent * float(fnc_sorted.size())); */
  /* return min_res + fnc_sorted.at(idx) * (max_res - min_res); */
  return min_res + (percent) * (max_res - min_res);
  /* return min_res + (1.0f - percent) * (max_res - min_res); */
}
/*//}*/

/*//{ getNearestNeighbors() */
std::vector<PointType> FeatureSelection::getNearestNeighbors(const pcl::PointCloud<PointType>::Ptr &cloud_full_res,
                                                             const std::vector<unsigned int> &row_features_idxs, const int &point_idx, const float &max_range) {

  std::vector<PointType> ret;

  int        start_idx;
  const auto iter = std::find(row_features_idxs.begin(), row_features_idxs.end(), point_idx);
  if (iter != row_features_idxs.end()) {
    start_idx = iter - row_features_idxs.begin();
  } else {
    ROS_ERROR("[FeatureExtractor] getNearestNeighbors(): point index %d is not present in row indices.", point_idx);
    return ret;
  }

  /* ROS_DEBUG( */
  /*     "[FeatureExtractor] getNearestNeighbors(): point index: %d, start index: %d, cloud size: %ld, row_features_idxs start: %d, row_features_idxs size: %ld,
   * first index: %d,
   * " */
  /*     "last index: %d", */
  /*     point_idx, start_idx, cloud->size(), row_features_idxs.first, row_features_idxs.second.size(), row_features_idxs.second.front(),
   * row_features_idxs.second.back()); */

  const PointType point = cloud_full_res->at(point_idx);
  const double    d1    = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
  const double    d1d1  = d1 * d1;

  // Iterate left
  for (int i = start_idx - 1; i >= 0; i--) {
    // TODO: use given ranges, precompute angle
    const int       idx   = row_features_idxs.at(i);
    const double    angle = (point_idx - idx) * ANGULAR_RESOLUTION;
    const PointType p     = cloud_full_res->at(idx);
    const double    d2    = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    const double    r     = std::sqrt(d1d1 + d2 * d2 - 2.0 * d1 * d2 * std::cos(angle));
    if (r < max_range) {
      ret.push_back(p);
    } else {
      break;
    }
  }

  // Iterate right
  for (unsigned int i = start_idx + 1; i < row_features_idxs.size(); i++) {
    // TODO: use given ranges, precompute angle
    const int       idx   = row_features_idxs.at(i);
    const double    angle = (idx - point_idx) * ANGULAR_RESOLUTION;
    const PointType p     = cloud_full_res->at(idx);
    const double    d2    = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    const double    r     = std::sqrt(d1d1 + d2 * d2 - 2.0 * d1 * d2 * std::cos(angle));
    if (r < max_range) {
      ret.push_back(p);
    } else {
      break;
    }
  }

  return ret;
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
