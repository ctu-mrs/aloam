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


  // TODO: precompute for multiple resolutions (R)
  const float  R      = 0.25f;
  unsigned int sample = 0;
  bool         stop   = false;
  while (!stop) {

    const float        range     = sample++ * NEIGH_IDX_PREC_RANGE_RES;
    const unsigned int max_v_idx = std::floor(std::atan2(R, range) / VFOV_RESOLUTION);
    /* ROS_DEBUG("sample: %d, range: %0.1f, max_v_idx: %d", sample - 1, range, max_v_idx); */

    std::vector<std::pair<unsigned int, unsigned int>> idxs;

    for (unsigned int i = 0; i <= max_v_idx; i++) {

      const float        k         = range * std::tan(float(i) * VFOV_RESOLUTION);
      const unsigned int max_h_idx = std::floor((std::atan2(std::sqrt(R * R - k * k), range)) / HFOV_RESOLUTION);

      /* ROS_DEBUG("max_h_idx: %d", max_h_idx); */
      idxs.push_back(std::make_pair(max_v_idx, max_h_idx));

      if (max_v_idx == 0 && max_h_idx == 0) {
        stop = true;
      }
    }

    _neigh_idxs_rows_cols.push_back(idxs);
  }

  /* const float        k         = d * std::tan((float(this_r) - float(i)) * VFOV_RESOLUTION); */
  /* const unsigned int max_h_idx = std::floor((std::atan2(std::sqrt(max_range_sq - k * k), d)) / HFOV_RESOLUTION); */

  /* std::vector<float> neigh_idxs_rows; */
  /* std::vector<float> neigh_idxs_cols; */

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

  TicToc     t_corners;
  const auto feature_set_corners =
      selectFeaturesFromCloudByGradient(extracted_features, extracted_features->aloam_diag_msg->feature_extraction.corner_points_less_sharp_count,
                                        extracted_features->indices_corners_less_sharp, 2.0 * _resolution_corners, _corners_keep_percentile);
  diag_msg.corners_time_ms = t_corners.toc();

  TicToc     t_surfs;
  const auto feature_set_surfs =
      selectFeaturesFromCloudByGradient(extracted_features, extracted_features->aloam_diag_msg->feature_extraction.surf_points_less_flat_count,
                                        extracted_features->indices_surfs_less_flat, _resolution_surfs, _surfs_keep_percentile);
  diag_msg.surfs_time_ms = t_surfs.toc();
  diag_msg.time_ms       = diag_msg.surfs_time_ms + diag_msg.corners_time_ms;

  // TODO: analyze the influence of estimateResolution method
  /* _resolution_corners = estimateResolution(corner_grad_cutoff, corner_gradients, _resolution_corners_min, _resolution_corners_max); */
  /* _resolution_surfs   = estimateResolution(surf_grad_cutoff, surf_gradients, _resolution_surfs_min, _resolution_surfs_max); */
  /* _resolution_corners = 0.4f; */
  /* _resolution_surfs   = 0.8f; */

  diag_msg.number_of_corners_out          = feature_set_corners.sorted_features_selected.size();
  diag_msg.number_of_surfs_out            = feature_set_surfs.sorted_features_selected.size();
  diag_msg.number_of_features_out         = diag_msg.number_of_corners_out + diag_msg.number_of_surfs_out;
  diag_msg.corners_resolution             = _resolution_corners;
  diag_msg.surfs_resolution               = _resolution_surfs;
  diag_msg.corners_keep_percentile        = _corners_keep_percentile;
  diag_msg.surfs_keep_percentile          = _surfs_keep_percentile;
  diag_msg.corners_gradient_mean          = feature_set_corners.grad_mean;
  diag_msg.surfs_gradient_mean            = feature_set_surfs.grad_mean;
  diag_msg.corners_gradient_stddev        = feature_set_corners.grad_stddev;
  diag_msg.surfs_gradient_stddev          = feature_set_surfs.grad_stddev;
  diag_msg.corners_selected_gradients     = getGradients(feature_set_corners.sorted_features_selected);
  diag_msg.surfs_selected_gradients       = getGradients(feature_set_surfs.sorted_features_selected);
  diag_msg.corners_others_gradients       = getGradients(feature_set_corners.sorted_features_others);
  diag_msg.surfs_others_gradients         = getGradients(feature_set_surfs.sorted_features_others);
  diag_msg.sizeof_features_corners_kb_out = (feature_set_corners.sorted_features_selected.size() * POINT_SIZE) / 1024.0f;
  diag_msg.sizeof_features_surfs_kb_out   = (feature_set_surfs.sorted_features_selected.size() * POINT_SIZE) / 1024.0f;

  extracted_features->aloam_diag_msg->feature_selection = diag_msg;

  const auto selected_corners = featuresToCloud(extracted_features, feature_set_corners.sorted_features_selected);
  const auto selected_surfs   = featuresToCloud(extracted_features, feature_set_surfs.sorted_features_selected);

  // Publish selected features
  publishCloud(_pub_features_corners_selected, selected_corners);
  publishCloud(_pub_features_surfs_selected, selected_surfs);

  /* ROS_WARN_THROTTLE(0.5, "[FeatureSelection] feature selection of corners and surfs: %0.1f ms", diag_msg.corners_time_ms + diag_msg.surfs_time_ms); */

  return std::make_tuple(selected_corners, selected_surfs, _resolution_corners, _resolution_surfs);
}
/*//}*/

/*//{ selectFeaturesFromCloudByGradient() */
FEATURE_SET FeatureSelection::selectFeaturesFromCloudByGradient(const std::shared_ptr<ExtractedFeatures> &    extracted_features,
                                                                const unsigned int &                          features_count,
                                                                const std::vector<std::vector<unsigned int>> &indices_in_filt, const float &search_radius,
                                                                const float &percentile) {

  FEATURE_SET feature_set;

  // TODO: parametrize by segments
  feature_set.idx_row_from = 0;
  feature_set.idx_row_to   = ROW_SIZE;

  if (features_count == 0) {
    return feature_set;
  }

  feature_set.sorted_features_selected.resize(features_count);
  feature_set.sorted_features_others.resize(features_count);

  const auto [standalone_points_indices, gradients, grad_norm_mean, grad_norm_std] =
      estimateGradients(extracted_features, features_count, indices_in_filt, search_radius);

  float grad_cutoff_thrd = 0.0f;

  /* const unsigned int min_feature_count = std::floor(_features_min_count_percent * (features_count - standalone_points.size())); */

  // Fill with features, which have no neighbors (standalone)
  for (unsigned int i = 0; i < standalone_points_indices.size(); i++) {
    FEATURE f;
    f.idx_in_filt_cloud                        = standalone_points_indices.at(i);
    f.gradient                                 = 1.0f;
    feature_set.sorted_features_selected.at(i) = f;
  }

  // Select features with gradient above given percentile
  unsigned int k_selected = standalone_points_indices.size();
  unsigned int k_others   = 0;
  if (gradients.size() > 0) {
    grad_cutoff_thrd = gradients.at(std::floor(percentile * gradients.size())).second;

    for (unsigned int i = 0; i < gradients.size(); i++) {

      FEATURE f;
      f.idx_in_filt_cloud = gradients.at(i).first;
      f.gradient          = gradients.at(i).second;

      // TODO: always keep atleast min_feature_count in sorted features
      if (f.gradient >= grad_cutoff_thrd) {
        feature_set.sorted_features_selected.at(k_selected++) = f;
      } else {
        feature_set.sorted_features_others.at(k_others++) = f;
      }
    }
  } else {
    ROS_WARN("[FeatureSelection] Gradients size is 0.");
  }

  if (k_selected != features_count) {
    feature_set.sorted_features_selected.resize(k_selected);
  }
  if (k_others != features_count) {
    feature_set.sorted_features_others.resize(k_others);
  }

  return feature_set;
}
/*//}*/

/*//{ estimateGradients() */
std::tuple<std::vector<unsigned int>, std::vector<std::pair<unsigned int, float>>, float, float> FeatureSelection::estimateGradients(
    const std::shared_ptr<ExtractedFeatures> &extracted_features, const unsigned int &features_count,
    const std::vector<std::vector<unsigned int>> &indices_in_filt, const float &search_radius) {

  // DEBUG
  /* for (unsigned int i = 0; i < indices_in_full_res.size(); i++) { */
  /*   ROS_ERROR("RING: %d, start index: %d, features: %ld", i, indices_in_full_res.at(i).first, indices_in_full_res.at(i).second.size()); */
  /*   for (int j = 0; j < 10; j++) { */
  /*     ROS_WARN("- %d", indices_in_full_res.at(i).second.at(j)); */
  /*   } */
  /* } */

  unsigned int grouped_features_count    = 0;
  unsigned int standalone_features_count = 0;
  float        grad_norm_max             = 0.0f;
  float        grad_norm_mean            = 0.0f;
  float        grad_norm_std             = 0.0f;

  std::vector<unsigned int>                   standalone_points_indices(features_count);
  std::vector<std::pair<unsigned int, float>> gradient_norms(features_count);

  TicToc t_tmp;

  std::unordered_map<unsigned int, std::vector<Eigen::Vector3f>> neighbors = getNeighborsInBB(extracted_features, indices_in_filt, search_radius);

  // Iterate through input cloud
  for (unsigned int i = 0; i < indices_in_filt.size(); i++) {

    // TODO: pass also ranges to speed-up, inner indexes might be the same for entire row

    for (unsigned int j = 0; j < indices_in_filt.at(i).size(); j++) {

      const unsigned int idx = indices_in_filt.at(i).at(j);

      // Compute gradient
      if (neighbors[idx].size() > 0) {
        const Eigen::Vector3f point_xyz =
            Eigen::Vector3f(extracted_features->cloud_filt->at(idx).x, extracted_features->cloud_filt->at(idx).y, extracted_features->cloud_filt->at(idx).z);
        Eigen::Vector3f gradient = Eigen::Vector3f::Zero();
        for (const auto &neighbor : neighbors[idx]) {
          gradient += neighbor - point_xyz;
        }

        // Sum gradients for mean estimation
        const float grad_norm                       = (gradient / float(neighbors[idx].size())).norm();
        gradient_norms.at(grouped_features_count++) = std::make_pair(idx, grad_norm);

        grad_norm_max = std::fmax(grad_norm_max, grad_norm);
      } else {
        standalone_points_indices.at(standalone_features_count++) = idx;
      }
    }
  }

  /* const float time_first_iteration = t_tmp.toc() - time_build_tree; */

  if (grouped_features_count != features_count) {
    gradient_norms.resize(grouped_features_count);
  }
  if (standalone_features_count != features_count) {
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
  grad_norm_std = std::sqrt(grad_norm_std / float(features_count));

  // Sort gradients in non-ascending order
  std::sort(gradient_norms.begin(), gradient_norms.end(),
            [&](const std::pair<unsigned int, float> &i, const std::pair<unsigned int, float> &j) { return i.second > j.second; });

  /* const float time_mean_std_estimation = t_tmp.toc() - time_first_iteration; */
  /* ROS_WARN("[FeatureExtractor]: Select features timing -> finding neighbors: %0.1f ms, finding gradients: %0.1f ms, mean/std estimation: %0.1f ms", */
  /*          time_build_tree, time_first_iteration, time_mean_std_estimation); */

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

/*//{ getNeighborsInBB() */
std::unordered_map<unsigned int, std::vector<Eigen::Vector3f>> FeatureSelection::getNeighborsInBB(const std::shared_ptr<ExtractedFeatures> &extracted_features,
                                                                                                  const std::vector<std::vector<unsigned int>> &indices_in_filt,
                                                                                                  const float &                                 max_range) {

  std::unordered_map<unsigned int, std::vector<Eigen::Vector3f>> neighbors_map;

  /* TicToc                                t; */
  std::vector<std::vector<int>>         ordered_table = extracted_features->buildOrderedFeatureTable(indices_in_filt);
  const pcl::PointCloud<PointType>::Ptr cloud_filt    = extracted_features->cloud_filt;
  /* ROS_WARN("timer: building feature table took %0.1f ms", t.toc()); */

  const int cloud_width  = extracted_features->cloud_raw->width;
  const int cloud_height = ordered_table.size();

  /* const float max_range_sq = max_range * max_range; */
  /* int m = 0; */

  for (const auto &indices_row : indices_in_filt) {

    for (const auto &idx_in_filt : indices_row) {

      const auto [this_r, this_c] = extracted_features->getRowColInRawData(idx_in_filt);
      const float d               = extracted_features->getRange(idx_in_filt);

      const auto row_col_idxs = getNearestNeighborLimits(d);

      const unsigned int max_v_idx = row_col_idxs.back().first;
      const unsigned int row_min   = std::max(0, int(this_r - max_v_idx));
      const unsigned int row_max   = std::min(cloud_height, int(this_r + max_v_idx + 1));

      /* ROS_DEBUG("this_r: %d, this_c: %d, row min: %d, row max: %d, max_v_idx: %d", this_r, this_c, row_min, row_max, max_v_idx); */

      ordered_table.at(this_r).at(this_c) = -1;

      for (unsigned int i = row_min; i < row_max; i++) {
        const unsigned int row_diff = std::abs(int(i - this_r));
        const unsigned int h_idx    = row_col_idxs.at(row_diff).second;
        const unsigned int col_min  = std::max(0, int(this_c - h_idx));
        const unsigned int col_max  = std::min(cloud_width, int(this_c + h_idx + 1));

        /* ROS_DEBUG("col min: %d, col max: %d, h_idx: %d", col_min, col_max, h_idx); */
        for (unsigned int j = col_min; j < col_max; j++) {
          /* ROS_DEBUG("i: %d, j: %d", i, j); */

          if (i == this_r && j == this_c) {
            continue;
          }

          const int neighbor_idx_in_filt = ordered_table.at(i).at(j);

          /* ROS_ERROR("i: %d, j: %d, neighbor_idx_in_filt: %d", i, j, neighbor_idx_in_filt); */

          if (neighbor_idx_in_filt != -1) {
            const auto            neighbor_point = cloud_filt->at(neighbor_idx_in_filt);
            const Eigen::Vector3f point          = Eigen::Vector3f(cloud_filt->at(idx_in_filt).x, cloud_filt->at(idx_in_filt).y, cloud_filt->at(idx_in_filt).z);
            const Eigen::Vector3f neighbor       = Eigen::Vector3f(neighbor_point.x, neighbor_point.y, neighbor_point.z);

            if ((neighbor - point).norm() < max_range) {
              /* ROS_DEBUG("- i: %d, j: %d: %0.2f, %0.2f, %0.2f", i, j, neighbor.x(), neighbor.y(), neighbor.z()); */
              neighbors_map[idx_in_filt].push_back(neighbor);
              neighbors_map[neighbor_idx_in_filt].push_back(point);
            }
          }
        }
      }

      /* ROS_DEBUG("this_r: %d, row min: %d, row max: %d, v_idx: %d", this_r, row_min, row_max, v_idx); */
      /* for (unsigned int l = 0; l < h_idxs.size(); l++) { */
      /*   ROS_DEBUG("- l: %d, h_idxs[l]: %d", l, h_idxs.at(l)); */
      /* } */
    }
    /* ROS_WARN("timer: row %d took %0.1f ms", m++, t.toc()); */
  }
  /* ROS_WARN("timer: double iterations took in total %0.1f ms", t.toc()); */

  /* ROS_ERROR("pes"); */
  return neighbors_map;
}
/*//}*/

/*//{ getNeighborsKdTree() */
void FeatureSelection::getNeighborsKdTree() {
  // TODO: implement for comparison of kdtree radius search and our speedup

  /*//{ old NN method, ineffective but working */
  // Create kd tree for fast search, TODO: find neighbors by indexing
  /* pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_features; */
  /* kdtree_features.setInputCloud(cloud); */
  /* const float time_build_tree = t_tmp.toc(); */
  /* const float time_build_tree = 0.0f; */

  // Iterate through input cloud
  /* for (unsigned int i = 0; i < features_count; i++) { */

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
}
/*//}*/

/*//{ getNearestNeighborLimits() */
std::vector<std::pair<unsigned int, unsigned int>> FeatureSelection::getNearestNeighborLimits(const float &point_distance) {
  const unsigned int idx = std::floor(point_distance / NEIGH_IDX_PREC_RANGE_RES);
  /* ROS_DEBUG("getNearestNeighborLimits: distance: %0.1f, idx: %d", point_distance, idx); */
  if (idx < _neigh_idxs_rows_cols.size()) {
    return _neigh_idxs_rows_cols.at(idx);
  }
  return std::vector<std::pair<unsigned int, unsigned int>>();
}
/*//}*/

/*//{ getGradients() */
std::vector<float> FeatureSelection::getGradients(const std::vector<FEATURE> &feature_vector) {
  std::vector<float> gradients(feature_vector.size(), 0.0f);
  for (unsigned int i = 0; i < feature_vector.size(); i++) {
    gradients.at(i) = feature_vector.at(i).gradient;
  }
  return gradients;
}
/*//}*/

/*//{ publishCloud() */
void FeatureSelection::publishCloud(ros::Publisher publisher, const pcl::PointCloud<PointType>::Ptr &cloud) {
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

/*//{ featuresToCloud() */
pcl::PointCloud<PointType>::Ptr FeatureSelection::featuresToCloud(const std::shared_ptr<ExtractedFeatures> &extracted_features,
                                                                  const std::vector<FEATURE> &              feature_vec) {

  pcl::PointCloud<PointType>::Ptr cloud = boost::make_shared<pcl::PointCloud<PointType>>();
  cloud->header                         = extracted_features->cloud_filt->header;
  cloud->width                          = 1;
  cloud->height                         = feature_vec.size();
  cloud->is_dense                       = true;

  cloud->resize(feature_vec.size());
  for (unsigned int i = 0; i < feature_vec.size(); i++) {
    const PointType p = extracted_features->cloud_filt->at(feature_vec.at(i).idx_in_filt_cloud);
    cloud->at(i)      = p;
  }

  return cloud;
}

/*//}*/

}  // namespace aloam_slam
