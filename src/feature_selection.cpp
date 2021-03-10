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
  param_loader.loadParam("feature_selection/local_segments", _features_selection_number_of_segments, 1);
  param_loader.loadParam("feature_selection/min_count_percent", _features_min_count_percent, 0.3f);
  param_loader.loadParam("feature_selection/corners/keep_percentile", _corners_keep_percentile, 0.5f);
  param_loader.loadParam("feature_selection/corners/keep_standalone", _corners_keep_standalone, false);
  param_loader.loadParam("feature_selection/surfs/keep_percentile", _surfs_keep_percentile, 0.5f);
  param_loader.loadParam("feature_selection/surfs/keep_standalone", _surfs_keep_standalone, false);
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
  const auto feature_set_corners = selectFeaturesFromCloudByGradient(
      extracted_features, extracted_features->aloam_diag_msg->feature_extraction.corner_points_less_sharp_count, extracted_features->indices_corners_less_sharp,
      2.0 * _resolution_corners, _corners_keep_percentile, _corners_keep_standalone, _features_selection_number_of_segments);
  diag_msg.corners_time_ms = t_corners.toc();

  TicToc     t_surfs;
  const auto feature_set_surfs = selectFeaturesFromCloudByGradient(
      extracted_features, extracted_features->aloam_diag_msg->feature_extraction.surf_points_less_flat_count, extracted_features->indices_surfs_less_flat,
      _resolution_surfs, _surfs_keep_percentile, _surfs_keep_standalone, _features_selection_number_of_segments);
  diag_msg.surfs_time_ms = t_surfs.toc();
  diag_msg.time_ms       = diag_msg.surfs_time_ms + diag_msg.corners_time_ms;

  // TODO: analyze the influence of estimateResolution method
  /* _resolution_corners = estimateResolution(corner_grad_cutoff, corner_gradients, _resolution_corners_min, _resolution_corners_max); */
  /* _resolution_surfs   = estimateResolution(surf_grad_cutoff, surf_gradients, _resolution_surfs_min, _resolution_surfs_max); */
  /* _resolution_corners = 0.4f; */
  /* _resolution_surfs   = 0.8f; */

  diag_msg.number_of_corners_out          = feature_set_corners.features_standalone.size();
  diag_msg.number_of_surfs_out            = feature_set_surfs.features_standalone.size();
  diag_msg.number_of_features_out         = diag_msg.number_of_corners_out + diag_msg.number_of_surfs_out;
  diag_msg.corners_resolution             = _resolution_corners;
  diag_msg.surfs_resolution               = _resolution_surfs;
  diag_msg.corners_keep_percentile        = _corners_keep_percentile;
  diag_msg.surfs_keep_percentile          = _surfs_keep_percentile;
  diag_msg.corners_gradient_mean          = feature_set_corners.grad_mean;
  diag_msg.surfs_gradient_mean            = feature_set_surfs.grad_mean;
  diag_msg.corners_gradient_stddev        = feature_set_corners.grad_stddev;
  diag_msg.surfs_gradient_stddev          = feature_set_surfs.grad_stddev;
  diag_msg.corners_selected_gradients     = getGradients(feature_set_corners.features_standalone);
  diag_msg.surfs_selected_gradients       = getGradients(feature_set_surfs.features_standalone);
  diag_msg.corners_others_gradients       = getGradients(feature_set_corners.features_coupled_sorted);
  diag_msg.surfs_others_gradients         = getGradients(feature_set_surfs.features_coupled_sorted);
  diag_msg.sizeof_features_corners_kb_out = (feature_set_corners.features_standalone.size() * POINT_SIZE) / 1024.0f;
  diag_msg.sizeof_features_surfs_kb_out   = (feature_set_surfs.features_standalone.size() * POINT_SIZE) / 1024.0f;

  extracted_features->aloam_diag_msg->feature_selection = diag_msg;

  const auto selected_corners = featuresToCloud(extracted_features, feature_set_corners.features_standalone, false);
  const auto selected_surfs   = featuresToCloud(extracted_features, feature_set_surfs.features_standalone, false);

  // TODO: remove this redundancy after testing
  const auto selected_corners_viz = featuresToCloud(extracted_features, feature_set_corners.features_standalone, true);
  const auto selected_surfs_viz   = featuresToCloud(extracted_features, feature_set_surfs.features_standalone, true);

  // Publish selected features
  publishCloud(_pub_features_corners_selected, selected_corners_viz);
  publishCloud(_pub_features_surfs_selected, selected_surfs_viz);

  /* ROS_WARN_THROTTLE(0.5, "[FeatureSelection] feature selection of corners and surfs: %0.1f ms", diag_msg.corners_time_ms + diag_msg.surfs_time_ms); */

  return std::make_tuple(selected_corners, selected_surfs, _resolution_corners, _resolution_surfs);
}
/*//}*/

/*//{ selectFeaturesFromCloudByGradient() */
FEATURE_SET FeatureSelection::selectFeaturesFromCloudByGradient(const std::shared_ptr<ExtractedFeatures> &    extracted_features,
                                                                const unsigned int &                          features_count,
                                                                const std::vector<std::vector<unsigned int>> &indices_in_filt, const float &search_radius,
                                                                const float &percentile, const bool &keep_standalone, const unsigned int &number_of_segments) {

  FEATURE_SET out;

  if (features_count == 0) {
    return out;
  }

  FEATURE_VEC  features_selected(features_count);
  FEATURE_VEC  features_others(features_count);
  unsigned int features_selected_ptr = 0;
  unsigned int features_others_ptr   = 0;
  float        mean                  = 0.0f;
  float        stddev                = 0.0f;

  const LOCAL_FEATURE_SETS local_feature_sets =
      estimateGradients(extracted_features, features_count, indices_in_filt, search_radius, keep_standalone, number_of_segments);

  for (const auto &feature_set : local_feature_sets.feature_sets) {

    // Select standalone features (no neighbors)
    if (keep_standalone) {
      features_selected.insert(features_selected.begin() + features_selected_ptr, feature_set.features_standalone.begin(),
                               feature_set.features_standalone.end());
      features_selected_ptr += feature_set.features_standalone.size();
    }

    // Select features with gradient above given percentile
    if (!feature_set.features_coupled_sorted.empty()) {
      const float grad_cutoff_thrd = feature_set.features_coupled_sorted.at(std::floor(percentile * feature_set.features_coupled_sorted.size())).gradient;

      for (const auto &f : feature_set.features_coupled_sorted) {

        // TODO: always keep atleast min_feature_count in sorted features
        if (f.gradient >= grad_cutoff_thrd) {
          features_selected.at(features_selected_ptr++) = f;
        } else {
          features_others.at(features_others_ptr++) = f;
        }
      }
    } else {
      ROS_WARN("[FeatureSelection] Gradients size is 0.");
    }

    mean += feature_set.grad_mean;
    stddev += feature_set.grad_stddev;
  }

  if (features_selected_ptr != features_count) {
    features_selected.resize(features_selected_ptr);
  }
  if (features_others_ptr != features_count) {
    features_others.resize(features_others_ptr);
  }

  out.features_standalone     = features_selected;
  out.features_coupled_sorted = features_others;

  out.grad_mean   = mean / float(number_of_segments);
  out.grad_stddev = stddev / float(number_of_segments);
  /* out.idx_row_from = feature_set.idx_row_from; */
  /* out.idx_row_to   = feature_set.idx_row_to; */

  return out;
}
/*//}*/

/*//{ estimateGradients() */
LOCAL_FEATURE_SETS FeatureSelection::estimateGradients(const std::shared_ptr<ExtractedFeatures> &extracted_features, const unsigned int &features_count,
                                                       const std::vector<std::vector<unsigned int>> &indices_in_filt, const float &search_radius,
                                                       const bool &keep_standalone, const unsigned int &number_of_segments) {

  // DEBUG
  /* for (unsigned int i = 0; i < indices_in_full_res.size(); i++) { */
  /*   ROS_ERROR("RING: %d, start index: %d, features: %ld", i, indices_in_full_res.at(i).first, indices_in_full_res.at(i).second.size()); */
  /*   for (int j = 0; j < 10; j++) { */
  /*     ROS_WARN("- %d", indices_in_full_res.at(i).second.at(j)); */
  /*   } */
  /* } */

  // Precompute neneighbors for each feature in cloud
  std::unordered_map<unsigned int, std::vector<Eigen::Vector3f>> neighbors = getNeighborsInBB(extracted_features, indices_in_filt, search_radius);

  // Prepare return struct
  LOCAL_FEATURE_SETS local_feature_sets;
  local_feature_sets.number_of_local_segments = number_of_segments;
  local_feature_sets.feature_sets.resize(number_of_segments);
  std::vector<unsigned int> coupled_features_counter(number_of_segments, 0);
  std::vector<unsigned int> standalone_features_counter(number_of_segments, 0);
  std::vector<float>        max_gradients(number_of_segments, 0.0f);
  std::vector<float>        mean_gradients(number_of_segments, 0.0f);
  std::vector<float>        std_gradients(number_of_segments, 0.0f);

  const unsigned int seg_step                  = ROW_SIZE / number_of_segments;
  const unsigned int max_segment_feature_count = indices_in_filt.size() * seg_step;  // number of rows * max feature count per segment in row
  for (unsigned int i = 0; i < number_of_segments; i++) {
    local_feature_sets.feature_sets.at(i).features_standalone.resize(max_segment_feature_count);
    local_feature_sets.feature_sets.at(i).features_coupled_sorted.resize(max_segment_feature_count);
    local_feature_sets.feature_sets.at(i).idx_row_from = i * seg_step;
    local_feature_sets.feature_sets.at(i).idx_row_to   = (i + 1) * seg_step;
  }

  // Iterate through input cloud
  /* ROS_ERROR("----"); */
  for (unsigned int i = 0; i < indices_in_filt.size(); i++) {
    /* ROS_ERROR("i: %d", i); */
    for (unsigned int j = 0; j < indices_in_filt.at(i).size(); j++) {
      const unsigned int idx    = indices_in_filt.at(i).at(j);
      const unsigned int seg_no = (idx % ROW_SIZE) / seg_step;
      /* ROS_ERROR_COND(j % 2 == 0, "  j: %d, idx: %d", j, idx); */

      FEATURE f;
      f.idx_in_filt_cloud = idx;
      f.local_segment     = seg_no;

      if (neighbors[idx].empty()) {

        if (keep_standalone) {
          f.gradient                                                                                                  = 1.0f;
          local_feature_sets.feature_sets.at(seg_no).features_standalone.at(standalone_features_counter.at(seg_no)++) = f;
        } else {
          local_feature_sets.feature_sets.at(seg_no).features_coupled_sorted.at(coupled_features_counter.at(seg_no)++) = f;
        }
      } else {

        // Compute gradient
        const Eigen::Vector3f point_xyz =
            Eigen::Vector3f(extracted_features->cloud_filt->at(idx).x, extracted_features->cloud_filt->at(idx).y, extracted_features->cloud_filt->at(idx).z);
        Eigen::Vector3f gradient = Eigen::Vector3f::Zero();
        for (const auto &neighbor : neighbors[idx]) {
          gradient += neighbor - point_xyz;
        }

        // Sum gradients for mean estimation
        f.gradient = (gradient / float(neighbors[idx].size())).norm();

        max_gradients.at(seg_no) = std::fmax(max_gradients.at(seg_no), f.gradient);

        local_feature_sets.feature_sets.at(seg_no).features_coupled_sorted.at(coupled_features_counter.at(seg_no)++) = f;
      }
    }
  }

  // Resize feature vectors, estimate mean & stddev, sort coupled gradients
  for (unsigned int i = 0; i < number_of_segments; i++) {

    if (coupled_features_counter.at(i) != max_segment_feature_count) {
      local_feature_sets.feature_sets.at(i).features_coupled_sorted.resize(coupled_features_counter.at(i));
    }
    if (standalone_features_counter.at(i) != max_segment_feature_count) {
      local_feature_sets.feature_sets.at(i).features_standalone.resize(standalone_features_counter.at(i));
    }

    for (auto &f : local_feature_sets.feature_sets.at(i).features_coupled_sorted) {
      f.gradient /= max_gradients.at(i);
      mean_gradients.at(i) += f.gradient;
    }

    // Estimate mean
    mean_gradients.at(i) /= float(coupled_features_counter.at(i));

    // Estimate standard deviation
    for (auto &f : local_feature_sets.feature_sets.at(i).features_coupled_sorted) {
      std_gradients.at(i) += std::pow(f.gradient - mean_gradients.at(i), 2);
    }
    std_gradients.at(i) = std::sqrt(std_gradients.at(i) / float(coupled_features_counter.at(i)));

    // Sort gradients in non-ascending order
    std::sort(local_feature_sets.feature_sets.at(i).features_coupled_sorted.begin(), local_feature_sets.feature_sets.at(i).features_coupled_sorted.end(),
              [&](const FEATURE &i, const FEATURE &j) { return i.gradient > j.gradient; });
  }


  /* const float time_mean_std_estimation = t_tmp.toc() - time_first_iteration; */
  /* ROS_WARN("[FeatureExtractor]: Select features timing -> finding neighbors: %0.1f ms, finding gradients: %0.1f ms, mean/std estimation: %0.1f ms", */
  /*          time_build_tree, time_first_iteration, time_mean_std_estimation); */

  return local_feature_sets;
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
  /*     standalone_points_indices.at(standalone_feature_count++) = i; */
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
                                                                  const std::vector<FEATURE> &feature_vec, const bool &color_by_segment) {

  pcl::PointCloud<PointType>::Ptr cloud = boost::make_shared<pcl::PointCloud<PointType>>();
  cloud->header                         = extracted_features->cloud_filt->header;
  cloud->width                          = 1;
  cloud->height                         = feature_vec.size();
  cloud->is_dense                       = true;

  cloud->resize(feature_vec.size());
  for (unsigned int i = 0; i < feature_vec.size(); i++) {
    const PointType p = extracted_features->cloud_filt->at(feature_vec.at(i).idx_in_filt_cloud);
    cloud->at(i)      = p;
    if (color_by_segment) {
      cloud->at(i).intensity = feature_vec.at(i).local_segment;
    }
  }

  return cloud;
}

/*//}*/

}  // namespace aloam_slam
