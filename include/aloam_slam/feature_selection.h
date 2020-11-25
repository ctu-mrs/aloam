#pragma once

/* #include "aloam_slam/feature_extractor.h" */
#include "aloam_slam/mapping.h"
#include <pcl/search/organized.h>

namespace aloam_slam
{
class FeatureSelection {

public:
  FeatureSelection(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader);

  std::tuple<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr, float, float> selectFeatures(
      const std::shared_ptr<ExtractedFeatures> &extracted_features);

  bool is_initialized = false;

  std::shared_ptr<LUT> lut;

private:
  ros::Publisher _pub_features_corners_selected;
  ros::Publisher _pub_features_surfs_selected;

  float _resolution_corners;
  float _resolution_surfs;
  float _resolution_corners_min;
  float _resolution_corners_max;
  float _resolution_surfs_min;
  float _resolution_surfs_max;

  bool  _features_selection_enabled;
  float _features_min_count_percent;
  float _corners_keep_percentile;
  float _surfs_keep_percentile;

  const double       HFOV_RESOLUTION          = 0.00613592315;
  const double       VFOV_RESOLUTION          = 0.03862995413;
  const unsigned int ROW_SIZE                 = 1024;
  const double       TAN_HFOV                 = std::tan(HFOV_RESOLUTION);
  const double       TAN_VFOV                 = std::tan(VFOV_RESOLUTION);
  const float        NEIGH_IDX_PREC_RANGE_RES = 0.2f;

  // sample -> vector<tuple(v_idx, vector<h_idx>)>
  std::vector<std::vector<std::pair<unsigned int, unsigned int>>> _neigh_idxs_rows_cols;
  /* float _features_corners_gradient_limit_upper; */
  /* float _features_corners_gradient_limit_bottom; */
  /* float _features_surfs_gradient_limit_upper; */
  /* float _features_surfs_gradient_limit_bottom; */

  std::tuple<pcl::PointCloud<PointType>::Ptr, std::vector<float>, float, float, float> selectFeaturesFromCloudByGradient(
      const std::shared_ptr<ExtractedFeatures> &extracted_features, const unsigned int &features_count,
      const std::vector<std::vector<unsigned int>> &indices_in_filt, const float &search_radius, const float &percentile);

  std::tuple<std::vector<unsigned int>, std::vector<std::pair<unsigned int, float>>, float, float> estimateGradients(
      const std::shared_ptr<ExtractedFeatures> &extracted_features, const unsigned int &features_count,
      const std::vector<std::vector<unsigned int>> &indices_in_filt, const float &search_radius);

  std::unordered_map<unsigned int, std::vector<Eigen::Vector3f>> getNeighborsInBB(const std::shared_ptr<ExtractedFeatures> &    extracted_features,
                                                                                  const std::vector<std::vector<unsigned int>> &indices_in_filt,
                                                                                  const float &                                 max_range);

  std::unordered_map<unsigned int, std::vector<Eigen::Vector3f>> getNeighbors(const std::shared_ptr<ExtractedFeatures> &    extracted_features,
                                                                              const std::vector<std::vector<unsigned int>> &indices_in_filt,
                                                                              const float &                                 max_range);

  void fillRowNeighbors(std::unordered_map<unsigned int, std::vector<Eigen::Vector3f>> &neighbors, const pcl::PointCloud<PointType>::Ptr &cloud_filt,
                        const std::vector<unsigned int> &row_features_idxs, const float &max_range_sq);
  void fillColNeighbors(std::unordered_map<unsigned int, std::vector<Eigen::Vector3f>> &neighbors, const std::shared_ptr<ExtractedFeatures> &extracted_features,
                        const std::vector<std::vector<unsigned int>> &indices_in_filt, const float &max_range_sq);

  void getNeighborsKdTree();

  float estimateResolution(const float &percent, const std::vector<float> &fnc_sorted, const float &min_res, const float &max_res);

  std::vector<std::pair<unsigned int, unsigned int>> getNearestNeighborLimits(const float &point_distance);

  void publishCloud(ros::Publisher publisher, const pcl::PointCloud<PointType>::Ptr cloud);

  const size_t POINT_SIZE = sizeof(PointType);
};
}  // namespace aloam_slam
