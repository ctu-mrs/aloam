#include "aloam_slam/feature_selection.h"

namespace aloam_slam
{

/*//{ FeatureSelection() */
FeatureSelection::FeatureSelection(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader) {

  ros::NodeHandle nh_(parent_nh);
  ros::Time::waitForValid();

  param_loader.loadParam("mapping_line_resolution", _features_corners_resolution, 0.4f);
  param_loader.loadParam("mapping_plane_resolution", _features_surfs_resolution, 0.8f);

  param_loader.loadParam("feature_selection/enable", _features_selection_enabled, false);
  param_loader.loadParam("feature_selection/corners/gradient/limit/upper", _features_corners_gradient_limit_upper, 1.0f);
  param_loader.loadParam("feature_selection/corners/gradient/limit/bottom", _features_corners_gradient_limit_bottom, 0.0f);
  param_loader.loadParam("feature_selection/surfs/gradient/limit/upper", _features_surfs_gradient_limit_upper, 1.0f);
  param_loader.loadParam("feature_selection/surfs/gradient/limit/bottom", _features_surfs_gradient_limit_bottom, 0.0f);

  _pub_features_corners_selected = nh_.advertise<sensor_msgs::PointCloud2>("features_corners_selected_out", 1);
  _pub_features_surfs_selected   = nh_.advertise<sensor_msgs::PointCloud2>("features_surfs_selected_out", 1);
}
/*//}*/

/*//{ selectFeatures() */
pcl::PointCloud<PointType>::Ptr FeatureSelection::selectFeatures(const pcl::PointCloud<PointType>::Ptr &cloud) {
  if (!_features_selection_enabled) {
    return cloud;
  }

  TicToc t_routine;

  // TODO: pass parameters
  pcl::PointCloud<PointType>::Ptr selected_features =
      selectFeaturesFromCloudByGradient(cloud, _features_corners_resolution, _features_corners_gradient_limit_bottom, _features_corners_gradient_limit_upper);

  ROS_WARN_THROTTLE(0.5, "[FeatureSelection] feature selection from single cloud took: %0.1f ms", t_routine.toc());

  return selected_features;
}
/*//}*/

/*//{ selectFeatures() */
std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr> FeatureSelection::selectFeatures(
    const pcl::PointCloud<PointType>::Ptr &corner_points, const pcl::PointCloud<PointType>::Ptr &surf_points) {
  if (!_features_selection_enabled) {
    return std::make_pair(corner_points, surf_points);
  }

  TicToc t_routine;

  pcl::PointCloud<PointType>::Ptr selected_corners = selectFeaturesFromCloudByGradient(
      corner_points, _features_corners_resolution, _features_corners_gradient_limit_bottom, _features_corners_gradient_limit_upper);
  pcl::PointCloud<PointType>::Ptr selected_surfs =
      selectFeaturesFromCloudByGradient(surf_points, _features_surfs_resolution, _features_surfs_gradient_limit_bottom, _features_surfs_gradient_limit_upper);

  ROS_WARN_THROTTLE(0.5, "[FeatureSelection] feature selection of corners and surfs: %0.1f ms", t_routine.toc());

  // TODO: publish some diagnostics
  publishCloud(_pub_features_corners_selected, selected_corners);
  publishCloud(_pub_features_surfs_selected, selected_surfs);

  return std::make_pair(selected_corners, selected_surfs);
}
/*//}*/

/*//{ selectFeaturesFromCloudByGradient() */
pcl::PointCloud<PointType>::Ptr FeatureSelection::selectFeaturesFromCloudByGradient(const pcl::PointCloud<PointType>::Ptr cloud, const float search_radius,
                                                                                    const float grad_min, const float grad_max) {

  pcl::PointCloud<PointType>::Ptr selected_features = boost::make_shared<pcl::PointCloud<PointType>>();

  if (cloud->size() == 0) {
    return selected_features;
  }

  std::vector<float> gradient_norms(cloud->size());
  double             grad_norm_max = -1.0;

  // Create kd tree for fast search, TODO: find neighbors by indexing
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_features;
  kdtree_features.setInputCloud(cloud);

  for (unsigned int i = 0; i < cloud->size(); i++) {

    const pcl::PointXYZI point = cloud->at(i);
    std::vector<int>     indices;
    std::vector<float>   squared_distances;

    // Find all features in given radius, TODO: find neighbors by indexing
    kdtree_features.radiusSearch(point, search_radius, indices, squared_distances);

    Eigen::Vector3f gradient  = Eigen::Vector3f::Zero();
    double          grad_norm = 0.0;

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

  // Filter by gradient value
  for (unsigned int i = 0; i < cloud->size(); i++) {

    const float grad_norm = gradient_norms.at(i) / grad_norm_max;

    if (grad_norm >= grad_min && grad_norm <= grad_max) {
      pcl::PointXYZI point_grad = cloud->at(i);
      point_grad.intensity      = grad_norm;
      selected_features->push_back(point_grad);
    }
  }

  selected_features->header   = cloud->header;
  selected_features->width    = 1;
  selected_features->height   = selected_features->points.size();
  selected_features->is_dense = cloud->is_dense;

  return selected_features;
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
