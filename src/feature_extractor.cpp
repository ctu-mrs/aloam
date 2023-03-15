#include "aloam_slam/feature_extractor.h"

namespace aloam_slam
{

/*//{ FeatureExtractor() */
template <class T_pt>
FeatureExtractor<T_pt>::FeatureExtractor(const std::shared_ptr<CommonHandlers_t> handlers, const std::shared_ptr<AloamOdometry<T_pt>> aloam_odometry) {

  _handlers        = handlers;
  _aloam_odometry  = aloam_odometry;
  _scan_period_sec = 1.0f / _handlers->frequency;

  _has_required_parameters = _scan_period_sec > 0.0f;

  if (_has_required_parameters) {

    // 0.5 sec delay
    _initialization_frames_delay = int(0.5 / _scan_period_sec);

    // Setup feature extraction library
    _fe_edge_plane_ = std::make_unique<feature_extraction::LidarExtractionEdgePlane>(*handlers->param_loader, handlers->overwrite_intensity_with_curvature);

  } else {

    _sub_input_data_processing_diag =
        handlers->nh.subscribe("input_proc_diag_in", 1, &FeatureExtractor::callbackInputDataProcDiag, this, ros::TransportHints().tcpNoDelay());
  }

  // Subscribers
  if (!handlers->offline_run) {
    _sub_points_ = handlers->nh.subscribe("laser_cloud_in", 1, &FeatureExtractor::callbackPointCloud, this, ros::TransportHints().tcpNoDelay());
    ROS_INFO("PES: subscribing to: %s", _sub_points_.getTopic().c_str());
  }

  // Publishers
  _pub_points_finite_           = handlers->nh.advertise<sensor_msgs::PointCloud2>("points_out", 1);
  _pub_features_edges_          = handlers->nh.advertise<sensor_msgs::PointCloud2>("edges_out", 1);
  _pub_features_planes_         = handlers->nh.advertise<sensor_msgs::PointCloud2>("planes_out", 1);
  _pub_features_edges_salient_  = handlers->nh.advertise<sensor_msgs::PointCloud2>("edges_salient_out", 1);
  _pub_features_planes_salient_ = handlers->nh.advertise<sensor_msgs::PointCloud2>("planes_salient_out", 1);
}
/*//}*/

/*//{ extractFeatures() */
template <class T_pt>
bool FeatureExtractor<T_pt>::extractFeatures(const sensor_msgs::PointCloud2::ConstPtr msg, const std::shared_ptr<FECloudManagersOut_t<T_pt>> managers) {

  if (!is_initialized) {
    ROS_WARN_THROTTLE(1.0, "[AloamFeatureExtractor] Calling uninitialized object.");
    return false;
  }

  if (!_has_required_parameters) {
    ROS_WARN("[AloamFeatureExtractor] Not all parameters loaded from config. Waiting for msg on topic (%s) to read them.",
             _sub_input_data_processing_diag.getTopic().c_str());
    return false;
  }

  ROS_INFO_ONCE("[AloamFeatureExtractor] Received first point cloud message.");

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("ALOAM::FeatureExtraction::callbackLaserCloud", _handlers->scope_timer_logger, _handlers->enable_scope_timer);
  mrs_lib::Routine    profiler_routine = _handlers->profiler->createRoutine("callbackLaserCloud");

  if (msg->height == 0 || msg->width == 0) {
    ROS_WARN("[AloamFeatureExtractor]: Received empty laser cloud msg. Skipping frame.");
    return false;
  }

  // Skip 1 sec of valid data
  ROS_INFO_ONCE("[AloamFeatureExtractor]: Received first laser cloud msg.");
  if (_frame_count++ < _initialization_frames_delay) {
    if (_frame_count == 1) {
      if (hasField("ring", msg)) {
        ROS_INFO("[AloamFeatureExtractor] Laser cloud msg contains field `ring`. Will use this information for data processing.");
      } else {
        ROS_ERROR("[AloamFeatureExtractor] Laser cloud msg does not contain field `ring`. Ending.");
        ros::shutdown();
      }
    }
    return false;
  }

  // Convert to PCL type
  const boost::shared_ptr<pcl::PointCloud<T_pt>> cloud = boost::make_shared<pcl::PointCloud<T_pt>>();
  pcl::fromROSMsg(*msg, *cloud);

  // Extract features from the given cloud
  feature_extraction::LidarExtractionEdgePlaneOutput_t<T_pt> fe;
  _fe_edge_plane_->computeFeatures(fe, cloud);

  if (!fe.success) {

    ROS_ERROR_THROTTLE(1.0, "%s", fe.error_msg.c_str());
    return false;
  }

  const std::shared_ptr<OdometryData<T_pt>> odometry_data = std::make_shared<OdometryData<T_pt>>();

  odometry_data->diagnostics_fe = fe.fe_diag_msg;

  std::uint64_t stamp_pcl;
  pcl_conversions::toPCL(msg->header.stamp, stamp_pcl);

  odometry_data->stamp_ros           = msg->header.stamp;
  odometry_data->stamp_pcl           = stamp_pcl;
  odometry_data->finite_points_count = fe.cloud->size();

  odometry_data->manager_corners_extracted = std::make_shared<feature_selection::FSCloudManager<T_pt>>(fe.cloud, fe.indices_edges);
  odometry_data->manager_surfs_extracted   = std::make_shared<feature_selection::FSCloudManager<T_pt>>(fe.cloud, fe.indices_planes);

  odometry_data->manager_corners_salient = std::make_shared<feature_selection::FSCloudManager<T_pt>>(fe.cloud, fe.indices_edges_salient);
  odometry_data->manager_surfs_salient   = std::make_shared<feature_selection::FSCloudManager<T_pt>>(fe.cloud, fe.indices_planes_salient);

  if (managers) {
    managers->man_edges_salient    = odometry_data->manager_corners_salient;
    managers->man_planes_salient   = odometry_data->manager_surfs_salient;
    managers->man_edges_extracted  = odometry_data->manager_corners_extracted;
    managers->man_planes_extracted = odometry_data->manager_surfs_extracted;
  }

  // Publish points/features if needed
  publishCloud(_pub_points_finite_, fe.cloud);
  publishCloud(_pub_features_edges_, odometry_data->manager_corners_extracted);
  publishCloud(_pub_features_planes_, odometry_data->manager_surfs_extracted);
  publishCloud(_pub_features_edges_salient_, odometry_data->manager_corners_salient);
  publishCloud(_pub_features_planes_salient_, odometry_data->manager_surfs_salient);

  _aloam_odometry->setData(odometry_data);

  return true;
}
/*//}*/

/*//{ callbackPointCloud() */
template <class T_pt>
void FeatureExtractor<T_pt>::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr msg) {
  extractFeatures(msg, nullptr);
}
/*//}*/

/*//{ callbackInputDataProcDiag */
template <class T_pt>
void FeatureExtractor<T_pt>::callbackInputDataProcDiag(const mrs_msgs::PclToolsDiagnosticsConstPtr &msg) {

  if (_has_required_parameters || msg->sensor_type != mrs_msgs::PclToolsDiagnostics::SENSOR_TYPE_LIDAR_3D) {
    return;
  }

  _scan_period_sec = 1.0f / msg->frequency;

  _initialization_frames_delay = int(msg->frequency / 2.0);  // 0.5 second delay
  _has_required_parameters     = _scan_period_sec > 0.0f;

  if (_has_required_parameters) {
    _aloam_odometry->setFrequency(msg->frequency);
    _fe_edge_plane_ = std::make_unique<feature_extraction::LidarExtractionEdgePlane>(*_handlers->param_loader, _handlers->overwrite_intensity_with_curvature);

    _sub_input_data_processing_diag.shutdown();
  }

  ROS_INFO("[AloamFeatureExtractor] Received input data diagnostics: rings count (%d), frequency (%.1f Hz)", msg->rows_after, msg->frequency);
}
/*//}*/

/*//{ hasField() */
template <class T_pt>
bool FeatureExtractor<T_pt>::hasField(const std::string field, const sensor_msgs::PointCloud2::ConstPtr &msg) {
  for (auto f : msg->fields) {
    if (f.name == field) {
      return true;
    }
  }
  return false;
}
/*//}*/

// Explicit template instantiation
template struct FECloudManagersOut_t<ouster_ros::Point>;
template struct FECloudManagersOut_t<pandar_pointcloud::PointXYZIT>;

template class FeatureExtractor<ouster_ros::Point>;
template class FeatureExtractor<pandar_pointcloud::PointXYZIT>;

}  // namespace aloam_slam
