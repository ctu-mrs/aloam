#include "aloam_slam/feature_extractor.h"

namespace aloam_slam
{

/*//{ FeatureExtractor() */
FeatureExtractor::FeatureExtractor(const std::shared_ptr<CommonHandlers_t> handlers, const std::shared_ptr<AloamOdometry> aloam_odometry) {

  _handlers        = handlers;
  _aloam_odometry  = aloam_odometry;
  _scan_period_sec = 1.0f / _handlers->frequency;

  _has_required_parameters = _scan_period_sec > 0.0f && _handlers->scan_lines > 0;

  if (_has_required_parameters) {

    _initialization_frames_delay = int(1.0 / _scan_period_sec);

    // Setup feature extraction library
    const std::shared_ptr<feature_selection::FeatureSelection> fs_ptr = std::make_shared<feature_selection::FeatureSelection>();
    fs_ptr->initialize(*handlers->param_loader, handlers->scan_lines, handlers->samples_per_row, handlers->vfov);

    _fe_edge_plane_ = std::make_unique<feature_extraction::LidarExtractionEdgePlane>(*handlers->param_loader, fs_ptr);

  } else {

    _sub_input_data_processing_diag =
        handlers->nh.subscribe("input_proc_diag_in", 1, &FeatureExtractor::callbackInputDataProcDiag, this, ros::TransportHints().tcpNoDelay());
  }

  // Setup subsribers and publishers
  _sub_points_ = handlers->nh.subscribe("laser_cloud_in", 1, &FeatureExtractor::callbackPointCloud, this, ros::TransportHints().tcpNoDelay());

  _pub_points_finite_            = handlers->nh.advertise<sensor_msgs::PointCloud2>("points_out", 1);
  _pub_features_edges_           = handlers->nh.advertise<sensor_msgs::PointCloud2>("edges_out", 1);
  _pub_features_planes_          = handlers->nh.advertise<sensor_msgs::PointCloud2>("planes_out", 1);
  _pub_features_edges_salient_   = handlers->nh.advertise<sensor_msgs::PointCloud2>("edges_salient_out", 1);
  _pub_features_planes_salient_  = handlers->nh.advertise<sensor_msgs::PointCloud2>("planes_salient_out", 1);
  _pub_features_edges_selected_  = handlers->nh.advertise<sensor_msgs::PointCloud2>("edges_selected_out", 1);
  _pub_features_planes_selected_ = handlers->nh.advertise<sensor_msgs::PointCloud2>("planes_selected_out", 1);
}
/*//}*/

/*//{ callbackPointCloud() */
void FeatureExtractor::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {

  if (!is_initialized) {
    return;
  }

  if (!_has_required_parameters) {
    ROS_WARN("[AloamFeatureExtractor] Not all parameters loaded from config. Waiting for msg on topic (%s) to read them.",
             _sub_input_data_processing_diag.getTopic().c_str());
    return;
  }

  ROS_INFO_ONCE("[AloamFeatureExtractor] Received first point cloud message.");

  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("ALOAM::FeatureExtraction::callbackLaserCloud", _handlers->scope_timer_logger, _handlers->enable_scope_timer);
  mrs_lib::Routine    profiler_routine = _handlers->profiler->createRoutine("callbackLaserCloud");

  if (msg->height == 0 || msg->width == 0) {
    ROS_WARN("[AloamFeatureExtractor]: Received empty laser cloud msg. Skipping frame.");
    return;
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
    return;
  }

  // Convert to PCL type
  const pcl::PointCloud<PointTypeOS>::Ptr cloud = boost::make_shared<pcl::PointCloud<PointTypeOS>>();
  pcl::fromROSMsg(*msg, *cloud);

  // Extract features from the given cloud
  const auto &fe = _fe_edge_plane_->computeFeatures(cloud);

  if (!fe.success) {

    ROS_ERROR_THROTTLE(1.0, "%s", fe.error_msg.c_str());
    return;
  }

  // Publish points/features if needed
  publishCloud(_pub_points_finite_, cloud, fe.indices_finite);
  publishCloud(_pub_features_edges_, cloud, fe.indices_edges);
  publishCloud(_pub_features_planes_, cloud, fe.indices_planes);
  publishCloud(_pub_features_edges_salient_, cloud, fe.indices_edges_salient);
  publishCloud(_pub_features_planes_salient_, cloud, fe.indices_planes_salient);

  if (fe.selection_enabled && fe.selection_success) {
    publishCloud(_pub_features_edges_selected_, cloud, fe.indices_edges_selected);
    publishCloud(_pub_features_planes_selected_, cloud, fe.indices_planes_selected);
  }

  const std::shared_ptr<OdometryData> odometry_data = std::make_shared<OdometryData>();

  odometry_data->diagnostics_fe = fe.fe_diag_msg;

  std::uint64_t stamp_pcl;
  pcl_conversions::toPCL(msg->header.stamp, stamp_pcl);

  odometry_data->stamp_ros           = msg->header.stamp;
  odometry_data->stamp_pcl           = stamp_pcl;
  odometry_data->finite_points_count = fe.indices_finite->size();

  odometry_data->manager_corners_sharp = std::make_shared<feature_selection::FSCloudManager>(cloud, fe.indices_edges_salient);
  odometry_data->manager_surfs_flat    = std::make_shared<feature_selection::FSCloudManager>(cloud, fe.indices_planes_salient);

  if (fe.selection_enabled && fe.selection_success) {
    odometry_data->manager_corners_less_sharp = std::make_shared<feature_selection::FSCloudManager>(cloud, fe.indices_edges_selected);
    odometry_data->manager_surfs_less_flat    = std::make_shared<feature_selection::FSCloudManager>(cloud, fe.indices_planes_selected);
  } else {
    odometry_data->manager_corners_less_sharp = std::make_shared<feature_selection::FSCloudManager>(cloud, fe.indices_edges);
    odometry_data->manager_surfs_less_flat    = std::make_shared<feature_selection::FSCloudManager>(cloud, fe.indices_planes);
  }

  _aloam_odometry->setData(odometry_data);
}
/*//}*/

/*//{ callbackInputDataProcDiag */
void FeatureExtractor::callbackInputDataProcDiag(const mrs_msgs::PclToolsDiagnosticsConstPtr &msg) {

  if (_has_required_parameters || msg->sensor_type != mrs_msgs::PclToolsDiagnostics::SENSOR_TYPE_LIDAR_3D) {
    return;
  }

  _handlers->scan_lines = msg->rows_after;
  _scan_period_sec      = 1.0f / msg->frequency;

  _initialization_frames_delay = int(msg->frequency);  // 1 second delay
  _has_required_parameters     = _scan_period_sec > 0.0f && _handlers->scan_lines > 0;

  if (_has_required_parameters) {
    _aloam_odometry->setFrequency(msg->frequency);

    const std::shared_ptr<feature_selection::FeatureSelection> fs_ptr = std::make_shared<feature_selection::FeatureSelection>();
    fs_ptr->initialize(*_handlers->param_loader, msg->rows_after, msg->cols_after, msg->vfov);

    _fe_edge_plane_ = std::make_unique<feature_extraction::LidarExtractionEdgePlane>(*_handlers->param_loader, fs_ptr);

    _sub_input_data_processing_diag.shutdown();
  }

  ROS_INFO("[AloamFeatureExtractor] Received input data diagnostics: VFoV (%.1f deg), rings count (%d), frequency (%.1f Hz)", msg->vfov, msg->rows_after,
           msg->frequency);
}
/*//}*/

/*//{ hasField() */
bool FeatureExtractor::hasField(const std::string field, const sensor_msgs::PointCloud2::ConstPtr &msg) {
  for (auto f : msg->fields) {
    if (f.name == field) {
      return true;
    }
  }
  return false;
}
/*//}*/

/*//{ publishCloud() */
void FeatureExtractor::publishCloud(const ros::Publisher &pub, const pcl::PointCloud<PointTypeOS>::Ptr cloud, const feature_extraction::indices_ptr_t indices) {
  if (cloud && pub.getNumSubscribers() > 0) {

    const sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();

    if (indices) {

      const auto subcloud = _fe_edge_plane_->toCloud(cloud, indices);
      pcl::toROSMsg(*subcloud, *msg);

    } else {

      pcl::toROSMsg(*cloud, *msg);
    }

    try {
      pub.publish(msg);
    }
    catch (...) {
    }
  }
}
/*//}*/

}  // namespace aloam_slam
