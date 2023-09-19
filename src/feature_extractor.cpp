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

  _tf_buffer   = std::make_shared<tf2_ros::Buffer>();
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

  // Publishers
  _pub_points_finite_           = handlers->nh.advertise<sensor_msgs::PointCloud2>("points_out", 1);
  _pub_features_edges_          = handlers->nh.advertise<sensor_msgs::PointCloud2>("edges_out", 1);
  _pub_features_planes_         = handlers->nh.advertise<sensor_msgs::PointCloud2>("planes_out", 1);
  _pub_features_edges_salient_  = handlers->nh.advertise<sensor_msgs::PointCloud2>("edges_salient_out", 1);
  _pub_features_planes_salient_ = handlers->nh.advertise<sensor_msgs::PointCloud2>("planes_salient_out", 1);
  _pub_range_diff_              = handlers->nh.advertise<sensor_msgs::PointCloud2>("cloud_range_diff_out", 1);
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

  if (_publish_cloud_diff) {
    publishRangeDiff(cloud, msg->header.stamp);
  }

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
void FeatureExtractor<T_pt>::callbackInputDataProcDiag(const mrs_msgs::PclToolsDiagnosticsConstPtr& msg) {

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

/*//{ publishRangeDiff */
template <class T_pt>
void FeatureExtractor<T_pt>::publishRangeDiff(const boost::shared_ptr<pcl::PointCloud<T_pt>> cloud, const ros::Time& stamp) {

  if (cloud->is_dense) {
    ROS_WARN("[AloamFeatureExtractor::publishRangeDiff] Input point cloud is not ordered. Can't compute cloud diff. Switching it off.");
    _publish_cloud_diff = false;
    return;
  }

  // DEBUG: look for rotation in /tf
  Eigen::Matrix3d R_k;
  try {
    const geometry_msgs::TransformStamped transformStamped = _tf_buffer->lookupTransform("world", cloud->header.frame_id, stamp);

    // Get current rotation
    R_k = mrs_lib::AttitudeConverter(transformStamped.transform.rotation);
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  if (_cloud_prev && _pub_range_diff_.getNumSubscribers() > 0) {

    // Get rotation matrix from time k-1 to k
    const Eigen::Matrix3d R_rel = R_k.inverse() * _R_prev;

    /* mrs_lib::AttitudeConverter tmp = mrs_lib::AttitudeConverter(_R_prev); */
    /* ROS_INFO("sensor in world k-1: RPY (%.4f, %.4f, %.4f)", tmp.getRoll(), tmp.getPitch(), tmp.getYaw()); */
    /* tmp = mrs_lib::AttitudeConverter(R_k); */
    /* ROS_INFO("sensor in world k:   RPY (%.4f, %.4f, %.4f)", tmp.getRoll(), tmp.getPitch(), tmp.getYaw()); */
    /* tmp = mrs_lib::AttitudeConverter(R_rel); */
    /* ROS_INFO("sensor in world rel: RPY (%.4f, %.4f, %.4f)", tmp.getRoll(), tmp.getPitch(), tmp.getYaw()); */

    const auto [range_exists, range_offset] = getFieldOffset<T_pt>("range");

    // Setup ordered cloud
    const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud_diff = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloud->width, cloud->height);

    cloud_diff->header   = cloud->header;
    cloud_diff->width    = cloud->width;
    cloud_diff->height   = cloud->height;
    cloud_diff->is_dense = cloud->is_dense;

    const bool ROTATE = false;

    if (ROTATE) {
      const double vfov     = M_PI_2;  // 90 deg
      const double col_step = (2.0 * M_PI) / cloud->width;
      const double row_step = vfov / (cloud->height - 1);

      // Rotate every point and project to grid WxH
      for (unsigned int i = 0; i < _cloud_prev->size(); i++) {
        const auto& p_prev = _cloud_prev->at(i);

        if (!pcl::isFinite(p_prev)) {
          continue;
        }

        // Rotate point by prior rotation estimate
        const Eigen::Vector3d p_rot = R_rel * Eigen::Vector3d(p_prev.x, p_prev.y, p_prev.z);

        const double dx = std::pow(p_rot.x(), 2);
        const double dy = std::pow(p_rot.y(), 2);
        const double dz = std::pow(p_rot.z(), 2);

        // Find spherical coordinates
        const double r = std::sqrt(dx + dy + dz);
        if (r > 0.0f) {
          double    el  = M_PI_2 - std::atan2(std::sqrt(dx + dy), p_rot.z());
          const int row = (int)(cloud->height / 2) + std::floor(el / row_step);

          // Can be projected outside of the vertical fov
          if (row < 0 || row >= cloud->height) {
            continue;
          }

          // Horizontal FoV is cyclic, so projetion is assured to fit in
          double az = std::atan2(p_rot.y(), p_rot.x());
          if (az < 0.0) {
            az += 2.0 * M_PI;
          }
          int col = std::floor(az / col_step);
          if (col >= cloud->width) {
            col -= cloud->width;
          }

          auto& p     = cloud_diff->at(col, row);
          p.x         = float(p_rot.x());
          p.y         = float(p_rot.y());
          p.z         = float(p_rot.z());
          p.intensity = float(r);
        }
      }
    }

    for (unsigned int i = 0; i < cloud->size(); i++) {

      const auto& p_k    = cloud->at(i);
      const auto& p_prev = _cloud_prev->at(i);

      auto& p = cloud_diff->at(i);

      if (!pcl::isFinite(p_k) || !pcl::isFinite(p_prev)) {
        p.x         = std::numeric_limits<float>::quiet_NaN();
        p.y         = std::numeric_limits<float>::quiet_NaN();
        p.z         = std::numeric_limits<float>::quiet_NaN();
        p.intensity = 0.0f;
        continue;
      }

      float range_k;
      if (range_exists) {
        const std::uint32_t range_curr_mm = getFieldValue<T_pt, std::uint32_t>(p_k, range_offset);
        range_k                           = range_curr_mm / 1000.0f;
      } else {
        range_k = float(std::sqrt(std::pow(p_k.x, 2) + std::pow(p_k.y, 2) + std::pow(p_k.z, 2)));
      }

      float range_prev;
      if (ROTATE) {
        // We have range in the intensity already
        range_prev = p.intensity;
      } else if (range_exists) {
        const std::uint32_t range_curr_mm = getFieldValue<T_pt, std::uint32_t>(p_prev, range_offset);
        range_prev                        = range_curr_mm / 1000.0f;
      } else {
        range_prev = float(std::sqrt(std::pow(p_prev.x, 2) + std::pow(p_prev.y, 2) + std::pow(p_prev.z, 2)));
      }

      p.intensity = std::fabs(range_k - range_prev);

      if (p.intensity < 0.1f) {
        // debug filtering
        p.x = std::numeric_limits<float>::quiet_NaN();
        p.y = std::numeric_limits<float>::quiet_NaN();
        p.z = std::numeric_limits<float>::quiet_NaN();
      } else {
        p.intensity = std::log(p.intensity); // for visualization purposes
        p.x         = p_k.x;
        p.y         = p_k.y;
        p.z         = p_k.z;
      }
    }

    /* const float min_diff_val = 0.5f * (max_diff - min_diff) + min_diff; */
    /* ROS_WARN("max_diff: %.5f, min_diff: %.5f, min_diff_val: %.5f", max_diff, min_diff, min_diff_val); */

    /* for (unsigned int i = 0; i < cloud_diff->size(); i++) { */
    /*   auto& p = cloud_diff->at(i); */
    /*   if (p.intensity < min_diff_val) { */
    /*     p.x = std::numeric_limits<float>::quiet_NaN(); */
    /*     p.y = std::numeric_limits<float>::quiet_NaN(); */
    /*     p.z = std::numeric_limits<float>::quiet_NaN(); */
    /*   } */
    /* } */

    const sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloud_diff, *msg);

    try {
      _pub_range_diff_.publish(msg);
    }
    catch (...) {
    }
  }
  _R_prev     = R_k;
  _cloud_prev = cloud;
}
/*//}*/

// Explicit template instantiation
template struct FECloudManagersOut_t<ouster_ros::Point>;
template struct FECloudManagersOut_t<pandar_pointcloud::PointXYZIT>;
/* template struct FECloudManagersOut_t<pcl::PointXYZI>; */

template class FeatureExtractor<ouster_ros::Point>;
template class FeatureExtractor<pandar_pointcloud::PointXYZIT>;
/* template class FeatureExtractor<pcl::PointXYZI>; */

}  // namespace aloam_slam
