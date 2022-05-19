#include "aloam_slam/feature_extractor.h"

namespace aloam_slam
{

/*//{ FeatureExtractor() */
FeatureExtractor::FeatureExtractor(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::shared_ptr<mrs_lib::Profiler> profiler,
                                   const std::shared_ptr<AloamOdometry> odometry, const std::string &map_frame, const float scan_period_sec,
                                   const bool enable_scope_timer, const std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger)
    : _profiler(profiler),
      _odometry(odometry),
      _frame_map(map_frame),
      _scan_period_sec(scan_period_sec),
      _scope_timer_logger(scope_timer_logger),
      _enable_scope_timer(enable_scope_timer) {

  ros::NodeHandle nh_(parent_nh);

  ros::Time::waitForValid();

  param_loader.loadParam("vertical_fov", _vertical_fov_half, -1.0f);
  param_loader.loadParam("scan_line", _number_of_rings, -1);

  _has_required_parameters = scan_period_sec > 0.0f && _vertical_fov_half > 0.0f && _number_of_rings > 0;

  if (_has_required_parameters) {
    _ray_vert_delta = _vertical_fov_half / float(_number_of_rings - 1);  // vertical resolution
    _vertical_fov_half /= 2.0;                                           // get half fov

    _initialization_frames_delay = int(1.0 / scan_period_sec);
  } else {
    _sub_input_data_processing_diag =
        nh_.subscribe("input_proc_diag_in", 1, &FeatureExtractor::callbackInputDataProcDiag, this, ros::TransportHints().tcpNoDelay());
  }

  /* _pub_dbg = nh_.advertise<sensor_msgs::PointCloud2>("fe/dbg", 1); */

  mrs_lib::SubscribeHandlerOptions shopts(nh_);
  shopts.node_name          = "FeatureExtractor";
  shopts.no_message_timeout = ros::Duration(5.0);
  _sub_laser_cloud          = mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>(shopts, "laser_cloud_in",
                                                                         std::bind(&FeatureExtractor::callbackLaserCloud, this, std::placeholders::_1));
  /* ROS_INFO_STREAM("[AloamFeatureExtractor]: Listening to laser cloud at topic: " << _sub_laser_cloud.topicName()); */
}
/*//}*/

/*//{ callbackLaserCloud() */
void FeatureExtractor::callbackLaserCloud(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2> &sub) {
  if (!is_initialized || !sub.hasMsg()) {
    return;
  }
  if (!_has_required_parameters) {
    ROS_WARN("[AloamFeatureExtractor] Not all parameters loaded from config. Waiting for msg on topic (%s) to read them.",
             _sub_input_data_processing_diag.getTopic().c_str());
    return;
  }

  const auto laserCloudMsg = sub.getMsg();
  if (laserCloudMsg->data.size() == 0) {
    ROS_WARN("[AloamFeatureExtractor]: Received empty laser cloud msg. Skipping frame.");
    return;
  }
  mrs_lib::ScopeTimer timer            = mrs_lib::ScopeTimer("ALOAM::FeatureExtraction::callbackLaserCloud", _scope_timer_logger, _enable_scope_timer);
  mrs_lib::Routine    profiler_routine = _profiler->createRoutine("callbackLaserCloud");

  // Skip 1s of data
  ROS_INFO_ONCE("[AloamFeatureExtractor]: Received first laser cloud msg.");
  if (_frame_count++ < _initialization_frames_delay) {
    if (_frame_count == 1) {
      if (hasField("ring", laserCloudMsg)) {
        ROS_INFO("[AloamFeatureExtractor] Laser cloud msg contains field `ring`. Will use this information for data processing.");
      } else {
        ROS_ERROR("[AloamFeatureExtractor] Laser cloud msg does not contain field `ring`. Ending.");
        ros::shutdown();
      }
    }
    return;
  }

  // Process input data per row
  const pcl::PointCloud<PointType>::Ptr                 cloud_raw     = boost::make_shared<pcl::PointCloud<PointType>>();
  const pcl::PointCloud<PointType>::Ptr                 cloud_wo_nans = boost::make_shared<pcl::PointCloud<PointType>>();
  std::vector<int>                                      rows_start_index;
  std::vector<int>                                      rows_end_index;
  std::unordered_map<int, feature_selection::IndexPair> indices_map_proc_in_raw;

  rows_start_index.resize(_number_of_rings, 0);
  rows_end_index.resize(_number_of_rings, 0);

  if (!parseRowsFromOusterMsg(laserCloudMsg, cloud_raw, cloud_wo_nans, rows_start_index, rows_end_index, indices_map_proc_in_raw)) {
    return;
  }

  timer.checkpoint("parsing lidar data");

  const auto &points = cloud_wo_nans->points;

  std::vector<float> cloudCurvature;
  std::vector<int>   cloudSortInd;
  std::vector<int>   cloudNeighborPicked;
  std::vector<int>   cloudLabel;

  const unsigned int cloud_size = points.size();
  cloudCurvature.resize(cloud_size);
  cloudSortInd.resize(cloud_size);
  cloudNeighborPicked.resize(cloud_size, 0);
  cloudLabel.resize(cloud_size, 0);

  for (unsigned int i = 5; i < cloud_size - 5; i++) {
    const float diffX = points.at(i - 5).x + points.at(i - 4).x + points.at(i - 3).x + points.at(i - 2).x + points.at(i - 1).x - 10 * points.at(i).x +
                        points.at(i + 1).x + points.at(i + 2).x + points.at(i + 3).x + points.at(i + 4).x + points.at(i + 5).x;
    const float diffY = points.at(i - 5).y + points.at(i - 4).y + points.at(i - 3).y + points.at(i - 2).y + points.at(i - 1).y - 10 * points.at(i).y +
                        points.at(i + 1).y + points.at(i + 2).y + points.at(i + 3).y + points.at(i + 4).y + points.at(i + 5).y;
    const float diffZ = points.at(i - 5).z + points.at(i - 4).z + points.at(i - 3).z + points.at(i - 2).z + points.at(i - 1).z - 10 * points.at(i).z +
                        points.at(i + 1).z + points.at(i + 2).z + points.at(i + 3).z + points.at(i + 4).z + points.at(i + 5).z;

    cloudCurvature.at(i) = diffX * diffX + diffY * diffY + diffZ * diffZ;
    cloudSortInd.at(i)   = i;
  }

  const IndicesPtr indices_corners_sharp      = std::make_shared<feature_selection::Indices_t>();
  const IndicesPtr indices_corners_less_sharp = std::make_shared<feature_selection::Indices_t>();
  const IndicesPtr indices_surfs_flat         = std::make_shared<feature_selection::Indices_t>();
  const IndicesPtr indices_surfs_less_flat    = std::make_shared<feature_selection::Indices_t>();

  indices_corners_sharp->reserve(cloud_size);
  indices_corners_less_sharp->reserve(cloud_size);
  indices_surfs_flat->reserve(cloud_size);
  indices_surfs_less_flat->reserve(cloud_size);

  const static double CURVATURE_THRD = 0.1;

  /*//{ Compute features (planes and edges) in two resolutions */
  for (int i = 0; i < _number_of_rings; i++) {

    if (rows_end_index.at(i) - rows_start_index.at(i) < 6) {
      continue;
    }

    feature_selection::Indices_t indices_surfs_less_flat_scan;
    indices_surfs_less_flat_scan.reserve(rows_end_index.at(i) - rows_start_index.at(i));

    /* pcl::PointCloud<PointType>::Ptr cloud_surfs_less_flat_scan = boost::make_shared<pcl::PointCloud<PointType>>(); */

    for (int j = 0; j < 6; j++) {

      const int sp = rows_start_index.at(i) + (rows_end_index.at(i) - rows_start_index.at(i)) * j / 6;
      const int ep = rows_start_index.at(i) + (rows_end_index.at(i) - rows_start_index.at(i)) * (j + 1) / 6 - 1;

      // Sort feature indices by curvature in ascending order in given 1/6 segment
      std::sort(cloudSortInd.begin() + sp, cloudSortInd.begin() + ep + 1,
                [&cloudCurvature](int i, int j) { return cloudCurvature.at(i) < cloudCurvature.at(j); });

      /*//{ Select edges: highest curvature */
      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        const int ind = cloudSortInd.at(k);

        if (cloudNeighborPicked.at(ind) == 0 && cloudCurvature.at(ind) > CURVATURE_THRD) {

          // Pick max number of corner features in given segment
          largestPickedNum++;
          if (largestPickedNum <= 20) {

            const auto &fs_idx = indices_map_proc_in_raw.at(ind);

            indices_corners_less_sharp->push_back(fs_idx);

            if (largestPickedNum > 2) {

              cloudLabel.at(ind) = 1;

            } else {

              cloudLabel.at(ind) = 2;

              indices_corners_sharp->push_back(fs_idx);
            }

          } else {
            break;
          }

          // Mark this point and its neighbors as non-selectable as feature (to remove very-close features)
          cloudNeighborPicked.at(ind) = 1;

          for (int l = 1; l <= 5; l++) {
            const float diffX = points.at(ind + l).x - points.at(ind + l - 1).x;
            const float diffY = points.at(ind + l).y - points.at(ind + l - 1).y;
            const float diffZ = points.at(ind + l).z - points.at(ind + l - 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }
          for (int l = -1; l >= -5; l--) {
            const float diffX = points.at(ind + l).x - points.at(ind + l + 1).x;
            const float diffY = points.at(ind + l).y - points.at(ind + l + 1).y;
            const float diffZ = points.at(ind + l).z - points.at(ind + l + 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }
        }
      }
      /*//}*/

      /*//{ Select surfs: lowest curvature */
      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        const int ind = cloudSortInd.at(k);

        if (cloudNeighborPicked.at(ind) == 0 && cloudCurvature.at(ind) < CURVATURE_THRD) {

          const auto &fs_idx = indices_map_proc_in_raw.at(ind);

          // Mark first few lowest-curvature points as sharp edges
          cloudLabel.at(ind) = -1;
          indices_surfs_flat->push_back(fs_idx);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          // Mark this point and its neighbors as non-selectable as feature (to remove very-close features)
          cloudNeighborPicked.at(ind) = 1;

          for (int l = 1; l <= 5; l++) {
            const float diffX = points.at(ind + l).x - points.at(ind + l - 1).x;
            const float diffY = points.at(ind + l).y - points.at(ind + l - 1).y;
            const float diffZ = points.at(ind + l).z - points.at(ind + l - 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }

          for (int l = -1; l >= -5; l--) {
            const float diffX = points.at(ind + l).x - points.at(ind + l + 1).x;
            const float diffY = points.at(ind + l).y - points.at(ind + l + 1).y;
            const float diffZ = points.at(ind + l).z - points.at(ind + l + 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }
        }
      }

      // Mark the rest of features as a bit-curvy surfs
      for (int k = sp; k <= ep; k++) {
        if (cloudLabel.at(k) <= 0) {

          const auto &fs_idx = indices_map_proc_in_raw.at(k);
          indices_surfs_less_flat_scan.push_back(fs_idx);

          /* cloud_surfs_less_flat_scan->push_back(points.at(k)); */
        }
      }

      /*//}*/
    }

    // There is now many less-flat surf features: downsample
    pseudoDownsampleIndices(indices_surfs_less_flat, cloud_raw, indices_surfs_less_flat_scan, 0.04);

    /* pcl::VoxelGrid<PointType> vg; */
    /* vg.setInputCloud(cloud_surfs_less_flat_scan); */
    /* vg.setLeafSize(0.2, 0.2, 0.2); */
    /* vg.filter(*cloud_surfs_less_flat_scan); */
    /* *cloud_surfs_less_flat += *cloud_surfs_less_flat_scan; */
  }

  /*//}*/

  timer.checkpoint("parsing features");

  cloud_wo_nans->header.frame_id = _frame_map;

  std::uint64_t stamp_pcl;
  pcl_conversions::toPCL(laserCloudMsg->header.stamp, stamp_pcl);
  cloud_wo_nans->header.stamp = stamp_pcl;

  const std::shared_ptr<OdometryData> odometry_data = std::make_shared<OdometryData>();

  odometry_data->stamp_ros                  = laserCloudMsg->header.stamp;
  odometry_data->stamp_pcl                  = stamp_pcl;
  odometry_data->cloud_raw                  = cloud_raw;
  odometry_data->manager_finite_points      = std::make_shared<CloudManager>(cloud_wo_nans);
  odometry_data->manager_corners_sharp      = std::make_shared<CloudManager>(cloud_raw, indices_corners_sharp);
  odometry_data->manager_corners_less_sharp = std::make_shared<CloudManager>(cloud_raw, indices_corners_less_sharp);
  odometry_data->manager_surfs_flat         = std::make_shared<CloudManager>(cloud_raw, indices_surfs_flat);
  odometry_data->manager_surfs_less_flat    = std::make_shared<CloudManager>(cloud_raw, indices_surfs_less_flat);
  odometry_data->time_feature_extraction    = timer.getLifetime();

  _odometry->setData(odometry_data);

  // TODO: Publish debug visualization msg with extracted features
  /* if (_pub_dbg.getNumSubscribers() > 0) { */
  /*   _pub_dbg.publish(odometry_data->manager_surfs_less_flat->getUnorderedCloudPtr()); */
  /* } */

  /* ROS_INFO_THROTTLE(1.0, "[AloamFeatureExtractor] Run time: %0.1f ms (%0.1f Hz)", time_whole, std::min(1.0f / _scan_period_sec, 1000.0f / time_whole)); */
  /* ROS_DEBUG_THROTTLE(1.0, "[AloamFeatureExtractor] points: %d; preparing data: %0.1f ms; q-sorting data: %0.1f ms; computing features: %0.1f ms",
   * cloud_size,
   */
  /*                    processing_time, time_q_sort, time_pts); */
  /* ROS_WARN_COND(time_whole > _scan_period_sec * 1000.0f, "[AloamFeatureExtractor] Feature extraction took over %0.2f ms", _scan_period_sec * 1000.0f); */
}
/*//}*/

/*//{ callbackInputDataProcDiag */
void FeatureExtractor::callbackInputDataProcDiag(const mrs_msgs::PclToolsDiagnosticsConstPtr &msg) {

  if (_has_required_parameters) {
    _sub_input_data_processing_diag.shutdown();
    return;
  } else if (msg->sensor_type != mrs_msgs::PclToolsDiagnostics::SENSOR_TYPE_LIDAR_3D) {
    return;
  }

  _vertical_fov_half = msg->vfov / 2.0f;
  _number_of_rings   = msg->rows_after;
  _scan_period_sec   = 1.0f / msg->frequency;

  _ray_vert_delta = _vertical_fov_half / float(_number_of_rings - 1);  // vertical resolution
  _vertical_fov_half /= 2.0;                                           // get half fov

  _initialization_frames_delay = int(msg->frequency);  // 1 second delay

  _has_required_parameters = _scan_period_sec > 0.0f && _vertical_fov_half > 0.0f && _number_of_rings > 0;

  ROS_INFO("[AloamFeatureExtractor] Received input data diagnostics: VFoV (%.1f deg), rings count (%d), frequency (%.1f Hz)", msg->vfov, msg->rows_after,
           msg->frequency);
}
/*//}*/

/*//{ parseRowsFromOusterMsg() */
bool FeatureExtractor::parseRowsFromOusterMsg(const sensor_msgs::PointCloud2::ConstPtr cloud, const pcl::PointCloud<PointType>::Ptr cloud_raw,
                                              const pcl::PointCloud<PointType>::Ptr cloud_processed, std::vector<int> &rows_start_index,
                                              std::vector<int> &                                     rows_end_index,
                                              std::unordered_map<int, feature_selection::IndexPair> &indices_map_proc_in_raw) {

  // Convert ROS msg to OS cloud
  const pcl::PointCloud<ouster_ros::Point>::Ptr cloud_os = boost::make_shared<pcl::PointCloud<ouster_ros::Point>>();
  pcl::fromROSMsg(*cloud, *cloud_os);

  // Check if data are valid
  if (!cloud_os->isOrganized()) {
    ROS_ERROR_THROTTLE(1.0, "[AloamFeatureExtractor] Input data are not organized.");
    return false;
  } else if (_number_of_rings != cloud_os->height) {
    ROS_ERROR_THROTTLE(1.0, "[AloamFeatureExtractor] Expected number of rings != input data height.");
    return false;
  }

  // Prepare metadata
  cloud_processed->header = cloud_os->header;
  cloud_raw->header       = cloud_os->header;
  cloud_raw->resize(cloud_os->height * cloud_os->width);
  cloud_raw->height = cloud_os->height;
  cloud_raw->width  = cloud_os->width;

  // Prepare outputs
  rows_start_index.resize(_number_of_rings, 0);
  rows_end_index.resize(_number_of_rings, 0);
  indices_map_proc_in_raw.reserve(cloud_raw->size());

  // Get start and end time of columns
  const uint32_t t_start    = cloud_os->at(0, 0).t;
  const uint32_t t_end      = cloud_os->at(static_cast<int>(cloud_os->width) - 1, 0).t;
  const float    t_duration = static_cast<float>(t_end - t_start);

  int index = 0;

  for (int r = 0; r < cloud_os->height; r++) {

    pcl::PointCloud<PointType> row;
    row.reserve(cloud->width);

    for (int c = 0; c < cloud_os->width; c++) {

      const auto &point_os = cloud_os->at(c, r);

      // TODO: remove after testing
      if (point_os.range < 450) {
        continue;
      }

      if (pcl::isXYZFinite(point_os) && point_os.range > 0) {

        auto &point = cloud_raw->at(c, r);

        point.x = point_os.x;
        point.y = point_os.y;
        point.z = point_os.z;

        // Read row (ring) directly from msg
        const int point_ring = point_os.ring;

        // Remap scan time to <0, 1>
        const float rel_time = static_cast<float>(point_os.t - t_start) / t_duration;

        // Store unique r/c info to intensity (for association purposes)
        point.intensity = static_cast<float>(point_ring) + _scan_period_sec * rel_time;

        row.push_back(point);

        // Store indices map from processed -> raw cloud
        feature_selection::IndexPair fs_idx;
        fs_idx.col = c;
        fs_idx.row = r;

        indices_map_proc_in_raw.insert({index++, fs_idx});
      }
    }

    rows_start_index.at(r) = cloud_processed->size() + 5;
    *cloud_processed += row;
    rows_end_index.at(r) = cloud_processed->size() - 6;
  }

  return true;
}
/*//}*/

/*//{ appendVoxelizedIndices() */
void FeatureExtractor::appendVoxelizedIndices(const IndicesPtr &indices_to_be_appended, const pcl::PointCloud<PointType>::Ptr &cloud_in,
                                              const feature_selection::Indices_t &indices_in_cloud, const float resolution) {

  // IS SO SLOW

  // Has the input dataset been set already?
  if (!cloud_in) {
    ROS_WARN("[FeatureExtractor] No input dataset given!");
    return;
  }
  if (!cloud_in->is_dense) {
    ROS_WARN("[FeatureExtractor] Voxel grid supports only dense point cloud too speed up computation.");
    return;
  }

  const Eigen::Array4f inverse_leaf_size = Eigen::Array4f::Ones() / Eigen::Vector4f(resolution, resolution, resolution, 1).array();

  // Get the minimum and maximum dimensions
  Eigen::Vector4f min_p, max_p;
  pcl::getMinMax3D(*cloud_in, min_p, max_p);

  // Check that the leaf size is not too small, given the size of the data
  const uint64_t dx = static_cast<uint64_t>((max_p[0] - min_p[0]) * inverse_leaf_size[0]) + 1;
  const uint64_t dy = static_cast<uint64_t>((max_p[1] - min_p[1]) * inverse_leaf_size[1]) + 1;
  const uint64_t dz = static_cast<uint64_t>((max_p[2] - min_p[2]) * inverse_leaf_size[2]) + 1;

  if ((dx * dy * dz) > static_cast<uint64_t>(std::numeric_limits<int>::max())) {
    PCL_WARN("[FeatureExtractor] Leaf size is too small for the input dataset. Integer indices would overflow.");
    return;
  }

  // Compute the minimum and maximum bounding box values
  Eigen::Vector4i min_b, max_b;
  min_b[0] = static_cast<int>(floor(min_p[0] * inverse_leaf_size[0]));
  max_b[0] = static_cast<int>(floor(max_p[0] * inverse_leaf_size[0]));
  min_b[1] = static_cast<int>(floor(min_p[1] * inverse_leaf_size[1]));
  max_b[1] = static_cast<int>(floor(max_p[1] * inverse_leaf_size[1]));
  min_b[2] = static_cast<int>(floor(min_p[2] * inverse_leaf_size[2]));
  max_b[2] = static_cast<int>(floor(max_p[2] * inverse_leaf_size[2]));

  // Compute the number of divisions needed along all axis
  const Eigen::Vector4i div_b = max_b - min_b + Eigen::Vector4i::Ones();
  /* div_b[3]              = 0; */

  // Set up the division multiplier
  const Eigen::Vector4i divb_mul = Eigen::Vector4i(1, div_b[0], div_b[0] * div_b[1], 0);

  // Storage for mapping leaf
  std::unordered_map<int, feature_selection::IndexPair> indices_map;

  indices_map.reserve(indices_in_cloud.size());

  // First pass: go over all points and insert them into the index_vector vector
  // with calculated idx. Points with the same idx value will contribute to the
  // same point of resulting CloudPoint
  for (const auto &idx : indices_in_cloud) {

    const auto &point = cloud_in->at(idx.col, idx.row);

    const int ijk0 = static_cast<int>(floor(point.x * inverse_leaf_size[0]) - static_cast<float>(min_b[0]));
    const int ijk1 = static_cast<int>(floor(point.y * inverse_leaf_size[1]) - static_cast<float>(min_b[1]));
    const int ijk2 = static_cast<int>(floor(point.z * inverse_leaf_size[2]) - static_cast<float>(min_b[2]));

    // Compute the centroid leaf index
    const int voxel_idx = ijk0 * divb_mul[0] + ijk1 * divb_mul[1] + ijk2 * divb_mul[2];

    // Store last added point index for each voxel cell
    indices_map[voxel_idx] = idx;
  }

  indices_to_be_appended->reserve(indices_to_be_appended->size() + indices_map.size());
  for (auto iter = std::begin(indices_map); iter != std::end(indices_map); iter++) {
    indices_to_be_appended->push_back(iter->second);
  }
}
/*//}*/

/*//{ pseudoDownsampleIndices() */
void FeatureExtractor::pseudoDownsampleIndices(const IndicesPtr &indices_to_be_appended, const pcl::PointCloud<PointType>::Ptr &cloud,
                                               const feature_selection::Indices_t &indices_in_cloud, const double resolution_sq) {

  PointType &point_prev = cloud->at(indices_in_cloud.at(0).col, indices_in_cloud.at(0).row);

  for (int i = 1; i < indices_in_cloud.size(); i++) {

    const auto &idx   = indices_in_cloud.at(i);
    const auto &point = cloud->at(idx.col, idx.row);

    const double dist_sq = std::pow(point.x - point_prev.x, 2) + std::pow(point.y - point_prev.y, 2) + std::pow(point.z - point_prev.z, 2);

    if (dist_sq > resolution_sq) {
      point_prev = point;
      indices_to_be_appended->push_back(idx);
    }
  }
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

}  // namespace aloam_slam
