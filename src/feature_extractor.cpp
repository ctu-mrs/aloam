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
  std::vector<int>                      rows_start_idxs(_number_of_rings, 0);
  std::vector<int>                      rows_end_idxs(_number_of_rings, 0);
  const pcl::PointCloud<PointType>::Ptr laser_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
  const bool                            parsed      = parseRowsFromOusterMsg(laserCloudMsg, laser_cloud, rows_start_idxs, rows_end_idxs);
  timer.checkpoint("parsing lidar data");

  if (!parsed) {
    return;
  }


  std::vector<float> cloudCurvature;
  std::vector<int>   cloudSortInd;
  std::vector<int>   cloudNeighborPicked;
  std::vector<int>   cloudLabel;

  const unsigned int cloud_size = laser_cloud->points.size();
  cloudCurvature.resize(cloud_size);
  cloudSortInd.resize(cloud_size);
  cloudNeighborPicked.resize(cloud_size, 0);
  cloudLabel.resize(cloud_size, 0);

  for (unsigned int i = 5; i < cloud_size - 5; i++) {
    const float diffX = laser_cloud->points.at(i - 5).x + laser_cloud->points.at(i - 4).x + laser_cloud->points.at(i - 3).x + laser_cloud->points.at(i - 2).x +
                        laser_cloud->points.at(i - 1).x - 10 * laser_cloud->points.at(i).x + laser_cloud->points.at(i + 1).x + laser_cloud->points.at(i + 2).x +
                        laser_cloud->points.at(i + 3).x + laser_cloud->points.at(i + 4).x + laser_cloud->points.at(i + 5).x;
    const float diffY = laser_cloud->points.at(i - 5).y + laser_cloud->points.at(i - 4).y + laser_cloud->points.at(i - 3).y + laser_cloud->points.at(i - 2).y +
                        laser_cloud->points.at(i - 1).y - 10 * laser_cloud->points.at(i).y + laser_cloud->points.at(i + 1).y + laser_cloud->points.at(i + 2).y +
                        laser_cloud->points.at(i + 3).y + laser_cloud->points.at(i + 4).y + laser_cloud->points.at(i + 5).y;
    const float diffZ = laser_cloud->points.at(i - 5).z + laser_cloud->points.at(i - 4).z + laser_cloud->points.at(i - 3).z + laser_cloud->points.at(i - 2).z +
                        laser_cloud->points.at(i - 1).z - 10 * laser_cloud->points.at(i).z + laser_cloud->points.at(i + 1).z + laser_cloud->points.at(i + 2).z +
                        laser_cloud->points.at(i + 3).z + laser_cloud->points.at(i + 4).z + laser_cloud->points.at(i + 5).z;

    cloudCurvature.at(i) = diffX * diffX + diffY * diffY + diffZ * diffZ;
    cloudSortInd.at(i)   = i;
  }

  const pcl::PointCloud<PointType>::Ptr corner_points_sharp      = boost::make_shared<pcl::PointCloud<PointType>>();
  const pcl::PointCloud<PointType>::Ptr corner_points_less_sharp = boost::make_shared<pcl::PointCloud<PointType>>();
  const pcl::PointCloud<PointType>::Ptr surf_points_flat         = boost::make_shared<pcl::PointCloud<PointType>>();
  const pcl::PointCloud<PointType>::Ptr surf_points_less_flat    = boost::make_shared<pcl::PointCloud<PointType>>();

  /*//{ Compute features (planes and edges) in two resolutions */
  for (int i = 0; i < _number_of_rings; i++) {
    if (rows_end_idxs.at(i) - rows_start_idxs.at(i) < 6) {
      continue;
    }
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan = boost::make_shared<pcl::PointCloud<PointType>>();
    for (int j = 0; j < 6; j++) {
      const int sp = rows_start_idxs.at(i) + (rows_end_idxs.at(i) - rows_start_idxs.at(i)) * j / 6;
      const int ep = rows_start_idxs.at(i) + (rows_end_idxs.at(i) - rows_start_idxs.at(i)) * (j + 1) / 6 - 1;

      std::sort(cloudSortInd.begin() + sp, cloudSortInd.begin() + ep + 1,
                [&cloudCurvature](int i, int j) { return cloudCurvature.at(i) < cloudCurvature.at(j); });

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        const int ind = cloudSortInd.at(k);

        if (cloudNeighborPicked.at(ind) == 0 && cloudCurvature.at(ind) > 0.1) {

          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel.at(ind) = 2;
            corner_points_sharp->push_back(laser_cloud->points.at(ind));
            corner_points_less_sharp->push_back(laser_cloud->points.at(ind));
          } else if (largestPickedNum <= 20) {
            cloudLabel.at(ind) = 1;
            corner_points_less_sharp->push_back(laser_cloud->points.at(ind));
          } else {
            break;
          }

          cloudNeighborPicked.at(ind) = 1;

          for (int l = 1; l <= 5; l++) {
            const float diffX = laser_cloud->points.at(ind + l).x - laser_cloud->points.at(ind + l - 1).x;
            const float diffY = laser_cloud->points.at(ind + l).y - laser_cloud->points.at(ind + l - 1).y;
            const float diffZ = laser_cloud->points.at(ind + l).z - laser_cloud->points.at(ind + l - 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }
          for (int l = -1; l >= -5; l--) {
            const float diffX = laser_cloud->points.at(ind + l).x - laser_cloud->points.at(ind + l + 1).x;
            const float diffY = laser_cloud->points.at(ind + l).y - laser_cloud->points.at(ind + l + 1).y;
            const float diffZ = laser_cloud->points.at(ind + l).z - laser_cloud->points.at(ind + l + 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        const int ind = cloudSortInd.at(k);

        if (cloudNeighborPicked.at(ind) == 0 && cloudCurvature.at(ind) < 0.1) {

          cloudLabel.at(ind) = -1;
          surf_points_flat->push_back(laser_cloud->points.at(ind));

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked.at(ind) = 1;
          for (int l = 1; l <= 5; l++) {
            const float diffX = laser_cloud->points.at(ind + l).x - laser_cloud->points.at(ind + l - 1).x;
            const float diffY = laser_cloud->points.at(ind + l).y - laser_cloud->points.at(ind + l - 1).y;
            const float diffZ = laser_cloud->points.at(ind + l).z - laser_cloud->points.at(ind + l - 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }
          for (int l = -1; l >= -5; l--) {
            const float diffX = laser_cloud->points.at(ind + l).x - laser_cloud->points.at(ind + l + 1).x;
            const float diffY = laser_cloud->points.at(ind + l).y - laser_cloud->points.at(ind + l + 1).y;
            const float diffZ = laser_cloud->points.at(ind + l).z - laser_cloud->points.at(ind + l + 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel.at(k) <= 0) {
          surfPointsLessFlatScan->push_back(laser_cloud->points.at(k));
        }
      }
    }

    pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointType>  downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    *surf_points_less_flat += surfPointsLessFlatScanDS;
  }
  /*//}*/

  timer.checkpoint("parsing features");

  laser_cloud->header.frame_id              = _frame_map;
  corner_points_sharp->header.frame_id      = _frame_map;
  corner_points_less_sharp->header.frame_id = _frame_map;
  surf_points_flat->header.frame_id         = _frame_map;
  surf_points_less_flat->header.frame_id    = _frame_map;

  std::uint64_t stamp;
  pcl_conversions::toPCL(laserCloudMsg->header.stamp, stamp);
  laser_cloud->header.stamp              = stamp;
  corner_points_sharp->header.stamp      = stamp;
  corner_points_less_sharp->header.stamp = stamp;
  surf_points_flat->header.stamp         = stamp;
  surf_points_less_flat->header.stamp    = stamp;

  _odometry->setData(corner_points_sharp, corner_points_less_sharp, surf_points_flat, surf_points_less_flat, laser_cloud);

  /* ROS_INFO_THROTTLE(1.0, "[AloamFeatureExtractor] Run time: %0.1f ms (%0.1f Hz)", time_whole, std::min(1.0f / _scan_period_sec, 1000.0f / time_whole)); */
  /* ROS_DEBUG_THROTTLE(1.0, "[AloamFeatureExtractor] points: %d; preparing data: %0.1f ms; q-sorting data: %0.1f ms; computing features: %0.1f ms", cloud_size,
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
bool FeatureExtractor::parseRowsFromOusterMsg(const sensor_msgs::PointCloud2::ConstPtr &cloud, const pcl::PointCloud<PointType>::Ptr &cloud_processed,
                                              std::vector<int> &rows_start_indices, std::vector<int> &rows_end_indices) {

  pcl::PointCloud<ouster_ros::Point>::Ptr cloud_raw = boost::make_shared<pcl::PointCloud<ouster_ros::Point>>();
  pcl::fromROSMsg(*cloud, *cloud_raw);

  if (!cloud_raw->isOrganized()) {
    ROS_ERROR_THROTTLE(1.0, "[AloamFeatureExtractor] Input data are not organized.");
    return false;
  } else if (_number_of_rings != cloud_raw->height) {
    ROS_ERROR_THROTTLE(1.0, "[AloamFeatureExtractor] Expected number of rings != input data height.");
    return false;
  }

  const uint32_t t_start    = cloud_raw->at(0, 0).t;
  const uint32_t t_end      = cloud_raw->at(static_cast<int>(cloud_raw->width) - 1, 0).t;
  const float    t_duration = static_cast<float>(t_end - t_start);

  for (int r = 0; r < cloud_raw->height; r++) {

    pcl::PointCloud<PointType> row;
    row.reserve(cloud->width);

    for (int c = 0; c < cloud_raw->width; c++) {

      const auto &point_os = cloud_raw->at(c, r);

      // TODO: remove
      /* if (point_os.range < 450) { */
      /*   continue; */
      /* } */

      if (std::isfinite(point_os.x) && std::isfinite(point_os.y) && std::isfinite(point_os.z)) {

        PointType point;
        point.x = point_os.x;
        point.y = point_os.y;
        point.z = point_os.z;

        // Read row (ring) directly from msg
        const int point_ring = point_os.ring;

        const float rel_time = static_cast<float>(point_os.t - t_start) / t_duration;
        point.intensity      = static_cast<float>(point_ring) + _scan_period_sec * rel_time;

        row.push_back(point);
      }
    }

    rows_start_indices.at(r) = cloud_processed->size() + 5;
    *cloud_processed += row;
    rows_end_indices.at(r) = cloud_processed->size() - 6;
  }

  return true;
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
