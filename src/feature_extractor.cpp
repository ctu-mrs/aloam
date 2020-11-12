#include "aloam_slam/feature_extractor.h"

namespace aloam_slam
{

/*//{ FeatureExtractor() */
FeatureExtractor::FeatureExtractor(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::shared_ptr<mrs_lib::Profiler> profiler,
                                   std::shared_ptr<AloamOdometry> odometry, std::string lidar_frame, float scan_period_sec)
    : _profiler(profiler), _odometry(odometry), _frame_lidar(lidar_frame), _scan_period_sec(scan_period_sec) {

  ros::NodeHandle nh_(parent_nh);

  ros::Time::waitForValid();

  param_loader.loadParam("vertical_fov", _vertical_fov_half);
  param_loader.loadParam("scan_line", _number_of_rings);

  if (_number_of_rings != 16) {
    ROS_ERROR_STREAM(
        "[AloamFeatureExtractor] MRS version of Aloam curently supports only 3D lidar with 16 scan rings. To be implemented for 32, 64 and 128 rings.");
    ros::shutdown();
  }

  // In OS1 rev. 1 sensor, all 64 rings are filled (those we did not pay for are filled with 0s)
  _os1_rings_diff = 64 / _number_of_rings;

  _ray_vert_delta = _vertical_fov_half / float(_number_of_rings - 1);  // vertical resolution
  _vertical_fov_half /= 2.0;                                           // get half fov

  _initialization_frames_delay = int(1.0 / scan_period_sec);

  _sub_laser_cloud = nh_.subscribe("laser_cloud_in", 1, &FeatureExtractor::callbackLaserCloud, this, ros::TransportHints().tcpNoDelay());

  ROS_INFO("[AloamFeatureExtractor]: Listening laser cloud at topic: %s", _sub_laser_cloud.getTopic().c_str());
}
/*//}*/

/*//{ laserCloudHandler */
void FeatureExtractor::callbackLaserCloud(const sensor_msgs::PointCloud2::ConstPtr &laserCloudMsg) {
  if (!is_initialized) {
    return;
  }
  if (laserCloudMsg->data.size() == 0) {
    ROS_WARN("[AloamFeatureExtractor]: Received empty laser cloud msg. Skipping frame.");
    return;
  }

  mrs_lib::Routine profiler_routine = _profiler->createRoutine("callbackLaserCloud");

  // Skip 1s of data
  ROS_INFO_ONCE("[AloamFeatureExtractor]: Received first laser cloud msg.");
  if (_frame_count++ < _initialization_frames_delay) {
    if (_frame_count == 1) {
      _data_have_ring_field = hasField("ring", laserCloudMsg);
      ROS_INFO_COND(_data_have_ring_field, "[AloamFeatureExtractor]: Laser cloud msg contains field `ring`. Will use this information for data processing.");
    }
    return;
  }

  // Process input data per row
  std::vector<int>                rows_start_idxs(_number_of_rings, 0);
  std::vector<int>                rows_end_idxs(_number_of_rings, 0);
  float                           processing_time;
  pcl::PointCloud<PointType>::Ptr laser_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
  if (_data_have_ring_field) {
    parseRowsFromOS1CloudMsg(laserCloudMsg, laser_cloud, rows_start_idxs, rows_end_idxs, processing_time);
  } else {
    parseRowsFromCloudMsg(laserCloudMsg, laser_cloud, rows_start_idxs, rows_end_idxs, processing_time);
  }

  TicToc             t_whole;
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

  TicToc t_pts;

  pcl::PointCloud<PointType>::Ptr corner_points_sharp      = boost::make_shared<pcl::PointCloud<PointType>>();
  pcl::PointCloud<PointType>::Ptr corner_points_less_sharp = boost::make_shared<pcl::PointCloud<PointType>>();
  pcl::PointCloud<PointType>::Ptr surf_points_flat         = boost::make_shared<pcl::PointCloud<PointType>>();
  pcl::PointCloud<PointType>::Ptr surf_points_less_flat    = boost::make_shared<pcl::PointCloud<PointType>>();

  /*//{ Compute features (planes and edges) in two resolutions */
  float time_q_sort = 0;
  for (int i = 0; i < _number_of_rings; i++) {
    if (rows_end_idxs.at(i) - rows_start_idxs.at(i) < 6) {
      continue;
    }
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan = boost::make_shared<pcl::PointCloud<PointType>>();
    for (int j = 0; j < 6; j++) {
      const int sp = rows_start_idxs.at(i) + (rows_end_idxs.at(i) - rows_start_idxs.at(i)) * j / 6;
      const int ep = rows_start_idxs.at(i) + (rows_end_idxs.at(i) - rows_start_idxs.at(i)) * (j + 1) / 6 - 1;

      TicToc t_tmp;
      std::sort(cloudSortInd.begin() + sp, cloudSortInd.begin() + ep + 1,
                [&cloudCurvature](int i, int j) { return cloudCurvature.at(i) < cloudCurvature.at(j); });
      time_q_sort += t_tmp.toc();

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

  const float time_pts = t_pts.toc();

  laser_cloud->header.frame_id              = _frame_lidar;
  corner_points_sharp->header.frame_id      = _frame_lidar;
  corner_points_less_sharp->header.frame_id = _frame_lidar;
  surf_points_flat->header.frame_id         = _frame_lidar;
  surf_points_less_flat->header.frame_id    = _frame_lidar;

  pcl::uint64_t stamp;
  pcl_conversions::toPCL(laserCloudMsg->header.stamp, stamp);
  laser_cloud->header.stamp              = stamp;
  corner_points_sharp->header.stamp      = stamp;
  corner_points_less_sharp->header.stamp = stamp;
  surf_points_flat->header.stamp         = stamp;
  surf_points_less_flat->header.stamp    = stamp;

  _odometry->setData(corner_points_sharp, corner_points_less_sharp, surf_points_flat, surf_points_less_flat, laser_cloud);

  const float time_whole = t_whole.toc() + processing_time;

  ROS_INFO_THROTTLE(1.0, "[AloamFeatureExtractor] Run time: %0.1f ms (%0.1f Hz)", time_whole, std::min(1.0f / _scan_period_sec, 1000.0f / time_whole));
  ROS_DEBUG_THROTTLE(1.0, "[AloamFeatureExtractor] points: %d; preparing data: %0.1f ms; q-sorting data: %0.1f ms; computing features: %0.1f ms", cloud_size,
                     processing_time, time_q_sort, time_pts);
  ROS_WARN_COND(time_whole > _scan_period_sec * 1000.0f, "[AloamFeatureExtractor] Feature extraction took over %0.2f ms", _scan_period_sec * 1000.0f);
}
/*//}*/

/*//{ parseRowsFromCloudMsg() */
void FeatureExtractor::parseRowsFromCloudMsg(const sensor_msgs::PointCloud2::ConstPtr &cloud, pcl::PointCloud<PointType>::Ptr &cloud_processed,
                                             std::vector<int> &rows_start_indices, std::vector<int> &rows_end_indices, float &processing_time) {
  TicToc t_prepare;

  pcl::PointCloud<PointType> cloud_pcl;
  pcl::fromROSMsg(*cloud, cloud_pcl);

  /*//{ Precompute points indices */
  const int   cloud_size    = cloud_pcl.points.size();
  const float azimuth_start = -std::atan2(cloud_pcl.points.at(0).y, cloud_pcl.points.at(0).x);
  float       azimuth_end   = -std::atan2(cloud_pcl.points.at(cloud_size - 1).y, cloud_pcl.points.at(cloud_size - 1).x) + 2 * M_PI;

  if (azimuth_end - azimuth_start > 3 * M_PI) {
    azimuth_end -= 2 * M_PI;
  } else if (azimuth_end - azimuth_start < M_PI) {
    azimuth_end += 2 * M_PI;
  }

  bool                                    halfPassed = false;
  int                                     count      = cloud_size;
  std::vector<pcl::PointCloud<PointType>> laser_cloud_rows(_number_of_rings);
  for (int i = 0; i < cloud_size; i++) {
    PointType point;
    point.x = cloud_pcl.points.at(i).x;
    point.y = cloud_pcl.points.at(i).y;
    point.z = cloud_pcl.points.at(i).z;

    int point_ring = 0;

    const float angle = (M_PI_2 - acos(point.z / sqrt(point.x * point.x + point.y * point.y + point.z * point.z))) * 180.0 / M_PI;

    if (_number_of_rings == 16) {
      point_ring = std::round((angle + _vertical_fov_half) / _ray_vert_delta);
      /* ROS_WARN("point: (%0.2f, %0.2f, %0.2f), angle: %0.2f deg, scan_id: %d", point.x, point.y, point.z, angle, point_ring); */
      /* point_ring = int((angle + 15) / 2 + 0.5); */
      if (point_ring > int(_number_of_rings - 1) || point_ring < 0) {
        count--;
        continue;
      }
    } else if (_number_of_rings == 32) {
      // TODO: get correct point_ring
      point_ring = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
      if (point_ring > int(_number_of_rings - 1) || point_ring < 0) {
        count--;
        continue;
      }
    } else if (_number_of_rings == 64) {
      // TODO: get correct point_ring
      if (angle >= -8.83)
        point_ring = int((2 - angle) * 3.0 + 0.5);
      else
        point_ring = int(_number_of_rings / 2 + int((-8.83 - angle) * 2.0 + 0.5));

      // use [0 50]  > 50 remove outlies
      if (angle > 2 || angle < -24.33 || point_ring > 50 || point_ring < 0) {
        count--;
        continue;
      }
    }

    float point_azimuth = -std::atan2(point.y, point.x);
    if (!halfPassed) {
      if (point_azimuth < azimuth_start - M_PI / 2) {
        point_azimuth += 2 * M_PI;
      } else if (point_azimuth > azimuth_start + M_PI * 3 / 2) {
        point_azimuth -= 2 * M_PI;
      }

      if (point_azimuth - azimuth_start > M_PI) {
        halfPassed = true;
      }
    } else {
      point_azimuth += 2 * M_PI;
      if (point_azimuth < azimuth_end - M_PI * 3 / 2) {
        point_azimuth += 2 * M_PI;
      } else if (point_azimuth > azimuth_end + M_PI / 2) {
        point_azimuth -= 2 * M_PI;
      }
    }

    const float rel_time = (point_azimuth - azimuth_start) / (azimuth_end - azimuth_start);
    point.intensity      = point_ring + _scan_period_sec * rel_time;
    laser_cloud_rows.at(point_ring).push_back(point);
  }
  /*//}*/

  for (int i = 0; i < _number_of_rings; i++) {
    rows_start_indices.at(i) = cloud_processed->size() + 5;
    *cloud_processed += laser_cloud_rows.at(i);
    rows_end_indices.at(i) = cloud_processed->size() - 6;
  }

  processing_time = t_prepare.toc();
}
/*//}*/

/*//{ parseRowsFromOS1CloudMsg() */
void FeatureExtractor::parseRowsFromOS1CloudMsg(const sensor_msgs::PointCloud2::ConstPtr &cloud, pcl::PointCloud<PointType>::Ptr &cloud_processed,
                                                std::vector<int> &rows_start_indices, std::vector<int> &rows_end_indices, float &processing_time) {
  TicToc t_prepare;

  // Create PointOS1 pointcloud {x, y, z, intensity, t, reflectivity, ring, noise, range}
  pcl::PointCloud<ouster_ros::OS1::PointOS1>::Ptr cloud_pcl_with_nans = boost::make_shared<pcl::PointCloud<ouster_ros::OS1::PointOS1>>();
  pcl::fromROSMsg(*cloud, *cloud_pcl_with_nans);

  // Remove NaNs
  std::vector<int>                                valid_indices;
  pcl::PointCloud<ouster_ros::OS1::PointOS1>::Ptr cloud_pcl = boost::make_shared<pcl::PointCloud<ouster_ros::OS1::PointOS1>>();
  removeNaNFromPointCloud(cloud_pcl_with_nans, cloud_pcl, valid_indices);

  /*//{ Precompute points indices */
  const int   cloud_size    = cloud_pcl->points.size();
  const float azimuth_start = -std::atan2(cloud_pcl->points.at(0).y, cloud_pcl->points.at(0).x);
  float       azimuth_end   = -std::atan2(cloud_pcl->points.at(cloud_size - 1).y, cloud_pcl->points.at(cloud_size - 1).x) + 2 * M_PI;

  if (azimuth_end - azimuth_start > 3 * M_PI) {
    azimuth_end -= 2 * M_PI;
  } else if (azimuth_end - azimuth_start < M_PI) {
    azimuth_end += 2 * M_PI;
  }

  bool                                    halfPassed = false;
  std::vector<pcl::PointCloud<PointType>> laser_cloud_rows(_number_of_rings);
  for (int i = 0; i < cloud_size; i++) {
    PointType point;
    point.x = cloud_pcl->points.at(i).x;
    point.y = cloud_pcl->points.at(i).y;
    point.z = cloud_pcl->points.at(i).z;

    // Read row (ring) directly from msg
    const int point_ring = cloud_pcl->points.at(i).ring / _os1_rings_diff;

    // Compute intensity TODO: can we polish this crazy ifs?
    float point_azimuth = -std::atan2(point.y, point.x);
    if (!halfPassed) {
      if (point_azimuth < azimuth_start - M_PI / 2) {
        point_azimuth += 2 * M_PI;
      } else if (point_azimuth > azimuth_start + M_PI * 3 / 2) {
        point_azimuth -= 2 * M_PI;
      }

      if (point_azimuth - azimuth_start > M_PI) {
        halfPassed = true;
      }
    } else {
      point_azimuth += 2 * M_PI;
      if (point_azimuth < azimuth_end - M_PI * 3 / 2) {
        point_azimuth += 2 * M_PI;
      } else if (point_azimuth > azimuth_end + M_PI / 2) {
        point_azimuth -= 2 * M_PI;
      }
    }

    // TODO: can we use `t` fiels from OS1 message?
    const float rel_time = (point_azimuth - azimuth_start) / (azimuth_end - azimuth_start);
    point.intensity      = point_ring + _scan_period_sec * rel_time;
    laser_cloud_rows.at(point_ring).push_back(point);
  }
  /*//}*/

  for (int i = 0; i < _number_of_rings; i++) {
    rows_start_indices.at(i) = cloud_processed->size() + 5;
    *cloud_processed += laser_cloud_rows.at(i);
    rows_end_indices.at(i) = cloud_processed->size() - 6;
  }

  processing_time = t_prepare.toc();
}
/*//}*/

/*//{ removeNaNFromPointCloud() */
void FeatureExtractor::removeNaNFromPointCloud(const pcl::PointCloud<ouster_ros::OS1::PointOS1>::Ptr cloud_in,
                                               pcl::PointCloud<ouster_ros::OS1::PointOS1>::Ptr &cloud_out, std::vector<int> &indices) {

  if (cloud_in->is_dense) {
    cloud_out = cloud_in;
    return;
  }

  unsigned int k = 0;
  cloud_out->resize(cloud_in->size());
  indices.resize(cloud_in->size());

  for (unsigned int i = 0; i < cloud_in->size(); i++) {
    if (std::isfinite(cloud_in->at(i).x) && std::isfinite(cloud_in->at(i).y) && std::isfinite(cloud_in->at(i).z)) {
      cloud_out->at(k) = cloud_in->at(i);
      indices.at(k++)  = i;
    }
  }

  cloud_out->header   = cloud_in->header;
  cloud_out->is_dense = true;

  if (cloud_in->size() != k) {
    cloud_out->resize(k);
    indices.resize(k);
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
