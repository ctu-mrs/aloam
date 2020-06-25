#include "aloam_slam/feature_extractor.h"
#include "aloam_slam/odometry.h"
#include "aloam_slam/mapping.h"

namespace aloam_slam
{

/*//{ FeatureExtractor() */
FeatureExtractor::FeatureExtractor(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::shared_ptr<mrs_lib::Profiler> profiler,
                                   std::shared_ptr<AloamOdometry> odometry, std::string map_frame, float scan_period_sec)
    : _profiler(profiler), _odometry(odometry), _frame_map(map_frame), _scan_period_sec(scan_period_sec) {

  ros::NodeHandle nh_(parent_nh);

  param_loader.loadParam("vertical_fov", _vertical_fov_half);
  param_loader.loadParam("scan_line", _number_of_rings);

  if (_number_of_rings != 16) {
    ROS_ERROR_STREAM(
        "[AloamFeatureExtractor] MRS version of Aloam curently supports only 3D lidar with 16 scan rings. To be implemented for 32, 64 and 128 rings.");
    ros::shutdown();
  }

  _ray_vert_delta = _vertical_fov_half / float(_number_of_rings - 1);  // vertical resolution
  _vertical_fov_half /= 2.0;                                           // get half fov

  _initialization_frames_delay = int(1.0 / scan_period_sec);

  _sub_laser_cloud = nh_.subscribe("laser_cloud_in", 1, &FeatureExtractor::callbackLaserCloud, this, ros::TransportHints().tcpNoDelay());

  ROS_INFO("[AloamFeatureExtractor]: Listening laser cloud at topic: %s", _sub_laser_cloud.getTopic().c_str());
}
/*//}*/

/*//{ laserCloudHandler */
void FeatureExtractor::callbackLaserCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
  if (!is_initialized) {
    return;
  }
  if (laserCloudMsg->data.size() == 0) {
    ROS_WARN("[AloamFeatureExtractor]: Received empty laser cloud msg. Skipping frame.");
    return;
  }

  mrs_lib::Routine profiler_routine = _profiler->createRoutine("callbackLaserCloud");

  if (_frame_count++ < _initialization_frames_delay) {
    if (_frame_count == 1) {
      ROS_INFO("[AloamFeatureExtractor]: Received first laser cloud msg.");
      for (auto field : laserCloudMsg->fields) {
        if (field.name == "ring") {
          _has_field_ring = true;
          ROS_INFO("[AloamFeatureExtractor]: Laser cloud msg contains field `ring`. Will use this information for data processing.");
          break;
        }
      }
    }
    return;
  }

  // Process input data per row
  std::vector<int>                rows_start_idxs(_number_of_rings, 0);
  std::vector<int>                rows_end_idxs(_number_of_rings, 0);
  float                           processing_time;
  pcl::PointCloud<PointType>::Ptr laser_cloud = boost::make_shared<pcl::PointCloud<PointType>>();
  parseRowsFromCloudMsg(laserCloudMsg, laser_cloud, rows_start_idxs, rows_end_idxs, processing_time);

  TicToc             t_whole;
  std::vector<float> cloudCurvature;
  std::vector<int>   cloudSortInd;
  std::vector<int>   cloudNeighborPicked;
  std::vector<int>   cloudLabel;

  int cloud_size = laser_cloud->points.size();
  cloudCurvature.resize(cloud_size);
  cloudSortInd.resize(cloud_size);
  cloudNeighborPicked.resize(cloud_size, 0);
  cloudLabel.resize(cloud_size, 0);

  for (int i = 5; i < cloud_size - 5; i++) {
    float diffX = laser_cloud->points.at(i - 5).x + laser_cloud->points.at(i - 4).x + laser_cloud->points.at(i - 3).x + laser_cloud->points.at(i - 2).x +
                  laser_cloud->points.at(i - 1).x - 10 * laser_cloud->points.at(i).x + laser_cloud->points.at(i + 1).x + laser_cloud->points.at(i + 2).x +
                  laser_cloud->points.at(i + 3).x + laser_cloud->points.at(i + 4).x + laser_cloud->points.at(i + 5).x;
    float diffY = laser_cloud->points.at(i - 5).y + laser_cloud->points.at(i - 4).y + laser_cloud->points.at(i - 3).y + laser_cloud->points.at(i - 2).y +
                  laser_cloud->points.at(i - 1).y - 10 * laser_cloud->points.at(i).y + laser_cloud->points.at(i + 1).y + laser_cloud->points.at(i + 2).y +
                  laser_cloud->points.at(i + 3).y + laser_cloud->points.at(i + 4).y + laser_cloud->points.at(i + 5).y;
    float diffZ = laser_cloud->points.at(i - 5).z + laser_cloud->points.at(i - 4).z + laser_cloud->points.at(i - 3).z + laser_cloud->points.at(i - 2).z +
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
      int sp = rows_start_idxs.at(i) + (rows_end_idxs.at(i) - rows_start_idxs.at(i)) * j / 6;
      int ep = rows_start_idxs.at(i) + (rows_end_idxs.at(i) - rows_start_idxs.at(i)) * (j + 1) / 6 - 1;

      TicToc t_tmp;
      std::sort(cloudSortInd.begin() + sp, cloudSortInd.begin() + ep + 1,
                [&cloudCurvature](int i, int j) { return cloudCurvature.at(i) < cloudCurvature.at(j); });
      time_q_sort += t_tmp.toc();

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSortInd.at(k);

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
            float diffX = laser_cloud->points.at(ind + l).x - laser_cloud->points.at(ind + l - 1).x;
            float diffY = laser_cloud->points.at(ind + l).y - laser_cloud->points.at(ind + l - 1).y;
            float diffZ = laser_cloud->points.at(ind + l).z - laser_cloud->points.at(ind + l - 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laser_cloud->points.at(ind + l).x - laser_cloud->points.at(ind + l + 1).x;
            float diffY = laser_cloud->points.at(ind + l).y - laser_cloud->points.at(ind + l + 1).y;
            float diffZ = laser_cloud->points.at(ind + l).z - laser_cloud->points.at(ind + l + 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSortInd.at(k);

        if (cloudNeighborPicked.at(ind) == 0 && cloudCurvature.at(ind) < 0.1) {

          cloudLabel.at(ind) = -1;
          surf_points_flat->push_back(laser_cloud->points.at(ind));

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked.at(ind) = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laser_cloud->points.at(ind + l).x - laser_cloud->points.at(ind + l - 1).x;
            float diffY = laser_cloud->points.at(ind + l).y - laser_cloud->points.at(ind + l - 1).y;
            float diffZ = laser_cloud->points.at(ind + l).z - laser_cloud->points.at(ind + l - 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laser_cloud->points.at(ind + l).x - laser_cloud->points.at(ind + l + 1).x;
            float diffY = laser_cloud->points.at(ind + l).y - laser_cloud->points.at(ind + l + 1).y;
            float diffZ = laser_cloud->points.at(ind + l).z - laser_cloud->points.at(ind + l + 1).z;
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

    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS = boost::make_shared<pcl::PointCloud<PointType>>();
    pcl::VoxelGrid<PointType>       downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(*surfPointsLessFlatScanDS);

    *surf_points_less_flat += *surfPointsLessFlatScanDS;
  }

  /*//}*/

  float time_pts = t_pts.toc();

  laser_cloud->header.frame_id              = _frame_map;
  corner_points_sharp->header.frame_id      = _frame_map;
  corner_points_less_sharp->header.frame_id = _frame_map;
  surf_points_flat->header.frame_id         = _frame_map;
  surf_points_less_flat->header.frame_id    = _frame_map;

  pcl::uint64_t stamp;
  pcl_conversions::toPCL(laserCloudMsg->header.stamp, stamp);
  laser_cloud->header.stamp              = stamp;
  corner_points_sharp->header.stamp      = stamp;
  corner_points_less_sharp->header.stamp = stamp;
  surf_points_flat->header.stamp         = stamp;
  surf_points_less_flat->header.stamp    = stamp;

  _odometry->setData(corner_points_sharp, corner_points_less_sharp, surf_points_flat, surf_points_less_flat, laser_cloud);

  float time_whole = t_whole.toc() + processing_time;

  ROS_INFO_THROTTLE(1.0, "[AloamFeatureExtractor] Run time: %0.1f ms (%0.1f Hz)", time_whole, std::min(1.0f / _scan_period_sec, 1000.0f / time_whole));
  ROS_DEBUG_THROTTLE(1.0, "[AloamFeatureExtractor] points: %d; preparing data: %0.1f ms; q-sorting data: %0.1f ms; computing features: %0.1f ms", cloud_size,
                     processing_time, time_q_sort, time_pts);
  ROS_WARN_COND(time_whole > _scan_period_sec * 1000.0f, "[AloamFeatureExtractor] Feature extraction took over %0.2f ms", _scan_period_sec * 1000.0f);
}
/*//}*/

/*//{ parseRowsFromCloudMsg() */
void FeatureExtractor::parseRowsFromCloudMsg(const sensor_msgs::PointCloud2ConstPtr &cloud, pcl::PointCloud<PointType>::Ptr cloud_processed,
                                    std::vector<int> &rows_start_indices, std::vector<int> &rows_end_indices, float &processing_time) {
  TicToc t_prepare;

  pcl::PointCloud<PointType> cloud_pcl;
  pcl::fromROSMsg(*cloud, cloud_pcl);
  std::vector<int> indices;

  /*//{ Precompute points indices */
  const int cloud_size = cloud_pcl.points.size();
  float     ori_start   = -std::atan2(cloud_pcl.points.at(0).y, cloud_pcl.points.at(0).x);
  float     ori_end     = -std::atan2(cloud_pcl.points.at(cloud_size - 1).y, cloud_pcl.points.at(cloud_size - 1).x) + 2 * M_PI;

  if (ori_end - ori_start > 3 * M_PI) {
    ori_end -= 2 * M_PI;
  } else if (ori_end - ori_start < M_PI) {
    ori_end += 2 * M_PI;
  }

  bool                                    halfPassed = false;
  int                                     count      = cloud_size;
  PointType                               point;
  std::vector<pcl::PointCloud<PointType>> laserCloudScans(_number_of_rings);
  for (int i = 0; i < cloud_size; i++) {
    point.x = cloud_pcl.points.at(i).x;
    point.y = cloud_pcl.points.at(i).y;
    point.z = cloud_pcl.points.at(i).z;

    int   scanID = 0;
    float angle  = (M_PI_2 - acos(point.z / sqrt(point.x * point.x + point.y * point.y + point.z * point.z))) * 180.0 / M_PI;

    if (_number_of_rings == 16) {
      scanID = std::round((angle + _vertical_fov_half) / _ray_vert_delta);
      /* ROS_WARN("point: (%0.2f, %0.2f, %0.2f), angle: %0.2f deg, scan_id: %d", point.x, point.y, point.z, angle, scanID); */
      /* scanID = int((angle + 15) / 2 + 0.5); */
      if (scanID > int(_number_of_rings - 1) || scanID < 0) {
        count--;
        continue;
      }
    } else if (_number_of_rings == 32) {
      // TODO: get correct scanID
      scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
      if (scanID > int(_number_of_rings - 1) || scanID < 0) {
        count--;
        continue;
      }
    } else if (_number_of_rings == 64) {
      // TODO: get correct scanID
      if (angle >= -8.83)
        scanID = int((2 - angle) * 3.0 + 0.5);
      else
        scanID = int(_number_of_rings / 2 + int((-8.83 - angle) * 2.0 + 0.5));

      // use [0 50]  > 50 remove outlies
      if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0) {
        count--;
        continue;
      }
    } else {
      /* printf("wrong scan number\n"); */
      ROS_ERROR_STREAM("[AloamFeatureExtractor] wrong scan number");
      ROS_BREAK();
    }
    // printf("angle %f scanID %d \n", angle, scanID);

    float ori = -std::atan2(point.y, point.x);
    if (!halfPassed) {
      if (ori < ori_start - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > ori_start + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - ori_start > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;
      if (ori < ori_end - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > ori_end + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    float rel_time   = (ori - ori_start) / (ori_end - ori_start);
    point.intensity = scanID + _scan_period_sec * rel_time;
    laserCloudScans.at(scanID).push_back(point);
  }
  /*//}*/

  for (int i = 0; i < _number_of_rings; i++) {
    rows_start_indices.at(i) = cloud_processed->size() + 5;
    *cloud_processed += laserCloudScans.at(i);
    rows_end_indices.at(i) = cloud_processed->size() - 6;
  }

  processing_time = t_prepare.toc();
}
/*//}*/

}  // namespace aloam_slam

