#include "aloam_slam/feature_extractor.h"
#include "aloam_slam/odometry.h"
#include "aloam_slam/mapping.h"

namespace aloam_slam
{

/* //{ class FeatureExtractor */

/*//{ FeatureExtractor() */
FeatureExtractor::FeatureExtractor(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::shared_ptr<AloamOdometry> odometry,
                                   std::string map_frame, float scan_period_sec)
    : _odometry(odometry), _frame_map(map_frame), _scan_period_sec(scan_period_sec) {

  ros::NodeHandle nh_(parent_nh);

  param_loader.loadParam("vertical_fov", _vertical_fov_half);
  param_loader.loadParam("scan_line", _number_of_rings);
  param_loader.loadParam("min_range", _min_range_sq);
  param_loader.loadParam("max_range", _max_range_sq);
  param_loader.loadParam("external_preprocessing", _external_preprocessing);

  if (_number_of_rings != 16) {
    ROS_ERROR_STREAM("[AloamFeatureExtractor] MRS version of Aloam curently supports only 3D lidar with 16 scan rings. To be implemented for 32, 64 and 128 rings.");
    ros::shutdown();
  }

  _min_range_sq *= _min_range_sq;
  _max_range_sq *= _max_range_sq;
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
  ROS_INFO_ONCE("[AloamFeatureExtractor]: Received first laser cloud msg.");
  if (_frame_count++ < _initialization_frames_delay) {
    return;
  }

  TicToc           t_whole;
  TicToc           t_prepare;
  std::vector<int> scanStartInd(_number_of_rings, 0);
  std::vector<int> scanEndInd(_number_of_rings, 0);

  pcl::PointCloud<PointType> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
  std::vector<int> indices;

  if (!_external_preprocessing) {
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeCloseAndFarPointCloud(laserCloudIn, laserCloudIn);
  }

  int   cloudSize = laserCloudIn.points.size();
  float startOri  = -std::atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
  float endOri    = -std::atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool                                    halfPassed = false;
  int                                     count      = cloudSize;
  PointType                               point;
  std::vector<pcl::PointCloud<PointType>> laserCloudScans(_number_of_rings);
  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn.points[i].x;
    point.y = laserCloudIn.points[i].y;
    point.z = laserCloudIn.points[i].z;

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
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;
      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    float relTime   = (ori - startOri) / (endOri - startOri);
    point.intensity = scanID + _scan_period_sec * relTime;
    laserCloudScans[scanID].push_back(point);
  }

  cloudSize = count;
  /* printf("points size %d \n", cloudSize); */
  ROS_INFO_STREAM_THROTTLE(1.0, "[AloamFeatureExtractor] points size " << cloudSize);

  pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
  /* ROS_ERROR("Cloud:"); */
  for (int i = 0; i < _number_of_rings; i++) {
    scanStartInd[i] = laserCloud->size() + 5;
    /* ROS_ERROR("Ring %d: %ld", i, laserCloudScans[i].points.size()); */
    *laserCloud += laserCloudScans[i];
    scanEndInd[i] = laserCloud->size() - 6;
  }

  /* printf("prepare time %f \n", t_prepare.toc()); */
  ROS_INFO_STREAM_THROTTLE(1.0, "[AloamFeatureExtractor] prepare time " << t_prepare.toc() << " ms");

  for (int i = 5; i < cloudSize - 5; i++) {
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x +
                  laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x +
                  laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y +
                  laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y +
                  laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z +
                  laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z +
                  laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

    cloudCurvature[i]      = diffX * diffX + diffY * diffY + diffZ * diffZ;
    cloudSortInd[i]        = i;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i]          = 0;
  }

  TicToc t_pts;

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>);

  float t_q_sort = 0;
  for (int i = 0; i < _number_of_rings; i++) {
    if (scanEndInd[i] - scanStartInd[i] < 6)
      continue;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
    for (int j = 0; j < 6; j++) {
      int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
      int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

      TicToc t_tmp;
      /* std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, comp); */
      std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, [this](int i, int j) { return cloudCurvature[i] < cloudCurvature[j]; });
      t_q_sort += t_tmp.toc();

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSortInd[k];

        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1) {

          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel[ind] = 2;
            cornerPointsSharp->push_back(laserCloud->points[ind]);
            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;

          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSortInd[k];

        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1) {

          cloudLabel[ind] = -1;
          surfPointsFlat->push_back(laserCloud->points[ind]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfPointsLessFlatScan->push_back(laserCloud->points[k]);
        }
      }
    }

    pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointType>  downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    *surfPointsLessFlat += surfPointsLessFlatScanDS;
  }
  /* printf("sort q time %f \n", t_q_sort); */
  /* printf("seperate points time %f \n", t_pts.toc()); */
  ROS_INFO_STREAM_THROTTLE(1.0, "[AloamFeatureExtractor] sort q time " << t_q_sort << " ms");
  ROS_INFO_STREAM_THROTTLE(1.0, "[AloamFeatureExtractor] separate points time " << t_pts.toc() << " ms");

  laserCloud->header.frame_id            = _frame_map;
  cornerPointsSharp->header.frame_id     = _frame_map;
  cornerPointsLessSharp->header.frame_id = _frame_map;
  surfPointsFlat->header.frame_id        = _frame_map;
  surfPointsLessFlat->header.frame_id    = _frame_map;

  pcl::uint64_t stamp;
  pcl_conversions::toPCL(laserCloudMsg->header.stamp, stamp);
  laserCloud->header.stamp            = stamp;
  cornerPointsSharp->header.stamp     = stamp;
  cornerPointsLessSharp->header.stamp = stamp;
  surfPointsFlat->header.stamp        = stamp;
  surfPointsLessFlat->header.stamp    = stamp;

  /* printf("scan registration time %f ms *************\n", t_whole.toc()); */
  ROS_INFO_STREAM_THROTTLE(1.0, "[AloamFeatureExtractor] scan registration time " << t_whole.toc() << " ms");
  if (t_whole.toc() > _scan_period_sec * 1000) {
    ROS_WARN("[AloamFeatureExtractor] scan registration process over %0.2f ms", _scan_period_sec * 1000);
  }

  _odometry->compute_local_odometry(cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat, laserCloud);
}
/*//}*/

/*//{ comp */
bool FeatureExtractor::comp(int i, int j) {
  return (cloudCurvature[i] < cloudCurvature[j]);
}
/*//}*/

/*//{ removeCloseAndFarPointCloud() */
void FeatureExtractor::removeCloseAndFarPointCloud(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out) {
  if (&cloud_in != &cloud_out) {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize(cloud_in.points.size());
  }

  size_t j = 0;

  for (size_t i = 0; i < cloud_in.points.size(); ++i) {
    double dist_sq = cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z;
    if (dist_sq < _min_range_sq || dist_sq > _max_range_sq) {
      continue;
    }
    cloud_out.points[j] = cloud_in.points[i];
    j++;
  }
  if (j != cloud_in.points.size()) {
    cloud_out.points.resize(j);
  }

  cloud_out.height   = 1;
  cloud_out.width    = static_cast<uint32_t>(j);
  cloud_out.is_dense = true;
}
/*//}*/

//}

}  // namespace aloam_slam

