// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*//{ Includes */
#include <cmath>
#include <vector>
#include <string>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>

#include <opencv/cv.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"

#include <mrs_lib/transformer.h>
/*//}*/

#define DISTORTION 0

/*//{ Variables scanRegistration */
using std::atan2;
using std::cos;
using std::sin;

bool   systemRegistrationInited = false;
double scanPeriod;

const int systemDelay     = 0;
int       systemInitCount = 0;
int       N_SCANS         = 0;
float     vertical_fov_half;
float     ray_vert_delta;
float     cloudCurvature[400000];
int       cloudSortInd[400000];
int       cloudNeighborPicked[400000];
int       cloudLabel[400000];

double min_range_sq;
double max_range_sq;

/*//}*/

/*//{ Variables laserOdometry */
std::string           _frame_lidar;
std::string           _frame_fcu;
std::string           _frame_map;
mrs_lib::Transformer *transformer_;
tf::Transform         transform_lidar_to_fcu_;

int corner_correspondence = 0, plane_correspondence = 0;

/* constexpr double SCAN_PERIOD           = 0.1; */
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN           = 2.5;

int  skipFrameNum         = 5;
bool systemOdometryInited = false;

double timeCornerPointsSharp     = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat        = 0;
double timeSurfPointsLessFlat    = 0;
double timeLaserCloudFullRes     = 0;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum   = 0;

// Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d    t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};

Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
Eigen::Map<Eigen::Vector3d>    t_last_curr(para_t);

std::queue<pcl::PointCloud<PointType>::Ptr> cornerSharpBuf;
std::queue<pcl::PointCloud<PointType>::Ptr> cornerLessSharpBuf;
std::queue<pcl::PointCloud<PointType>::Ptr> surfFlatBuf;
std::queue<pcl::PointCloud<PointType>::Ptr> surfLessFlatBuf;
std::queue<pcl::PointCloud<PointType>::Ptr> fullPointsBuf;
std::mutex                                  mBuf;
/*//}*/

/*//{ Functions scanRegistration */
bool comp(int i, int j) {
  return (cloudCurvature[i] < cloudCurvature[j]);
}

void removeCloseAndFarPointCloud(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out) {
  if (&cloud_in != &cloud_out) {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize(cloud_in.points.size());
  }

  size_t j = 0;

  for (size_t i = 0; i < cloud_in.points.size(); ++i) {
    double dist_sq = cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z;
    if (dist_sq < min_range_sq || dist_sq > max_range_sq) {
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

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
  if (!systemRegistrationInited) {
    systemInitCount++;
    if (systemInitCount >= systemDelay) {
      systemRegistrationInited = true;
    } else
      return;
  }

  TicToc           t_whole;
  TicToc           t_prepare;
  std::vector<int> scanStartInd(N_SCANS, 0);
  std::vector<int> scanEndInd(N_SCANS, 0);

  pcl::PointCloud<PointType> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
  std::vector<int> indices;

  pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
  removeCloseAndFarPointCloud(laserCloudIn, laserCloudIn);

  int   cloudSize = laserCloudIn.points.size();
  float startOri  = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
  float endOri    = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }
  // printf("end Ori %f\n", endOri);

  bool                                    halfPassed = false;
  int                                     count      = cloudSize;
  PointType                               point;
  std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn.points[i].x;
    point.y = laserCloudIn.points[i].y;
    point.z = laserCloudIn.points[i].z;

    int   scanID = 0;
    float angle  = (M_PI_2 - acos(point.z / sqrt(point.x * point.x + point.y * point.y + point.z * point.z))) * 180.0 / M_PI;

    if (N_SCANS == 16) {
      scanID = std::round((angle + vertical_fov_half) / ray_vert_delta);
      /* ROS_WARN("point: (%0.2f, %0.2f, %0.2f), angle: %0.2f deg, scan_id: %d", point.x, point.y, point.z, angle, scanID); */
      /* scanID = int((angle + 15) / 2 + 0.5); */
      if (scanID > (N_SCANS - 1) || scanID < 0) {
        count--;
        continue;
      }
    } else if (N_SCANS == 32) {
      scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
      if (scanID > (N_SCANS - 1) || scanID < 0) {
        count--;
        continue;
      }
    } else if (N_SCANS == 64) {
      if (angle >= -8.83)
        scanID = int((2 - angle) * 3.0 + 0.5);
      else
        scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

      // use [0 50]  > 50 remove outlies
      if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0) {
        count--;
        continue;
      }
    } else {
      /* printf("wrong scan number\n"); */
      ROS_ERROR_STREAM("wrong scan number");
      ROS_BREAK();
    }
    // printf("angle %f scanID %d \n", angle, scanID);

    float ori = -atan2(point.y, point.x);
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
    point.intensity = scanID + scanPeriod * relTime;
    laserCloudScans[scanID].push_back(point);
  }

  cloudSize = count;
  /* printf("points size %d \n", cloudSize); */
  ROS_INFO_STREAM_THROTTLE(1, "points size " << cloudSize);

  pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
  /* ROS_ERROR("Cloud:"); */
  for (int i = 0; i < N_SCANS; i++) {
    scanStartInd[i] = laserCloud->size() + 5;
    /* ROS_ERROR("Ring %d: %ld", i, laserCloudScans[i].points.size()); */
    *laserCloud += laserCloudScans[i];
    scanEndInd[i] = laserCloud->size() - 6;
  }

  /* printf("prepare time %f \n", t_prepare.toc()); */
  ROS_INFO_STREAM_THROTTLE(1, "prepare time " << t_prepare.toc() << " ms");

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

  pcl::PointCloud<PointType> cornerPointsSharp;
  pcl::PointCloud<PointType> cornerPointsLessSharp;
  pcl::PointCloud<PointType> surfPointsFlat;
  pcl::PointCloud<PointType> surfPointsLessFlat;

  float t_q_sort = 0;
  for (int i = 0; i < N_SCANS; i++) {
    if (scanEndInd[i] - scanStartInd[i] < 6)
      continue;
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
    for (int j = 0; j < 6; j++) {
      int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
      int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

      TicToc t_tmp;
      std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, comp);
      t_q_sort += t_tmp.toc();

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSortInd[k];

        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1) {

          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel[ind] = 2;
            cornerPointsSharp.push_back(laserCloud->points[ind]);
            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
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
          surfPointsFlat.push_back(laserCloud->points[ind]);

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

    surfPointsLessFlat += surfPointsLessFlatScanDS;
  }
  /* printf("sort q time %f \n", t_q_sort); */
  /* printf("seperate points time %f \n", t_pts.toc()); */
  ROS_INFO_STREAM_THROTTLE(1, "sort q time " << t_q_sort << " ms");
  ROS_INFO_STREAM_THROTTLE(1, "separate points time " << t_pts.toc() << " ms");

  laserCloud->header.frame_id           = _frame_map;
  cornerPointsSharp.header.frame_id     = _frame_map;
  cornerPointsLessSharp.header.frame_id = _frame_map;
  surfPointsFlat.header.frame_id        = _frame_map;
  surfPointsLessFlat.header.frame_id    = _frame_map;

  pcl::uint64_t stamp;
  pcl_conversions::toPCL(laserCloudMsg->header.stamp, stamp);
  laserCloud->header.stamp           = stamp;
  cornerPointsSharp.header.stamp     = stamp;
  cornerPointsLessSharp.header.stamp = stamp;
  surfPointsFlat.header.stamp        = stamp;
  surfPointsLessFlat.header.stamp    = stamp;

  mBuf.lock();
  fullPointsBuf.push(laserCloud);
  cornerSharpBuf.push(cornerPointsSharp.makeShared());
  cornerLessSharpBuf.push(cornerPointsLessSharp.makeShared());
  surfFlatBuf.push(surfPointsFlat.makeShared());
  surfLessFlatBuf.push(surfPointsLessFlat.makeShared());
  mBuf.unlock();

  /* printf("scan registration time %f ms *************\n", t_whole.toc()); */
  ROS_INFO_STREAM_THROTTLE(1, "scan registration time " << t_whole.toc() << " ms");
  if (t_whole.toc() > 100)
    ROS_WARN("scan registration process over 100ms");
}
/*//}*/

/*//{ Functions laserOdometry */
// undistort lidar point
void TransformToStart(PointType const *const pi, PointType *const po) {
  // interpolation ratio
  double s;
  if (DISTORTION)
    s = (pi->intensity - int(pi->intensity)) / scanPeriod;
  else
    s = 1.0;
  // s = 1;
  Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
  Eigen::Vector3d    t_point_last = s * t_last_curr;
  Eigen::Vector3d    point(pi->x, pi->y, pi->z);
  Eigen::Vector3d    un_point = q_point_last * point + t_point_last;

  po->x         = un_point.x();
  po->y         = un_point.y();
  po->z         = un_point.z();
  po->intensity = pi->intensity;
}

// transform all lidar points to the start of the next frame

void TransformToEnd(PointType const *const pi, PointType *const po) {
  // undistort point first
  pcl::PointXYZI un_point_tmp;
  TransformToStart(pi, &un_point_tmp);

  Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
  Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

  po->x = point_end.x();
  po->y = point_end.y();
  po->z = point_end.z();

  // Remove distortion time info
  po->intensity = int(pi->intensity);
}
/*//}*/

/*//{ main */
int main(int argc, char **argv) {
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle nh;

  nh.param<int>("scan_line", N_SCANS, 16);
  if (N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64) {
    /* printf("only support velodyne with 16, 32 or 64 scan line!"); */
    ROS_ERROR_STREAM("only support velodyne with 16, 32 or 64 scan line!");
    return 0;
  }
  ROS_INFO_STREAM("scan line number " << N_SCANS);

  nh.param<double>("min_range", min_range_sq, 0.0);
  nh.param<double>("max_range", max_range_sq, 1000.0);
  ROS_INFO_STREAM("min range" << min_range_sq);
  ROS_INFO_STREAM("max range" << max_range_sq);
  min_range_sq *= min_range_sq;
  max_range_sq *= max_range_sq;

  nh.param<float>("vertical_fov", vertical_fov_half, 30.0);
  ROS_INFO_STREAM("vertical fov" << vertical_fov_half);
  ray_vert_delta = vertical_fov_half / float(N_SCANS - 1);
  vertical_fov_half /= 2.0;
  ROS_INFO_STREAM("vertical fov delta step" << ray_vert_delta);
  
  nh.param<double>("frequency", scanPeriod, 10.0);
  ROS_INFO_STREAM("frequency" << scanPeriod);
  scanPeriod = 1.0 / scanPeriod;

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/sensor_points", 100, laserCloudHandler);

  nh.param<int>("mapping_skip_frame", skipFrameNum, 2);

  std::string uav_name;
  nh.getParam("uav_name", uav_name);
  transformer_ = new mrs_lib::Transformer("aloamLaserOdometry", uav_name);
  nh.getParam("lidar_frame", _frame_lidar);
  nh.getParam("fcu_frame", _frame_fcu);
  nh.getParam("map_frame", _frame_map);

  // Get static transform lidar->fcu
  mrs_lib::TransformStamped tf_mrs;
  ROS_INFO("Waiting 0.5 second to fill transform buffer.");
  ros::Duration(0.5).sleep();
  // TODO: rewrite to timer
  while (!transformer_->getTransform(_frame_lidar, _frame_fcu, ros::Time(0), tf_mrs)) {
    ROS_INFO_THROTTLE(0.5, "Looking for transform from %s to %s", _frame_lidar.c_str(), _frame_fcu.c_str());
  }
  tf::transformMsgToTF(tf_mrs.getTransform().transform, transform_lidar_to_fcu_);
  delete transformer_;

  ROS_INFO_STREAM_THROTTLE(1, "Mapping at %d " << 10 / skipFrameNum << " Hz");

  ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100);
  ros::Publisher pubLaserCloudSurfLast   = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100);
  ros::Publisher pubLaserCloudFullRes    = nh.advertise<sensor_msgs::PointCloud2>("/sensor_cloud_3", 100);
  ros::Publisher pubLaserOdometry        = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
  ros::Publisher pubLaserPath            = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

  nav_msgs::Path laserPath;

  int       frameCount = 0;
  ros::Rate rate(100);

  while (ros::ok()) {
    ros::spinOnce();

    ros::Time stamp;
    if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() && !surfFlatBuf.empty() && !surfLessFlatBuf.empty() && !fullPointsBuf.empty()) {
      pcl_conversions::fromPCL(cornerSharpBuf.front()->header.stamp, stamp);
      timeCornerPointsSharp = stamp.toSec();
      pcl_conversions::fromPCL(cornerLessSharpBuf.front()->header.stamp, stamp);
      timeCornerPointsLessSharp = stamp.toSec();
      pcl_conversions::fromPCL(surfFlatBuf.front()->header.stamp, stamp);
      timeSurfPointsFlat = stamp.toSec();
      pcl_conversions::fromPCL(fullPointsBuf.front()->header.stamp, stamp);
      timeLaserCloudFullRes = stamp.toSec();
      pcl_conversions::fromPCL(surfLessFlatBuf.front()->header.stamp, stamp);
      timeSurfPointsLessFlat = stamp.toSec();

      if (timeCornerPointsSharp != timeLaserCloudFullRes || timeCornerPointsLessSharp != timeLaserCloudFullRes || timeSurfPointsFlat != timeLaserCloudFullRes ||
          timeSurfPointsLessFlat != timeLaserCloudFullRes) {
        ROS_ERROR_STREAM("unsync message!");
        ROS_BREAK();
      }

      mBuf.lock();
      cornerPointsSharp = cornerSharpBuf.front();
      cornerSharpBuf.pop();

      cornerPointsLessSharp = cornerLessSharpBuf.front();
      cornerLessSharpBuf.pop();

      surfPointsFlat = surfFlatBuf.front();
      surfFlatBuf.pop();

      surfPointsLessFlat = surfLessFlatBuf.front();
      surfLessFlatBuf.pop();

      laserCloudFullRes = fullPointsBuf.front();
      fullPointsBuf.pop();
      mBuf.unlock();

      TicToc t_whole;
      // initializing
      if (!systemOdometryInited) {
        systemOdometryInited = true;
        std::cout << "Initialization finished \n";
      } else {
        int cornerPointsSharpNum = cornerPointsSharp->points.size();
        int surfPointsFlatNum    = surfPointsFlat->points.size();

        TicToc t_opt;
        for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {
          corner_correspondence = 0;
          plane_correspondence  = 0;

          // ceres::LossFunction *loss_function = NULL;
          ceres::LossFunction *         loss_function      = new ceres::HuberLoss(0.1);
          ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
          ceres::Problem::Options       problem_options;

          ceres::Problem problem(problem_options);
          problem.AddParameterBlock(para_q, 4, q_parameterization);
          problem.AddParameterBlock(para_t, 3);

          pcl::PointXYZI     pointSel;
          std::vector<int>   pointSearchInd;
          std::vector<float> pointSearchSqDis;

          TicToc t_data;
          // find correspondence for corner features
          for (int i = 0; i < cornerPointsSharpNum; ++i) {
            TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
            kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

            int closestPointInd = -1, minPointInd2 = -1;
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) {
              closestPointInd        = pointSearchInd[0];
              int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

              double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
              // search in the direction of increasing scan line
              for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j) {
                // if in the same scan line, continue
                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                  continue;

                // if not in nearby scans, end the loop
                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                  break;

                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * (laserCloudCornerLast->points[j].x - pointSel.x) +
                                    (laserCloudCornerLast->points[j].y - pointSel.y) * (laserCloudCornerLast->points[j].y - pointSel.y) +
                                    (laserCloudCornerLast->points[j].z - pointSel.z) * (laserCloudCornerLast->points[j].z - pointSel.z);

                if (pointSqDis < minPointSqDis2) {
                  // find nearer point
                  minPointSqDis2 = pointSqDis;
                  minPointInd2   = j;
                }
              }

              // search in the direction of decreasing scan line
              for (int j = closestPointInd - 1; j >= 0; --j) {
                // if in the same scan line, continue
                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                  continue;

                // if not in nearby scans, end the loop
                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                  break;

                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * (laserCloudCornerLast->points[j].x - pointSel.x) +
                                    (laserCloudCornerLast->points[j].y - pointSel.y) * (laserCloudCornerLast->points[j].y - pointSel.y) +
                                    (laserCloudCornerLast->points[j].z - pointSel.z) * (laserCloudCornerLast->points[j].z - pointSel.z);

                if (pointSqDis < minPointSqDis2) {
                  // find nearer point
                  minPointSqDis2 = pointSqDis;
                  minPointInd2   = j;
                }
              }
            }
            if (minPointInd2 >= 0)  // both closestPointInd and minPointInd2 is valid
            {
              Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x, cornerPointsSharp->points[i].y, cornerPointsSharp->points[i].z);
              Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x, laserCloudCornerLast->points[closestPointInd].y,
                                           laserCloudCornerLast->points[closestPointInd].z);
              Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x, laserCloudCornerLast->points[minPointInd2].y,
                                           laserCloudCornerLast->points[minPointInd2].z);

              double s;
              if (DISTORTION)
                s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / scanPeriod;
              else
                s = 1.0;
              ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
              problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
              corner_correspondence++;
            }
          }

          // find correspondence for plane features
          for (int i = 0; i < surfPointsFlatNum; ++i) {
            TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
            kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

            int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
            if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD) {
              closestPointInd = pointSearchInd[0];

              // get closest point's scan ID
              int    closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
              double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

              // search in the direction of increasing scan line
              for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j) {
                // if not in nearby scans, end the loop
                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                  break;

                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * (laserCloudSurfLast->points[j].x - pointSel.x) +
                                    (laserCloudSurfLast->points[j].y - pointSel.y) * (laserCloudSurfLast->points[j].y - pointSel.y) +
                                    (laserCloudSurfLast->points[j].z - pointSel.z) * (laserCloudSurfLast->points[j].z - pointSel.z);

                // if in the same or lower scan line
                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2   = j;
                }
                // if in the higher scan line
                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3) {
                  minPointSqDis3 = pointSqDis;
                  minPointInd3   = j;
                }
              }

              // search in the direction of decreasing scan line
              for (int j = closestPointInd - 1; j >= 0; --j) {
                // if not in nearby scans, end the loop
                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                  break;

                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * (laserCloudSurfLast->points[j].x - pointSel.x) +
                                    (laserCloudSurfLast->points[j].y - pointSel.y) * (laserCloudSurfLast->points[j].y - pointSel.y) +
                                    (laserCloudSurfLast->points[j].z - pointSel.z) * (laserCloudSurfLast->points[j].z - pointSel.z);

                // if in the same or higher scan line
                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2) {
                  minPointSqDis2 = pointSqDis;
                  minPointInd2   = j;
                } else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3) {
                  // find nearer point
                  minPointSqDis3 = pointSqDis;
                  minPointInd3   = j;
                }
              }

              if (minPointInd2 >= 0 && minPointInd3 >= 0) {

                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x, surfPointsFlat->points[i].y, surfPointsFlat->points[i].z);
                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x, laserCloudSurfLast->points[closestPointInd].y,
                                             laserCloudSurfLast->points[closestPointInd].z);
                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x, laserCloudSurfLast->points[minPointInd2].y,
                                             laserCloudSurfLast->points[minPointInd2].z);
                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x, laserCloudSurfLast->points[minPointInd3].y,
                                             laserCloudSurfLast->points[minPointInd3].z);

                double s;
                if (DISTORTION)
                  s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / scanPeriod;
                else
                  s = 1.0;
                ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                plane_correspondence++;
              }
            }
          }

          // printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
          ROS_INFO_STREAM_THROTTLE(1, "data association time " << t_data.toc() << " ms");

          if ((corner_correspondence + plane_correspondence) < 10) {
            ROS_INFO_STREAM("less correspondence! *************************************************");
          }

          TicToc                 t_solver;
          ceres::Solver::Options options;
          options.linear_solver_type           = ceres::DENSE_QR;
          options.max_num_iterations           = 4;
          options.minimizer_progress_to_stdout = false;
          ceres::Solver::Summary summary;
          ceres::Solve(options, &problem, &summary);
          /* printf("solver time %f ms \n", t_solver.toc()); */
          ROS_INFO_STREAM_THROTTLE(1, "solver time " << t_solver.toc() << " ms");
        }
        /* printf("optimization twice time %f \n", t_opt.toc()); */
        ROS_INFO_STREAM_THROTTLE(1, "optimization twice time " << t_opt.toc() << " ms");

        t_w_curr = t_w_curr + q_w_curr * t_last_curr;
        q_w_curr = q_w_curr * q_last_curr;
      }

      TicToc t_pub;

      // publish odometry
      tf::Transform  transform;
      tf::Quaternion q;
      transform.setOrigin(tf::Vector3(t_w_curr(0), t_w_curr(1), t_w_curr(2)));
      q.setW(q_w_curr.w());
      q.setX(q_w_curr.x());
      q.setY(q_w_curr.y());
      q.setZ(q_w_curr.z());
      transform.setRotation(q);
      tf::Transform tf_odom = transform * transform_lidar_to_fcu_.inverse();

      // nav_msgs::Odometry msg
      nav_msgs::Odometry laserOdometry;
      laserOdometry.header.frame_id      = _frame_map;
      laserOdometry.child_frame_id       = _frame_fcu;
      laserOdometry.header.stamp         = stamp;
      tf::Vector3 origin                 = tf_odom.getOrigin();
      laserOdometry.pose.pose.position.x = origin.x();
      laserOdometry.pose.pose.position.y = origin.y();
      laserOdometry.pose.pose.position.z = origin.z();
      tf::quaternionTFToMsg(tf_odom.getRotation(), laserOdometry.pose.pose.orientation);
      pubLaserOdometry.publish(laserOdometry);

      // geometry_msgs::PoseStamped and Path
      geometry_msgs::PoseStamped laserPose;
      laserPose.header       = laserOdometry.header;
      laserPose.pose         = laserOdometry.pose.pose;
      laserPath.header.stamp = laserOdometry.header.stamp;
      laserPath.poses.push_back(laserPose);
      laserPath.header.frame_id = _frame_map;
      pubLaserPath.publish(laserPath);

      // transform corner features and plane features to the scan end point
      if (0) {
        int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
        for (int i = 0; i < cornerPointsLessSharpNum; i++) {
          TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
        }

        int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
        for (int i = 0; i < surfPointsLessFlatNum; i++) {
          TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
        }

        int laserCloudFullResNum = laserCloudFullRes->points.size();
        for (int i = 0; i < laserCloudFullResNum; i++) {
          TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
        }
      }

      pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
      cornerPointsLessSharp                          = laserCloudCornerLast;
      laserCloudCornerLast                           = laserCloudTemp;

      laserCloudTemp     = surfPointsLessFlat;
      surfPointsLessFlat = laserCloudSurfLast;
      laserCloudSurfLast = laserCloudTemp;

      laserCloudCornerLastNum = laserCloudCornerLast->points.size();
      laserCloudSurfLastNum   = laserCloudSurfLast->points.size();

      // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';

      kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
      kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

      if (frameCount % skipFrameNum == 0) {
        frameCount = 0;

        sensor_msgs::PointCloud2 laserCloudCornerLast2;
        pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
        laserCloudCornerLast2.header.stamp    = stamp;
        laserCloudCornerLast2.header.frame_id = _frame_lidar;

        sensor_msgs::PointCloud2 laserCloudSurfLast2;
        pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
        laserCloudSurfLast2.header.stamp    = stamp;
        laserCloudSurfLast2.header.frame_id = _frame_lidar;

        sensor_msgs::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp    = stamp;
        laserCloudFullRes3.header.frame_id = _frame_lidar;

        pubLaserCloudCornerLast.publish(laserCloudCornerLast2);
        pubLaserCloudSurfLast.publish(laserCloudSurfLast2);
        pubLaserCloudFullRes.publish(laserCloudFullRes3);
      }
      /* printf("publication time %f ms \n", t_pub.toc()); */
      /* printf("whole laserOdometry time %f ms \n \n", t_whole.toc()); */
      ROS_INFO_STREAM_THROTTLE(1, "publication time " << t_pub.toc() << " ms");
      ROS_INFO_STREAM_THROTTLE(1, "whole laserOdometry time " << t_whole.toc() << " ms");
      if (t_whole.toc() > 100)
        ROS_WARN("odometry process over 100ms");

      frameCount++;
    }
    rate.sleep();
  }
  return 0;
}
/*//}*/
