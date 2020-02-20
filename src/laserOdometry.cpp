/*//{*/
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
/*//}*/

/*//{ Includes */
#include <math.h>
#include <cmath>
#include <vector>
#include <string>
#include <thread>
#include <iostream>

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
#include <tf_conversions/tf_eigen.h>
#include <eigen3/Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <mutex>
#include <queue>

#include <ceres/ceres.h>
#include <opencv/cv.h>

#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "lidarFactor.hpp"

#include <mrs_lib/transformer.h>
/*//}*/

#define DISTORTION 0
#define ROS_THROTTLE_PERIOD 2.0

/*//{ Variables scanRegistration */
using std::atan2;
using std::cos;
using std::sin;

bool is_initialized_ = false;
bool   systemRegistrationInited = false;
bool   debug                    = false;
bool   perform_scan_preprocessing;
double scanPeriod;

const int systemDelay     = 10;
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

std::mutex         mutex_odom_mavros;
bool               got_mavros_odom;
nav_msgs::Odometry mavros_odom;

/*//}*/

/*//{ Variables laserOdometry */
std::string           _frame_lidar;
std::string           _frame_fcu;
std::string           _frame_map;
mrs_lib::Transformer *transformer_;
tf::Transform         tf_lidar_in_fcu_frame_;
tf::Transform         tf_fcu_in_lidar_frame_;

ros::Publisher o_pubLaserOdometry;
ros::Publisher o_pubLaserPath;
nav_msgs::Path laserPath;

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
pcl::PointCloud<PointType>::Ptr       laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr       laserCloudSurfLast(new pcl::PointCloud<PointType>());

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

/* std::queue<pcl::PointCloud<PointType>::Ptr> cornerSharpBuf; */
/* std::queue<pcl::PointCloud<PointType>::Ptr> cornerLessSharpBuf; */
/* std::queue<pcl::PointCloud<PointType>::Ptr> surfFlatBuf; */
/* std::queue<pcl::PointCloud<PointType>::Ptr> surfLessFlatBuf; */
/* std::queue<pcl::PointCloud<PointType>::Ptr> fullPointsBuf; */
/*//}*/

/* Variables laserMapping //{*/
int m_frameCount = 0;

double m_timeLaserCloudCornerLast = 0;
double m_timeLaserCloudSurfLast   = 0;
double m_timeLaserCloudFullRes    = 0;
double m_timeLaserOdometry        = 0;

int       m_laserCloudCenWidth  = 10;
int       m_laserCloudCenHeight = 10;
int       m_laserCloudCenDepth  = 5;
const int m_laserCloudWidth     = 21;
const int m_laserCloudHeight    = 21;
const int m_laserCloudDepth     = 11;
const int m_laserCloudNum       = m_laserCloudWidth * m_laserCloudHeight * m_laserCloudDepth;  // 4851

int m_laserCloudValidInd[125];

// input: from odom
/* pcl::PointCloud<PointType>::Ptr m_laserCloudCornerLast(new pcl::PointCloud<PointType>()); */
/* pcl::PointCloud<PointType>::Ptr m_laserCloudSurfLast(new pcl::PointCloud<PointType>()); */

/* // ouput: all visualble cube points */


/* // input & output: points in one frame. local --> global */
/* pcl::PointCloud<PointType>::Ptr m_laserCloudFullRes(new pcl::PointCloud<PointType>()); */

/* // points in every cube */
pcl::PointCloud<PointType>::Ptr m_laserCloudCornerArray[m_laserCloudNum];
pcl::PointCloud<PointType>::Ptr m_laserCloudSurfArray[m_laserCloudNum];

double                         m_parameters[7] = {0, 0, 0, 1, 0, 0, 0};
Eigen::Map<Eigen::Quaterniond> m_q_w_curr(m_parameters);
Eigen::Map<Eigen::Vector3d>    m_t_w_curr(m_parameters + 4);

// wmap_T_odom * odom_T_curr = wmap_T_curr;
// transformation between odom's world and map's world frame
Eigen::Quaterniond m_q_wmap_wodom(1, 0, 0, 0);
Eigen::Vector3d    m_t_wmap_wodom(0, 0, 0);

Eigen::Quaterniond m_q_wodom_curr(1, 0, 0, 0);
Eigen::Vector3d    m_t_wodom_curr(0, 0, 0);

/* std::queue<pcl::PointCloud<PointType>> m_cornerLastBuf; */
/* std::queue<pcl::PointCloud<PointType>> m_surfLastBuf; */
/* std::queue<pcl::PointCloud<PointType>> m_fullResBuf; */
/* std::queue<nav_msgs::Odometry>         m_odometryBuf; */

pcl::VoxelGrid<PointType> m_downSizeFilterCorner;
pcl::VoxelGrid<PointType> m_downSizeFilterSurf;

std::vector<int>   m_pointSearchInd;
std::vector<float> m_pointSearchSqDis;

PointType m_pointOri, m_pointSel;

ros::Publisher m_pubLaserCloudMap;
ros::Publisher m_pubLaserCloudFullRes;
ros::Publisher m_pubOdomAftMapped;
ros::Publisher m_pubLaserAfterMappedPath;

nav_msgs::Path m_laserAfterMappedPath;

/*//}*/

/* Functions laserMapping //{*/

// set initial guess
void m_transformAssociateToMap() {
  m_q_w_curr = m_q_wmap_wodom * m_q_wodom_curr;
  m_t_w_curr = m_q_wmap_wodom * m_t_wodom_curr + m_t_wmap_wodom;
}

void m_transformUpdate() {
  m_q_wmap_wodom = m_q_w_curr * m_q_wodom_curr.inverse();
  m_t_wmap_wodom = m_t_w_curr - m_q_wmap_wodom * m_t_wodom_curr;
}

void m_pointAssociateToMap(PointType const *const pi, PointType *const po) {
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = m_q_w_curr * point_curr + m_t_w_curr;
  po->x                   = point_w.x();
  po->y                   = point_w.y();
  po->z                   = point_w.z();
  po->intensity           = pi->intensity;
  // po->intensity = 1.0;
}

void m_pointAssociateTobeMapped(PointType const *const pi, PointType *const po) {
  Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_curr = m_q_w_curr.inverse() * (point_w - m_t_w_curr);
  po->x                      = point_curr.x();
  po->y                      = point_curr.y();
  po->z                      = point_curr.z();
  po->intensity              = pi->intensity;
}

void m_process(nav_msgs::Odometry odometry, pcl::PointCloud<PointType>::Ptr m_laserCloudCornerLast, pcl::PointCloud<PointType>::Ptr m_laserCloudSurfLast,
               pcl::PointCloud<PointType>::Ptr m_laserCloudFullRes) {

  ros::Time stamp_cornerLastBuf;
  ros::Time stamp_surfLastBuf;
  ros::Time stamp_fullResBuf;
  pcl_conversions::fromPCL(m_laserCloudCornerLast->header.stamp, stamp_cornerLastBuf);
  pcl_conversions::fromPCL(m_laserCloudSurfLast->header.stamp, stamp_surfLastBuf);
  pcl_conversions::fromPCL(m_laserCloudFullRes->header.stamp, stamp_fullResBuf);
  m_timeLaserCloudCornerLast = stamp_cornerLastBuf.toSec();
  m_timeLaserCloudSurfLast   = stamp_surfLastBuf.toSec();
  m_timeLaserCloudFullRes    = stamp_fullResBuf.toSec();
  m_timeLaserOdometry        = odometry.header.stamp.toSec();

  if (m_timeLaserCloudCornerLast != m_timeLaserOdometry || m_timeLaserCloudSurfLast != m_timeLaserOdometry || m_timeLaserCloudFullRes != m_timeLaserOdometry) {
    ROS_WARN_STREAM("[aloamMapping] Unsync message (should happen just once): time corner "
                    << m_timeLaserCloudCornerLast << " surf " << m_timeLaserCloudSurfLast << " full " << m_timeLaserCloudFullRes << " odom "
                    << m_timeLaserOdometry);
    return;
  }

  m_q_wodom_curr.x() = odometry.pose.pose.orientation.x;
  m_q_wodom_curr.y() = odometry.pose.pose.orientation.y;
  m_q_wodom_curr.z() = odometry.pose.pose.orientation.z;
  m_q_wodom_curr.w() = odometry.pose.pose.orientation.w;
  m_t_wodom_curr.x() = odometry.pose.pose.position.x;
  m_t_wodom_curr.y() = odometry.pose.pose.position.y;
  m_t_wodom_curr.z() = odometry.pose.pose.position.z;

  /*//{*/
  TicToc t_whole;

  m_transformAssociateToMap();

  TicToc t_shift;
  int    centerCubeI = int((m_t_w_curr.x() + 25.0) / 50.0) + m_laserCloudCenWidth;
  int    centerCubeJ = int((m_t_w_curr.y() + 25.0) / 50.0) + m_laserCloudCenHeight;
  int    centerCubeK = int((m_t_w_curr.z() + 25.0) / 50.0) + m_laserCloudCenDepth;

  if (m_t_w_curr.x() + 25.0 < 0)
    centerCubeI--;
  if (m_t_w_curr.y() + 25.0 < 0)
    centerCubeJ--;
  if (m_t_w_curr.z() + 25.0 < 0)
    centerCubeK--;

  while (centerCubeI < 3) {
    for (int j = 0; j < m_laserCloudHeight; j++) {
      for (int k = 0; k < m_laserCloudDepth; k++) {
        int                             i = m_laserCloudWidth - 1;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
        for (; i >= 1; i--) {
          m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] =
              m_laserCloudCornerArray[i - 1 + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
          m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] =
              m_laserCloudSurfArray[i - 1 + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
        }
        m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] = laserCloudCubeCornerPointer;
        m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k]   = laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeI++;
    m_laserCloudCenWidth++;
  }

  while (centerCubeI >= m_laserCloudWidth - 3) {
    for (int j = 0; j < m_laserCloudHeight; j++) {
      for (int k = 0; k < m_laserCloudDepth; k++) {
        int                             i = 0;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
        for (; i < m_laserCloudWidth - 1; i++) {
          m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] =
              m_laserCloudCornerArray[i + 1 + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
          m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] =
              m_laserCloudSurfArray[i + 1 + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
        }
        m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] = laserCloudCubeCornerPointer;
        m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k]   = laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeI--;
    m_laserCloudCenWidth--;
  }

  while (centerCubeJ < 3) {
    for (int i = 0; i < m_laserCloudWidth; i++) {
      for (int k = 0; k < m_laserCloudDepth; k++) {
        int                             j = m_laserCloudHeight - 1;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
        for (; j >= 1; j--) {
          m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] =
              m_laserCloudCornerArray[i + m_laserCloudWidth * (j - 1) + m_laserCloudWidth * m_laserCloudHeight * k];
          m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] =
              m_laserCloudSurfArray[i + m_laserCloudWidth * (j - 1) + m_laserCloudWidth * m_laserCloudHeight * k];
        }
        m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] = laserCloudCubeCornerPointer;
        m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k]   = laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeJ++;
    m_laserCloudCenHeight++;
  }

  while (centerCubeJ >= m_laserCloudHeight - 3) {
    for (int i = 0; i < m_laserCloudWidth; i++) {
      for (int k = 0; k < m_laserCloudDepth; k++) {
        int                             j = 0;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
        for (; j < m_laserCloudHeight - 1; j++) {
          m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] =
              m_laserCloudCornerArray[i + m_laserCloudWidth * (j + 1) + m_laserCloudWidth * m_laserCloudHeight * k];
          m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] =
              m_laserCloudSurfArray[i + m_laserCloudWidth * (j + 1) + m_laserCloudWidth * m_laserCloudHeight * k];
        }
        m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] = laserCloudCubeCornerPointer;
        m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k]   = laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeJ--;
    m_laserCloudCenHeight--;
  }

  while (centerCubeK < 3) {
    for (int i = 0; i < m_laserCloudWidth; i++) {
      for (int j = 0; j < m_laserCloudHeight; j++) {
        int                             k = m_laserCloudDepth - 1;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
        for (; k >= 1; k--) {
          m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] =
              m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * (k - 1)];
          m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] =
              m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * (k - 1)];
        }
        m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] = laserCloudCubeCornerPointer;
        m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k]   = laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeK++;
    m_laserCloudCenDepth++;
  }

  while (centerCubeK >= m_laserCloudDepth - 3) {
    for (int i = 0; i < m_laserCloudWidth; i++) {
      for (int j = 0; j < m_laserCloudHeight; j++) {
        int                             k = 0;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k];
        for (; k < m_laserCloudDepth - 1; k++) {
          m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] =
              m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * (k + 1)];
          m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] =
              m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * (k + 1)];
        }
        m_laserCloudCornerArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k] = laserCloudCubeCornerPointer;
        m_laserCloudSurfArray[i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k]   = laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeK--;
    m_laserCloudCenDepth--;
  }

  int laserCloudValidNum = 0;

  for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
    for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
      for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++) {
        if (i >= 0 && i < m_laserCloudWidth && j >= 0 && j < m_laserCloudHeight && k >= 0 && k < m_laserCloudDepth) {
          m_laserCloudValidInd[laserCloudValidNum] = i + m_laserCloudWidth * j + m_laserCloudWidth * m_laserCloudHeight * k;
          laserCloudValidNum++;
        }
      }
    }
  }
  /*//}*/

  pcl::PointCloud<PointType>::Ptr m_laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr m_laserCloudSurfFromMap(new pcl::PointCloud<PointType>());
  for (int i = 0; i < laserCloudValidNum; i++) {
    *m_laserCloudCornerFromMap += *m_laserCloudCornerArray[m_laserCloudValidInd[i]];
    *m_laserCloudSurfFromMap += *m_laserCloudSurfArray[m_laserCloudValidInd[i]];
  }
  int laserCloudCornerFromMapNum = m_laserCloudCornerFromMap->points.size();
  int laserCloudSurfFromMapNum   = m_laserCloudSurfFromMap->points.size();

  /*//{*/
  pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
  m_downSizeFilterCorner.setInputCloud(m_laserCloudCornerLast);
  m_downSizeFilterCorner.filter(*laserCloudCornerStack);
  int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

  pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
  m_downSizeFilterSurf.setInputCloud(m_laserCloudSurfLast);
  m_downSizeFilterSurf.filter(*laserCloudSurfStack);
  int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

  /* printf("map prepare time %f ms\n", t_shift.toc()); */
  /* printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum); */
  ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamMapping] map prepare time " << t_shift.toc() << " ms");
  ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamMapping] map corner num " << laserCloudCornerFromMapNum << " surf num " << laserCloudSurfFromMapNum);
  if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50) {
    TicToc                           t_opt;
    TicToc                           t_tree;
    pcl::KdTreeFLANN<PointType>::Ptr m_kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
    pcl::KdTreeFLANN<PointType>::Ptr m_kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());
    m_kdtreeCornerFromMap->setInputCloud(m_laserCloudCornerFromMap);
    m_kdtreeSurfFromMap->setInputCloud(m_laserCloudSurfFromMap);
    /* printf("build tree time %f ms \n", t_tree.toc()); */
    ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamMapping] build tree time " << t_tree.toc() << " ms");

    for (int iterCount = 0; iterCount < 2; iterCount++) {
      // ceres::LossFunction *loss_function = NULL;
      ceres::LossFunction *         loss_function      = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
      ceres::Problem::Options       problem_options;

      ceres::Problem problem(problem_options);
      problem.AddParameterBlock(m_parameters, 4, q_parameterization);
      problem.AddParameterBlock(m_parameters + 4, 3);

      TicToc t_data;
      int    corner_num = 0;

      for (int i = 0; i < laserCloudCornerStackNum; i++) {
        m_pointOri = laserCloudCornerStack->points[i];
        // double sqrtDis = m_pointOri.x * m_pointOri.x + m_pointOri.y * m_pointOri.y + m_pointOri.z * m_pointOri.z;
        m_pointAssociateToMap(&m_pointOri, &m_pointSel);
        m_kdtreeCornerFromMap->nearestKSearch(m_pointSel, 5, m_pointSearchInd, m_pointSearchSqDis);

        if (m_pointSearchSqDis[4] < 1.0) {
          std::vector<Eigen::Vector3d> nearCorners;
          Eigen::Vector3d              center(0, 0, 0);
          for (int j = 0; j < 5; j++) {
            Eigen::Vector3d tmp(m_laserCloudCornerFromMap->points[m_pointSearchInd[j]].x, m_laserCloudCornerFromMap->points[m_pointSearchInd[j]].y,
                                m_laserCloudCornerFromMap->points[m_pointSearchInd[j]].z);
            center = center + tmp;
            nearCorners.push_back(tmp);
          }
          center = center / 5.0;

          Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
          for (int j = 0; j < 5; j++) {
            Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
            covMat                                  = covMat + tmpZeroMean * tmpZeroMean.transpose();
          }

          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

          // if is indeed line feature
          // note Eigen library sort eigenvalues in increasing order
          Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
          Eigen::Vector3d curr_point(m_pointOri.x, m_pointOri.y, m_pointOri.z);
          if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
            Eigen::Vector3d point_on_line = center;
            Eigen::Vector3d point_a, point_b;
            point_a = 0.1 * unit_direction + point_on_line;
            point_b = -0.1 * unit_direction + point_on_line;

            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
            problem.AddResidualBlock(cost_function, loss_function, m_parameters, m_parameters + 4);
            corner_num++;
          }
        }
        /*
        else if(m_pointSearchSqDis[4] < 0.01 * sqrtDis)
        {
                Eigen::Vector3d center(0, 0, 0);
                for (int j = 0; j < 5; j++)
                {
                        Eigen::Vector3d tmp(m_laserCloudCornerFromMap->points[m_pointSearchInd[j]].x,
                                                                m_laserCloudCornerFromMap->points[m_pointSearchInd[j]].y,
                                                                m_laserCloudCornerFromMap->points[m_pointSearchInd[j]].z);
                        center = center + tmp;
                }
                center = center / 5.0;
                Eigen::Vector3d curr_point(m_pointOri.x, m_pointOri.y, m_pointOri.z);
                ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
                problem.AddResidualBlock(cost_function, loss_function, m_parameters, m_parameters + 4);
        }
        */
      }

      int surf_num = 0;
      for (int i = 0; i < laserCloudSurfStackNum; i++) {
        m_pointOri = laserCloudSurfStack->points[i];
        // double sqrtDis = m_pointOri.x * m_pointOri.x + m_pointOri.y * m_pointOri.y + m_pointOri.z * m_pointOri.z;
        m_pointAssociateToMap(&m_pointOri, &m_pointSel);
        m_kdtreeSurfFromMap->nearestKSearch(m_pointSel, 5, m_pointSearchInd, m_pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (m_pointSearchSqDis[4] < 1.0) {

          for (int j = 0; j < 5; j++) {
            matA0(j, 0) = m_laserCloudSurfFromMap->points[m_pointSearchInd[j]].x;
            matA0(j, 1) = m_laserCloudSurfFromMap->points[m_pointSearchInd[j]].y;
            matA0(j, 2) = m_laserCloudSurfFromMap->points[m_pointSearchInd[j]].z;
            // printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
          }
          // find the norm of plane
          Eigen::Vector3d norm                 = matA0.colPivHouseholderQr().solve(matB0);
          double          negative_OA_dot_norm = 1 / norm.norm();
          norm.normalize();

          // Here n(pa, pb, pc) is unit norm of plane
          bool planeValid = true;
          for (int j = 0; j < 5; j++) {
            // if OX * n > 0.2, then plane is not fit well
            if (fabs(norm(0) * m_laserCloudSurfFromMap->points[m_pointSearchInd[j]].x + norm(1) * m_laserCloudSurfFromMap->points[m_pointSearchInd[j]].y +
                     norm(2) * m_laserCloudSurfFromMap->points[m_pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2) {
              planeValid = false;
              break;
            }
          }
          Eigen::Vector3d curr_point(m_pointOri.x, m_pointOri.y, m_pointOri.z);
          if (planeValid) {
            ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
            problem.AddResidualBlock(cost_function, loss_function, m_parameters, m_parameters + 4);
            surf_num++;
          }
        }
        /*
        else if(m_pointSearchSqDis[4] < 0.01 * sqrtDis)
        {
                Eigen::Vector3d center(0, 0, 0);
                for (int j = 0; j < 5; j++)
                {
                        Eigen::Vector3d tmp(m_laserCloudSurfFromMap->points[m_pointSearchInd[j]].x,
                                                                m_laserCloudSurfFromMap->points[m_pointSearchInd[j]].y,
                                                                m_laserCloudSurfFromMap->points[m_pointSearchInd[j]].z);
                        center = center + tmp;
                }
                center = center / 5.0;
                Eigen::Vector3d curr_point(m_pointOri.x, m_pointOri.y, m_pointOri.z);
                ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
                problem.AddResidualBlock(cost_function, loss_function, m_parameters, m_parameters + 4);
        }
        */
      }

      // printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
      // printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);

      /* printf("mapping data assosiation time %f ms \n", t_data.toc()); */
      ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamMapping] mapping data association time " << t_data.toc() << " ms");

      TicToc                 t_solver;
      ceres::Solver::Options options;
      options.linear_solver_type                = ceres::DENSE_QR;
      options.max_num_iterations                = 4;
      options.minimizer_progress_to_stdout      = false;
      options.check_gradients                   = false;
      options.gradient_check_relative_precision = 1e-4;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      /* printf("mapping solver time %f ms \n", t_solver.toc()); */
      ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamMapping] mapping solver time " << t_solver.toc() << " ms");

      // printf("time %f \n", m_timeLaserOdometry);
      // printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
      // printf("result q %f %f %f %f result t %f %f %f\n", m_parameters[3], m_parameters[0], m_parameters[1], m_parameters[2],
      //	   m_parameters[4], m_parameters[5], m_parameters[6]);
    }
    /* printf("mapping optimization time %f \n", t_opt.toc()); */
    ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamMapping] mapping optimization time " << t_opt.toc() << " ms");
  } else {
    ROS_WARN("[aloamMapping] time Map corner and surf num are not enough");
  }
  m_transformUpdate();
  /*//}*/

  TicToc t_add;
  for (int i = 0; i < laserCloudCornerStackNum; i++) {
    m_pointAssociateToMap(&laserCloudCornerStack->points[i], &m_pointSel);

    int cubeI = int((m_pointSel.x + 25.0) / 50.0) + m_laserCloudCenWidth;
    int cubeJ = int((m_pointSel.y + 25.0) / 50.0) + m_laserCloudCenHeight;
    int cubeK = int((m_pointSel.z + 25.0) / 50.0) + m_laserCloudCenDepth;

    if (m_pointSel.x + 25.0 < 0)
      cubeI--;
    if (m_pointSel.y + 25.0 < 0)
      cubeJ--;
    if (m_pointSel.z + 25.0 < 0)
      cubeK--;

    if (cubeI >= 0 && cubeI < m_laserCloudWidth && cubeJ >= 0 && cubeJ < m_laserCloudHeight && cubeK >= 0 && cubeK < m_laserCloudDepth) {
      int cubeInd = cubeI + m_laserCloudWidth * cubeJ + m_laserCloudWidth * m_laserCloudHeight * cubeK;
      m_laserCloudCornerArray[cubeInd]->push_back(m_pointSel);
    }
  }

  for (int i = 0; i < laserCloudSurfStackNum; i++) {
    m_pointAssociateToMap(&laserCloudSurfStack->points[i], &m_pointSel);

    int cubeI = int((m_pointSel.x + 25.0) / 50.0) + m_laserCloudCenWidth;
    int cubeJ = int((m_pointSel.y + 25.0) / 50.0) + m_laserCloudCenHeight;
    int cubeK = int((m_pointSel.z + 25.0) / 50.0) + m_laserCloudCenDepth;

    if (m_pointSel.x + 25.0 < 0)
      cubeI--;
    if (m_pointSel.y + 25.0 < 0)
      cubeJ--;
    if (m_pointSel.z + 25.0 < 0)
      cubeK--;

    if (cubeI >= 0 && cubeI < m_laserCloudWidth && cubeJ >= 0 && cubeJ < m_laserCloudHeight && cubeK >= 0 && cubeK < m_laserCloudDepth) {
      int cubeInd = cubeI + m_laserCloudWidth * cubeJ + m_laserCloudWidth * m_laserCloudHeight * cubeK;
      m_laserCloudSurfArray[cubeInd]->push_back(m_pointSel);
    }
  }
  /* printf("add points time %f ms\n", t_add.toc()); */
  ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamMapping] add points time " << t_add.toc() << " ms");


  TicToc t_filter;
  for (int i = 0; i < laserCloudValidNum; i++) {
    int ind = m_laserCloudValidInd[i];

    pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
    m_downSizeFilterCorner.setInputCloud(m_laserCloudCornerArray[ind]);
    m_downSizeFilterCorner.filter(*tmpCorner);
    m_laserCloudCornerArray[ind] = tmpCorner;

    pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
    m_downSizeFilterSurf.setInputCloud(m_laserCloudSurfArray[ind]);
    m_downSizeFilterSurf.filter(*tmpSurf);
    m_laserCloudSurfArray[ind] = tmpSurf;
  }
  /* printf("filter time %f ms \n", t_filter.toc()); */
  ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamMapping] filter time " << t_filter.toc() << " ms");

  TicToc    t_pub;
  ros::Time stamp = ros::Time().fromSec(m_timeLaserOdometry);

  if (m_frameCount % 20 == 0) {
    pcl::PointCloud<PointType> laserCloudMap;
    for (int i = 0; i < 4851; i++) {
      laserCloudMap += *m_laserCloudCornerArray[i];
      laserCloudMap += *m_laserCloudSurfArray[i];
    }
    sensor_msgs::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(laserCloudMap, laserCloudMsg);
    laserCloudMsg.header.stamp    = stamp;
    laserCloudMsg.header.frame_id = _frame_map;
    m_pubLaserCloudMap.publish(laserCloudMsg);
  }

  int laserCloudFullResNum = m_laserCloudFullRes->points.size();
  for (int i = 0; i < laserCloudFullResNum; i++) {
    m_pointAssociateToMap(&m_laserCloudFullRes->points[i], &m_laserCloudFullRes->points[i]);
  }

  sensor_msgs::PointCloud2::Ptr laserCloudFullRes3 = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*m_laserCloudFullRes, *laserCloudFullRes3);
  laserCloudFullRes3->header.stamp    = stamp;
  laserCloudFullRes3->header.frame_id = _frame_map;
  m_pubLaserCloudFullRes.publish(laserCloudFullRes3);

  /* printf("mapping pub time %f ms \n", t_pub.toc()); */
  /* printf("whole mapping time %f ms +++++\n", t_whole.toc()); */
  ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamMapping] mapping pub time " << t_pub.toc() << " ms");
  ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamMapping] whole mapping time " << t_whole.toc() << " ms");

  // TF fcu -> aloam origin (m_t_w_curr and m_q_w_curr is in origin -> fcu frame, hence inversion)
  static tf::TransformBroadcaster br;
  tf::Transform                   transform;
  tf::Quaternion                  q;
  transform.setOrigin(tf::Vector3(m_t_w_curr(0), m_t_w_curr(1), m_t_w_curr(2)));
  tf::quaternionEigenToTF(m_q_w_curr, q);
  transform.setRotation(q);

  tf::Transform tf_fcu = transform * tf_fcu_in_lidar_frame_;
  br.sendTransform(tf::StampedTransform(tf_fcu.inverse(), stamp, _frame_fcu, _frame_map));

  // nav_msgs::Odometry msg
  nav_msgs::Odometry odomAftMapped;
  odomAftMapped.header.frame_id      = _frame_map;
  odomAftMapped.header.stamp         = stamp;
  odomAftMapped.child_frame_id       = _frame_fcu;
  tf::Vector3 origin                 = tf_fcu.getOrigin();
  odomAftMapped.pose.pose.position.x = origin.x();
  odomAftMapped.pose.pose.position.y = origin.y();
  odomAftMapped.pose.pose.position.z = origin.z();
  tf::quaternionTFToMsg(tf_fcu.getRotation(), odomAftMapped.pose.pose.orientation);
  m_pubOdomAftMapped.publish(odomAftMapped);

  // Odometry path
  geometry_msgs::PoseStamped laserAfterMappedPose;
  laserAfterMappedPose.header   = odomAftMapped.header;
  laserAfterMappedPose.pose     = odomAftMapped.pose.pose;
  m_laserAfterMappedPath.header = odomAftMapped.header;
  m_laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
  m_pubLaserAfterMappedPath.publish(m_laserAfterMappedPath);

  m_frameCount++;
}
/*//}*/

/*//{ Functions aloamOdometry */
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

void o_process(pcl::PointCloud<PointType>::Ptr cornerPointsSharp, pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp,
               pcl::PointCloud<PointType>::Ptr surfPointsFlat, pcl::PointCloud<PointType>::Ptr surfPointsLessFlat, pcl::PointCloud<PointType>::Ptr laserCloudFullRes) {
  int frameCount = 0;

  ros::Time stamp;
  pcl_conversions::fromPCL(cornerPointsSharp->header.stamp, stamp);
  timeCornerPointsSharp = stamp.toSec();
  pcl_conversions::fromPCL(cornerPointsLessSharp->header.stamp, stamp);
  timeCornerPointsLessSharp = stamp.toSec();
  pcl_conversions::fromPCL(surfPointsFlat->header.stamp, stamp);
  timeSurfPointsFlat = stamp.toSec();
  pcl_conversions::fromPCL(laserCloudFullRes->header.stamp, stamp);
  timeLaserCloudFullRes = stamp.toSec();
  pcl_conversions::fromPCL(surfPointsLessFlat->header.stamp, stamp);
  timeSurfPointsLessFlat = stamp.toSec();

  if (timeCornerPointsSharp != timeLaserCloudFullRes || timeCornerPointsLessSharp != timeLaserCloudFullRes || timeSurfPointsFlat != timeLaserCloudFullRes ||
      timeSurfPointsLessFlat != timeLaserCloudFullRes) {
    ROS_ERROR_STREAM("[aloamOdometry] unsync message!");
    ROS_BREAK();
  }

  TicToc t_whole;
  // initializing
  if (!systemOdometryInited) {
    systemOdometryInited = true;
    ROS_INFO_STREAM("Initialization finished.");
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
      ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamOdometry] data association time " << t_data.toc() << " ms");

      if ((corner_correspondence + plane_correspondence) < 10) {
        ROS_INFO_STREAM("[aloamOdometry] less correspondence! *************************************************");
      }

      TicToc                 t_solver;
      ceres::Solver::Options options;
      options.linear_solver_type           = ceres::DENSE_QR;
      options.max_num_iterations           = 4;
      options.minimizer_progress_to_stdout = false;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      /* printf("solver time %f ms \n", t_solver.toc()); */
      ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamOdometry] solver time " << t_solver.toc() << " ms");
    }
    /* printf("optimization twice time %f \n", t_opt.toc()); */
    ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamOdometry] optimization twice time " << t_opt.toc() << " ms");

    t_w_curr = t_w_curr + q_w_curr * t_last_curr;
    q_w_curr = q_w_curr * q_last_curr;
  }

  TicToc t_pub;

  geometry_msgs::Quaternion ori;
  bool                      got_mavros = false;
  {
    std::scoped_lock lock(mutex_odom_mavros);
    if (got_mavros_odom && mavros_odom.header.stamp.toSec() - stamp.toSec() < 1.0) {
      got_mavros = true;
      ori        = mavros_odom.pose.pose.orientation;
    }
  }

  if (got_mavros) {
    tf::Quaternion q_mavros;
    tf::Quaternion q_odom;

    tf::quaternionMsgToTF(ori, q_mavros);
    tf::quaternionEigenToTF(q_w_curr, q_odom);

    // Parse roll and pitch from mavros and yaw from odometry
    double roll, pitch, yaw;
    double r, p, y;
    tf::Matrix3x3(q_mavros).getRPY(roll, pitch, y);
    tf::Matrix3x3(q_odom).getRPY(r, p, yaw);

    // Transform mavros fcu orientation to lidar orientation
    tf::Vector3 rpy = tf::quatRotate(tf_lidar_in_fcu_frame_.getRotation(), tf::Vector3(roll, pitch, 0));

    /* ROS_WARN("Lidar orientation: (%0.2f, %0.2f, %0.2f), odom orientation: (%0.2f, %0.2f, %0.2f)", rpy.x(), rpy.y(), rpy.z(), r, p, yaw); */
    ori = tf::createQuaternionMsgFromRollPitchYaw(rpy.x(), rpy.y(), yaw);
  } else {
    tf::quaternionEigenToMsg(q_w_curr, ori);
  }

  // Publish odometry as PoseStamped
  nav_msgs::Odometry laserOdometry;
  laserOdometry.header.frame_id = _frame_map;
  laserOdometry.header.stamp    = stamp;
  laserOdometry.child_frame_id  = _frame_lidar;

  tf::pointEigenToMsg(t_w_curr, laserOdometry.pose.pose.position);
  laserOdometry.pose.pose.orientation = ori;
  o_pubLaserOdometry.publish(laserOdometry);

  if (debug) {
    // geometry_msgs::PoseStamped and Path
    geometry_msgs::PoseStamped pose;
    pose.pose              = laserOdometry.pose.pose;
    pose.header.frame_id   = _frame_lidar;
    pose.header.stamp      = stamp;
    laserPath.header.stamp = laserOdometry.header.stamp;
    laserPath.poses.push_back(pose);

    o_pubLaserPath.publish(laserPath);
  }

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

    uint64_t pcl_stamp;
    pcl_conversions::toPCL(stamp, pcl_stamp);
    laserCloudCornerLast->header.stamp    = pcl_stamp;
    laserCloudCornerLast->header.frame_id = _frame_lidar;
    laserCloudSurfLast->header.stamp      = pcl_stamp;
    laserCloudSurfLast->header.frame_id   = _frame_lidar;
    laserCloudFullRes->header.stamp       = pcl_stamp;
    laserCloudFullRes->header.frame_id    = _frame_lidar;

    /* printf("publication time %f ms \n", t_pub.toc()); */
    /* printf("whole laserOdometry time %f ms \n \n", t_whole.toc()); */
    ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamOdometry] publication time " << t_pub.toc() << " ms");
    ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamOdometry] whole laserOdometry time " << t_whole.toc() << " ms");
    if (t_whole.toc() > 100) {
      ROS_WARN("[aloamOdometry] odometry process over 100ms");
    }

    m_process(laserOdometry, laserCloudCornerLast, laserCloudSurfLast, laserCloudFullRes);

    frameCount++;
  }
}

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
  if (!is_initialized_) {
    return;
  }
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

  if (perform_scan_preprocessing) {
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeCloseAndFarPointCloud(laserCloudIn, laserCloudIn);
  }

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
      ROS_ERROR_STREAM("[aloamOdometry] wrong scan number");
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
  ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamOdometry] points size " << cloudSize);

  pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
  /* ROS_ERROR("Cloud:"); */
  for (int i = 0; i < N_SCANS; i++) {
    scanStartInd[i] = laserCloud->size() + 5;
    /* ROS_ERROR("Ring %d: %ld", i, laserCloudScans[i].points.size()); */
    *laserCloud += laserCloudScans[i];
    scanEndInd[i] = laserCloud->size() - 6;
  }

  /* printf("prepare time %f \n", t_prepare.toc()); */
  ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamOdometry] prepare time " << t_prepare.toc() << " ms");

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
  ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamOdometry] sort q time " << t_q_sort << " ms");
  ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamOdometry] separate points time " << t_pts.toc() << " ms");

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
  ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamOdometry] scan registration time " << t_whole.toc() << " ms");
  if (t_whole.toc() > 100) {
    ROS_WARN("[aloamOdometry] scan registration process over 100ms");
  }
  o_process(cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat, laserCloud);
}
/*//}*/

/* //{ callbackMavrosOdometry() */
void callbackMavrosOdometry(const nav_msgs::OdometryConstPtr &msg) {
  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lock(mutex_odom_mavros);
  got_mavros_odom = true;
  mavros_odom     = *msg;
}

//}

/*//{ main() */
int main(int argc, char **argv) {
  ros::init(argc, argv, "ALOAM");
  ros::NodeHandle nh("~");

  nh.param<bool>("debug", debug, false);

  nh.param<int>("scan_line", N_SCANS, 16);
  /* if (N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64) { */
  if (N_SCANS == 32 && N_SCANS == 64) {
    ROS_ERROR("[aloamOdometry] MRS version of aloam does not work properly for %d rows. Function laserCloudHandler() needs to be updated.", N_SCANS);
    return 0;
  } else if (N_SCANS != 16) {
    /* printf("only support velodyne with 16, 32 or 64 scan line!"); */
    ROS_ERROR_STREAM("[aloamOdometry] MRS version of aloam curently supports only 3D lidar with 16 scan lines!");
    return 0;
  }
  ROS_INFO_STREAM("[aloamOdometry] scan line number " << N_SCANS);

  nh.param<bool>("perform_scan_preprocessing", perform_scan_preprocessing, true);
  nh.param<double>("min_range", min_range_sq, 0.0);
  nh.param<double>("max_range", max_range_sq, 1000.0);
  ROS_INFO_STREAM("[aloamOdometry] min range " << min_range_sq);
  ROS_INFO_STREAM("[aloamOdometry] max range " << max_range_sq);
  min_range_sq *= min_range_sq;
  max_range_sq *= max_range_sq;

  nh.param<float>("vertical_fov", vertical_fov_half, 30.0);
  ROS_INFO_STREAM("[aloamOdometry] vertical fov " << vertical_fov_half);
  ray_vert_delta = vertical_fov_half / float(N_SCANS - 1);
  vertical_fov_half /= 2.0;
  ROS_INFO_STREAM("[aloamOdometry] vertical fov delta step " << ray_vert_delta);

  nh.param<double>("frequency", scanPeriod, 10.0);
  ROS_INFO_STREAM("[aloamOdometry] frequency " << scanPeriod);
  scanPeriod = 1.0 / scanPeriod;

  ros::Subscriber o_subLaserCloud = nh.subscribe("/sensor_points", 100, &laserCloudHandler, ros::TransportHints().tcpNoDelay());
  ros::Subscriber o_sub_mavros    = nh.subscribe("/mavros_odom_in", 1, &callbackMavrosOdometry, ros::TransportHints().tcpNoDelay());

  nh.param<int>("mapping_skip_frame", skipFrameNum, 2);

  std::string uav_name;
  nh.getParam("uav_name", uav_name);
  transformer_ = new mrs_lib::Transformer("ALOAM", uav_name);
  nh.getParam("lidar_frame", _frame_lidar);
  nh.getParam("fcu_frame", _frame_fcu);
  nh.getParam("map_frame", _frame_map);

  float lineRes  = 0;
  float planeRes = 0;
  nh.param<float>("mapping_line_resolution", lineRes, 0.4);
  nh.param<float>("mapping_plane_resolution", planeRes, 0.8);

  /* printf("line resolution %f plane resolution %f \n", lineRes, planeRes); */
  ROS_INFO_STREAM("[aloamMapping] line resolution " << lineRes << " plane resolution " << planeRes);
  m_downSizeFilterCorner.setLeafSize(lineRes, lineRes, lineRes);
  m_downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);

  ROS_INFO_STREAM_THROTTLE(ROS_THROTTLE_PERIOD, "[aloamOdometry] Mapping at " << 10 / skipFrameNum << " Hz");

  o_pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 100);
  if (debug) {
    o_pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);
  }

  m_pubLaserCloudMap        = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);
  m_pubLaserCloudFullRes    = nh.advertise<sensor_msgs::PointCloud2>("/sensor_cloud_registered", 100);
  m_pubOdomAftMapped        = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);
  m_pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);

  for (int i = 0; i < m_laserCloudNum; i++) {
    m_laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
    m_laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
  }

  laserPath.header.frame_id = _frame_map;

  // Get static transform lidar->fcu and reverse
  ROS_INFO("[aloamOdometry] Waiting 0.5 second to fill transform buffer.");
  ros::Duration(0.5).sleep();
  // TODO: rewrite to timer
  ROS_INFO_ONCE("[aloamOdometry] Looking for transform from %s to %s", _frame_lidar.c_str(), _frame_fcu.c_str());
  ROS_INFO_ONCE("[aloamMapping] Looking for transform from %s to %s", _frame_fcu.c_str(), _frame_lidar.c_str());
  auto tf_lidar_fcu = transformer_->getTransform(_frame_lidar, _frame_fcu, ros::Time(0));
  while (!tf_lidar_fcu) {
    tf_lidar_fcu = transformer_->getTransform(_frame_lidar, _frame_fcu, ros::Time(0));
  }
  auto tf_fcu_lidar = transformer_->getTransform(_frame_fcu, _frame_lidar, ros::Time(0));
  while (!tf_fcu_lidar) {
    tf_fcu_lidar = transformer_->getTransform(_frame_fcu, _frame_lidar, ros::Time(0));
  }
  tf::transformMsgToTF(tf_lidar_fcu->getTransform().transform, tf_lidar_in_fcu_frame_);
  tf::transformMsgToTF(tf_fcu_lidar->getTransform().transform, tf_fcu_in_lidar_frame_);
  delete transformer_;
  ROS_INFO("[aloamOdometry] Successfully found transformation from %s to %s.", _frame_lidar.c_str(), _frame_fcu.c_str());
  ROS_INFO("[aloamMapping] Successfully found transformation from %s to %s.", _frame_fcu.c_str(), _frame_lidar.c_str());

  is_initialized_ = true;

  ros::spin();
  return 0;
}
/*//}*/
