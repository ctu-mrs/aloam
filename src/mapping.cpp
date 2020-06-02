#include "aloam_slam/mapping.h"

namespace aloam_slam
{

/* //{ class AloamMapping */

/*//{ AloamMapping() */
AloamMapping::AloamMapping(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::string frame_fcu, std::string frame_map,
                           tf::Transform tf_fcu_to_lidar)
    : _frame_fcu(frame_fcu), _frame_map(frame_map), _tf_fcu_to_lidar(tf_fcu_to_lidar), _q_w_curr(_parameters), _t_w_curr(_parameters + 4) {

  ros::NodeHandle nh_(parent_nh, "AloamMapping");

  float line_resolution;
  float plane_resolution;
  param_loader.loadParam("mapping_line_resolution", line_resolution, 0.2f);
  param_loader.loadParam("mapping_plane_resolution", plane_resolution, 0.4f);

  _q_wmap_wodom = Eigen::Quaterniond(1, 0, 0, 0);
  _t_wmap_wodom = Eigen::Vector3d(0, 0, 0);

  _q_wodom_curr = Eigen::Quaterniond(1, 0, 0, 0);
  _t_wodom_curr = Eigen::Vector3d(0, 0, 0);

  /* _q_w_curr = Eigen::Map<Eigen::Quaterniond>(_parameters); */
  /* _t_w_curr = Eigen::Map<Eigen::Vector3d>(_parameters + 4); */

  downSizeFilterCorner.setLeafSize(line_resolution, line_resolution, line_resolution);
  downSizeFilterSurf.setLeafSize(plane_resolution, plane_resolution, plane_resolution);

  laserCloudCornerArray.resize(laserCloudNum);
  laserCloudSurfArray.resize(laserCloudNum);
  for (int i = 0; i < laserCloudNum; i++) {
    laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
    laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
  }

  _pub_laser_cloud_map        = nh_.advertise<sensor_msgs::PointCloud2>("map_out", 1);
  _pub_laser_cloud_registered = nh_.advertise<sensor_msgs::PointCloud2>("scan_registered_out", 1);
  _pub_odom_global            = nh_.advertise<nav_msgs::Odometry>("odom_global_out", 1);
  _pub_path                   = nh_.advertise<nav_msgs::Path>("path_out", 1);
}
/*//}*/

/*//{ compute_mapping() */
void AloamMapping::compute_mapping(nav_msgs::Odometry odometry, pcl::PointCloud<PointType>::Ptr laserCloudCornerLast,
                                   pcl::PointCloud<PointType>::Ptr laserCloudSurfLast, pcl::PointCloud<PointType>::Ptr laserCloudFullRes) {

  ros::Time stamp_cornerLastBuf;
  ros::Time stamp_surfLastBuf;
  ros::Time stamp_fullResBuf;
  pcl_conversions::fromPCL(laserCloudCornerLast->header.stamp, stamp_cornerLastBuf);
  pcl_conversions::fromPCL(laserCloudSurfLast->header.stamp, stamp_surfLastBuf);
  pcl_conversions::fromPCL(laserCloudFullRes->header.stamp, stamp_fullResBuf);
  double timeLaserCloudCornerLast = stamp_cornerLastBuf.toSec();
  double timeLaserCloudSurfLast   = stamp_surfLastBuf.toSec();
  double timeLaserCloudFullRes    = stamp_fullResBuf.toSec();
  double timeLaserOdometry        = odometry.header.stamp.toSec();

  if (timeLaserCloudCornerLast != timeLaserOdometry || timeLaserCloudSurfLast != timeLaserOdometry || timeLaserCloudFullRes != timeLaserOdometry) {
    ROS_WARN_STREAM("[aloamMapping] Unsync message (should happen just once): time corner "
                    << timeLaserCloudCornerLast << " surf " << timeLaserCloudSurfLast << " full " << timeLaserCloudFullRes << " odom " << timeLaserOdometry);
    return;
  }

  _q_wodom_curr.x() = odometry.pose.pose.orientation.x;
  _q_wodom_curr.y() = odometry.pose.pose.orientation.y;
  _q_wodom_curr.z() = odometry.pose.pose.orientation.z;
  _q_wodom_curr.w() = odometry.pose.pose.orientation.w;
  _t_wodom_curr.x() = odometry.pose.pose.position.x;
  _t_wodom_curr.y() = odometry.pose.pose.position.y;
  _t_wodom_curr.z() = odometry.pose.pose.position.z;

  /*//{*/
  TicToc t_whole;

  transformAssociateToMap();

  TicToc t_shift;
  int    centerCubeI = int((_t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
  int    centerCubeJ = int((_t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
  int    centerCubeK = int((_t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

  if (_t_w_curr.x() + 25.0 < 0)
    centerCubeI--;
  if (_t_w_curr.y() + 25.0 < 0)
    centerCubeJ--;
  if (_t_w_curr.z() + 25.0 < 0)
    centerCubeK--;

  while (centerCubeI < 3) {
    for (int j = 0; j < laserCloudHeight; j++) {
      for (int k = 0; k < laserCloudDepth; k++) {
        int                             i                           = laserCloudWidth - 1;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer   = laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        for (; i >= 1; i--) {
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        }
        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]   = laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeI++;
    laserCloudCenWidth++;
  }

  while (centerCubeI >= laserCloudWidth - 3) {
    for (int j = 0; j < laserCloudHeight; j++) {
      for (int k = 0; k < laserCloudDepth; k++) {
        int                             i                           = 0;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer   = laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        for (; i < laserCloudWidth - 1; i++) {
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        }
        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]   = laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeI--;
    laserCloudCenWidth--;
  }

  while (centerCubeJ < 3) {
    for (int i = 0; i < laserCloudWidth; i++) {
      for (int k = 0; k < laserCloudDepth; k++) {
        int                             j                           = laserCloudHeight - 1;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer   = laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        for (; j >= 1; j--) {
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
        }
        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]   = laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeJ++;
    laserCloudCenHeight++;
  }

  while (centerCubeJ >= laserCloudHeight - 3) {
    for (int i = 0; i < laserCloudWidth; i++) {
      for (int k = 0; k < laserCloudDepth; k++) {
        int                             j                           = 0;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer   = laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        for (; j < laserCloudHeight - 1; j++) {
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
        }
        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]   = laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeJ--;
    laserCloudCenHeight--;
  }

  while (centerCubeK < 3) {
    for (int i = 0; i < laserCloudWidth; i++) {
      for (int j = 0; j < laserCloudHeight; j++) {
        int                             k                           = laserCloudDepth - 1;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer   = laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        for (; k >= 1; k--) {
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
        }
        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]   = laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeK++;
    laserCloudCenDepth++;
  }

  while (centerCubeK >= laserCloudDepth - 3) {
    for (int i = 0; i < laserCloudWidth; i++) {
      for (int j = 0; j < laserCloudHeight; j++) {
        int                             k                           = 0;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer = laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer   = laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
        for (; k < laserCloudDepth - 1; k++) {
          laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
          laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
        }
        laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] = laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]   = laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeK--;
    laserCloudCenDepth--;
  }

  int laserCloudValidNum = 0;

  for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
    for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
      for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++) {
        if (i >= 0 && i < laserCloudWidth && j >= 0 && j < laserCloudHeight && k >= 0 && k < laserCloudDepth) {
          laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
          laserCloudValidNum++;
        }
      }
    }
  }
  /*//}*/

  pcl::PointCloud<PointType>::Ptr m_laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr m_laserCloudSurfFromMap(new pcl::PointCloud<PointType>());
  for (int i = 0; i < laserCloudValidNum; i++) {
    *m_laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
    *m_laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
  }
  int laserCloudCornerFromMapNum = m_laserCloudCornerFromMap->points.size();
  int laserCloudSurfFromMapNum   = m_laserCloudSurfFromMap->points.size();

  /*//{*/
  pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
  downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
  downSizeFilterCorner.filter(*laserCloudCornerStack);
  int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

  pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
  downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
  downSizeFilterSurf.filter(*laserCloudSurfStack);
  int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

  /* printf("map prepare time %f ms\n", t_shift.toc()); */
  /* printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum); */
  ROS_INFO_STREAM_THROTTLE(1.0, "[aloamMapping] map prepare time " << t_shift.toc() << " ms");
  ROS_INFO_STREAM_THROTTLE(1.0, "[aloamMapping] map corner num " << laserCloudCornerFromMapNum << " surf num " << laserCloudSurfFromMapNum);
  if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50) {
    TicToc                           t_opt;
    TicToc                           t_tree;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());
    kdtreeCornerFromMap->setInputCloud(m_laserCloudCornerFromMap);
    kdtreeSurfFromMap->setInputCloud(m_laserCloudSurfFromMap);
    /* printf("build tree time %f ms \n", t_tree.toc()); */
    ROS_INFO_STREAM_THROTTLE(1.0, "[aloamMapping] build tree time " << t_tree.toc() << " ms");

    for (int iterCount = 0; iterCount < 2; iterCount++) {
      // ceres::LossFunction *loss_function = NULL;
      ceres::LossFunction *         loss_function      = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
      ceres::Problem::Options       problem_options;

      ceres::Problem problem(problem_options);
      problem.AddParameterBlock(_parameters, 4, q_parameterization);
      problem.AddParameterBlock(_parameters + 4, 3);

      TicToc t_data;
      int    corner_num = 0;

      for (int i = 0; i < laserCloudCornerStackNum; i++) {
        _pointOri = laserCloudCornerStack->points[i];
        // double sqrtDis = _pointOri.x * _pointOri.x + _pointOri.y * _pointOri.y + _pointOri.z * _pointOri.z;
        pointAssociateToMap(&_pointOri, &_pointSel);
        kdtreeCornerFromMap->nearestKSearch(_pointSel, 5, _pointSearchInd, m_pointSearchSqDis);

        if (m_pointSearchSqDis[4] < 1.0) {
          std::vector<Eigen::Vector3d> nearCorners;
          Eigen::Vector3d              center(0, 0, 0);
          for (int j = 0; j < 5; j++) {
            Eigen::Vector3d tmp(m_laserCloudCornerFromMap->points[_pointSearchInd[j]].x, m_laserCloudCornerFromMap->points[_pointSearchInd[j]].y,
                                m_laserCloudCornerFromMap->points[_pointSearchInd[j]].z);
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
          Eigen::Vector3d curr_point(_pointOri.x, _pointOri.y, _pointOri.z);
          if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
            Eigen::Vector3d point_on_line = center;
            Eigen::Vector3d point_a, point_b;
            point_a = 0.1 * unit_direction + point_on_line;
            point_b = -0.1 * unit_direction + point_on_line;

            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
            problem.AddResidualBlock(cost_function, loss_function, _parameters, _parameters + 4);
            corner_num++;
          }
        }
        /*
        else if(m_pointSearchSqDis[4] < 0.01 * sqrtDis)
        {
                Eigen::Vector3d center(0, 0, 0);
                for (int j = 0; j < 5; j++)
                {
                        Eigen::Vector3d tmp(m_laserCloudCornerFromMap->points[_pointSearchInd[j]].x,
                                                                m_laserCloudCornerFromMap->points[_pointSearchInd[j]].y,
                                                                m_laserCloudCornerFromMap->points[_pointSearchInd[j]].z);
                        center = center + tmp;
                }
                center = center / 5.0;
                Eigen::Vector3d curr_point(_pointOri.x, _pointOri.y, _pointOri.z);
                ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
                problem.AddResidualBlock(cost_function, loss_function, _parameters, _parameters + 4);
        }
        */
      }

      int surf_num = 0;
      for (int i = 0; i < laserCloudSurfStackNum; i++) {
        _pointOri = laserCloudSurfStack->points[i];
        // double sqrtDis = _pointOri.x * _pointOri.x + _pointOri.y * _pointOri.y + _pointOri.z * _pointOri.z;
        pointAssociateToMap(&_pointOri, &_pointSel);
        kdtreeSurfFromMap->nearestKSearch(_pointSel, 5, _pointSearchInd, m_pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (m_pointSearchSqDis[4] < 1.0) {

          for (int j = 0; j < 5; j++) {
            matA0(j, 0) = m_laserCloudSurfFromMap->points[_pointSearchInd[j]].x;
            matA0(j, 1) = m_laserCloudSurfFromMap->points[_pointSearchInd[j]].y;
            matA0(j, 2) = m_laserCloudSurfFromMap->points[_pointSearchInd[j]].z;
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
            if (fabs(norm(0) * m_laserCloudSurfFromMap->points[_pointSearchInd[j]].x + norm(1) * m_laserCloudSurfFromMap->points[_pointSearchInd[j]].y +
                     norm(2) * m_laserCloudSurfFromMap->points[_pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2) {
              planeValid = false;
              break;
            }
          }
          Eigen::Vector3d curr_point(_pointOri.x, _pointOri.y, _pointOri.z);
          if (planeValid) {
            ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
            problem.AddResidualBlock(cost_function, loss_function, _parameters, _parameters + 4);
            surf_num++;
          }
        }
        /*
        else if(m_pointSearchSqDis[4] < 0.01 * sqrtDis)
        {
                Eigen::Vector3d center(0, 0, 0);
                for (int j = 0; j < 5; j++)
                {
                        Eigen::Vector3d tmp(m_laserCloudSurfFromMap->points[_pointSearchInd[j]].x,
                                                                m_laserCloudSurfFromMap->points[_pointSearchInd[j]].y,
                                                                m_laserCloudSurfFromMap->points[_pointSearchInd[j]].z);
                        center = center + tmp;
                }
                center = center / 5.0;
                Eigen::Vector3d curr_point(_pointOri.x, _pointOri.y, _pointOri.z);
                ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
                problem.AddResidualBlock(cost_function, loss_function, _parameters, _parameters + 4);
        }
        */
      }

      // printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
      // printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);

      /* printf("mapping data assosiation time %f ms \n", t_data.toc()); */
      ROS_INFO_STREAM_THROTTLE(1.0, "[aloamMapping] mapping data association time " << t_data.toc() << " ms");

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
      ROS_INFO_STREAM_THROTTLE(1.0, "[aloamMapping] mapping solver time " << t_solver.toc() << " ms");

      // printf("time %f \n", timeLaserOdometry);
      // printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
      // printf("result q %f %f %f %f result t %f %f %f\n", _parameters[3], _parameters[0], _parameters[1], _parameters[2],
      //	   _parameters[4], _parameters[5], _parameters[6]);
    }
    /* printf("mapping optimization time %f \n", t_opt.toc()); */
    ROS_INFO_STREAM_THROTTLE(1.0, "[aloamMapping] mapping optimization time " << t_opt.toc() << " ms");
  } else {
    ROS_WARN("[aloamMapping] time Map corner and surf num are not enough");
  }
  transformUpdate();
  /*//}*/

  TicToc t_add;
  for (int i = 0; i < laserCloudCornerStackNum; i++) {
    pointAssociateToMap(&laserCloudCornerStack->points[i], &_pointSel);

    int cubeI = int((_pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
    int cubeJ = int((_pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
    int cubeK = int((_pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

    if (_pointSel.x + 25.0 < 0)
      cubeI--;
    if (_pointSel.y + 25.0 < 0)
      cubeJ--;
    if (_pointSel.z + 25.0 < 0)
      cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 && cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth) {
      int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
      laserCloudCornerArray[cubeInd]->push_back(_pointSel);
    }
  }

  for (int i = 0; i < laserCloudSurfStackNum; i++) {
    pointAssociateToMap(&laserCloudSurfStack->points[i], &_pointSel);

    int cubeI = int((_pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
    int cubeJ = int((_pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
    int cubeK = int((_pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

    if (_pointSel.x + 25.0 < 0)
      cubeI--;
    if (_pointSel.y + 25.0 < 0)
      cubeJ--;
    if (_pointSel.z + 25.0 < 0)
      cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 && cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth) {
      int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
      laserCloudSurfArray[cubeInd]->push_back(_pointSel);
    }
  }
  /* printf("add points time %f ms\n", t_add.toc()); */
  ROS_INFO_STREAM_THROTTLE(1.0, "[aloamMapping] add points time " << t_add.toc() << " ms");


  TicToc t_filter;
  for (int i = 0; i < laserCloudValidNum; i++) {
    int ind = laserCloudValidInd[i];

    pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
    downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
    downSizeFilterCorner.filter(*tmpCorner);
    laserCloudCornerArray[ind] = tmpCorner;

    pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
    downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
    downSizeFilterSurf.filter(*tmpSurf);
    laserCloudSurfArray[ind] = tmpSurf;
  }
  /* printf("filter time %f ms \n", t_filter.toc()); */
  ROS_INFO_STREAM_THROTTLE(1.0, "[aloamMapping] filter time " << t_filter.toc() << " ms");

  TicToc    t_pub;
  ros::Time stamp = ros::Time().fromSec(timeLaserOdometry);

  if (_frame_count % 20 == 0) {
    pcl::PointCloud<PointType> laserCloudMap;
    for (int i = 0; i < 4851; i++) {
      laserCloudMap += *laserCloudCornerArray[i];
      laserCloudMap += *laserCloudSurfArray[i];
    }
    sensor_msgs::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(laserCloudMap, laserCloudMsg);
    laserCloudMsg.header.stamp    = stamp;
    laserCloudMsg.header.frame_id = _frame_map;
    _pub_laser_cloud_map.publish(laserCloudMsg);
  }

  int laserCloudFullResNum = laserCloudFullRes->points.size();
  for (int i = 0; i < laserCloudFullResNum; i++) {
    pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
  }

  sensor_msgs::PointCloud2::Ptr laserCloudFullRes3 = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*laserCloudFullRes, *laserCloudFullRes3);
  laserCloudFullRes3->header.stamp    = stamp;
  laserCloudFullRes3->header.frame_id = _frame_map;
  _pub_laser_cloud_registered.publish(laserCloudFullRes3);

  /* printf("mapping pub time %f ms \n", t_pub.toc()); */
  /* printf("whole mapping time %f ms +++++\n", t_whole.toc()); */
  ROS_INFO_STREAM_THROTTLE(1.0, "[aloamMapping] mapping pub time " << t_pub.toc() << " ms");
  ROS_INFO_STREAM_THROTTLE(1.0, "[aloamMapping] whole mapping time " << t_whole.toc() << " ms");

  // TF fcu -> aloam origin (_t_w_curr and _q_w_curr is in origin -> fcu frame, hence inversion)
  static tf::TransformBroadcaster br;
  tf::Transform                   transform;
  tf::Quaternion                  q;
  transform.setOrigin(tf::Vector3(_t_w_curr(0), _t_w_curr(1), _t_w_curr(2)));
  tf::quaternionEigenToTF(_q_w_curr, q);
  transform.setRotation(q);

  tf::Transform tf_fcu = transform * _tf_fcu_to_lidar;
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
  _pub_odom_global.publish(odomAftMapped);

  // Odometry path
  geometry_msgs::PoseStamped laserAfterMappedPose;
  laserAfterMappedPose.header = odomAftMapped.header;
  laserAfterMappedPose.pose   = odomAftMapped.pose.pose;
  laserAfterMappedPath.header = odomAftMapped.header;
  laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
  _pub_path.publish(laserAfterMappedPath);

  _frame_count++;
}
/*//}*/

/*//{ transformAssociateToMap() */
void AloamMapping::transformAssociateToMap() {
  _q_w_curr = _q_wmap_wodom * _q_wodom_curr;
  _t_w_curr = _q_wmap_wodom * _t_wodom_curr + _t_wmap_wodom;
}
/*//}*/

/*//{ transformUpdate() */
void AloamMapping::transformUpdate() {
  _q_wmap_wodom = _q_w_curr * _q_wodom_curr.inverse();
  _t_wmap_wodom = _t_w_curr - _q_wmap_wodom * _t_wodom_curr;
}
/*//}*/

/*//{ pointAssociateToMap() */
void AloamMapping::pointAssociateToMap(PointType const *const pi, PointType *const po) {
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = _q_w_curr * point_curr + _t_w_curr;
  po->x                   = point_w.x();
  po->y                   = point_w.y();
  po->z                   = point_w.z();
  po->intensity           = pi->intensity;
  // po->intensity = 1.0;
}
/*//}*/

/*//{ pointAssociateTobeMapped() */
void AloamMapping::pointAssociateTobeMapped(PointType const *const pi, PointType *const po) {
  Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_curr = _q_w_curr.inverse() * (point_w - _t_w_curr);
  po->x                      = point_curr.x();
  po->y                      = point_curr.y();
  po->z                      = point_curr.z();
  po->intensity              = pi->intensity;
}
/*//}*/

//}
}  // namespace aloam_slam

