#include "aloam_slam/mapping.h"

namespace aloam_slam
{

/*//{ AloamMapping() */
AloamMapping::AloamMapping(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::string frame_fcu, std::string frame_map,
                           float scan_frequency, tf::Transform tf_fcu_to_lidar)
    : _frame_fcu(frame_fcu),
      _frame_map(frame_map),
      _scan_frequency(scan_frequency),
      _tf_fcu_to_lidar(tf_fcu_to_lidar),
      _q_w_curr(_parameters),
      _t_w_curr(_parameters + 4) {

  ros::NodeHandle nh_(parent_nh);

  param_loader.loadParam("mapping_line_resolution", _resolution_line, 0.2f);
  param_loader.loadParam("mapping_plane_resolution", _resolution_plane, 0.4f);
  param_loader.loadParam("mapping_rate", _mapping_frequency, 5.0f);
  param_loader.loadParam("map_publish_rate", _map_publish_period, 0.5f);

  _mapping_frequency = std::min(scan_frequency, _mapping_frequency);  // cannot be higher than scan frequency

  _map_publish_period = 1.0f / _map_publish_period;

  _q_wmap_wodom = Eigen::Quaterniond(1, 0, 0, 0);
  _t_wmap_wodom = Eigen::Vector3d(0, 0, 0);

  _q_wodom_curr = Eigen::Quaterniond(1, 0, 0, 0);
  _t_wodom_curr = Eigen::Vector3d(0, 0, 0);

  laserCloudCornerArray.resize(laserCloudNum);
  laserCloudSurfArray.resize(laserCloudNum);
  for (int i = 0; i < laserCloudNum; i++) {
    laserCloudCornerArray.at(i) = boost::make_shared<pcl::PointCloud<PointType>>();
    laserCloudSurfArray.at(i)   = boost::make_shared<pcl::PointCloud<PointType>>();
  }

  _pub_laser_cloud_map        = nh_.advertise<sensor_msgs::PointCloud2>("map_out", 1);
  _pub_laser_cloud_registered = nh_.advertise<sensor_msgs::PointCloud2>("scan_registered_out", 1);
  _pub_odom_global            = nh_.advertise<nav_msgs::Odometry>("odom_global_out", 1);
  _pub_path                   = nh_.advertise<nav_msgs::Path>("path_out", 1);

  if (_mapping_frequency > 0.0f) {
    _timer_mapping_loop = nh_.createTimer(ros::Rate(_mapping_frequency), &AloamMapping::mappingLoop, this, false, true);
  }
  _time_last_map_publish = ros::Time::now();
}
/*//}*/

/*//{ setData() */
void AloamMapping::setData(nav_msgs::Odometry aloam_odometry, pcl::PointCloud<PointType>::Ptr laserCloudCornerLast,
                           pcl::PointCloud<PointType>::Ptr laserCloudSurfLast, pcl::PointCloud<PointType>::Ptr laserCloudFullRes) {
  std::scoped_lock lock(_mutex_odometry_data);
  _has_new_data         = true;
  _aloam_odometry       = aloam_odometry;
  _laserCloudCornerLast = laserCloudCornerLast;
  _laserCloudSurfLast   = laserCloudSurfLast;
  _laserCloudFullRes    = laserCloudFullRes;
}
/*//}*/

/*//{ mappingLoop() */
void AloamMapping::mappingLoop([[maybe_unused]] const ros::TimerEvent &event) {
  if (!is_initialized) {
    return;
  }

  /*//{ Load latest odometry data */
  bool has_new_data;
  {
    std::scoped_lock lock(_mutex_odometry_data);
    has_new_data = _has_new_data;
  }

  if (!has_new_data) {
    return;
  }

  nav_msgs::Odometry              aloam_odometry;
  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
  {
    std::scoped_lock lock(_mutex_odometry_data);
    _has_new_data        = false;
    aloam_odometry       = _aloam_odometry;
    laserCloudCornerLast = _laserCloudCornerLast;
    laserCloudSurfLast   = _laserCloudSurfLast;
    laserCloudFullRes    = _laserCloudFullRes;
  }
  /*//}*/

  _q_wodom_curr.x() = aloam_odometry.pose.pose.orientation.x;
  _q_wodom_curr.y() = aloam_odometry.pose.pose.orientation.y;
  _q_wodom_curr.z() = aloam_odometry.pose.pose.orientation.z;
  _q_wodom_curr.w() = aloam_odometry.pose.pose.orientation.w;
  _t_wodom_curr.x() = aloam_odometry.pose.pose.position.x;
  _t_wodom_curr.y() = aloam_odometry.pose.pose.position.y;
  _t_wodom_curr.z() = aloam_odometry.pose.pose.position.z;

  /*//{ Associate odometry features to map features */
  TicToc t_whole;

  transformAssociateToMap();

  float time_shift        = 0.0f;
  float time_build_tree   = 0.0f;
  float time_association  = 0.0f;
  float time_optimization = 0.0f;
  float time_solver       = 0.0f;

  TicToc t_shift;
  int    centerCubeI = int((_t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
  int    centerCubeJ = int((_t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
  int    centerCubeK = int((_t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

  if (_t_w_curr.x() + 25.0 < 0) {
    centerCubeI--;
  }
  if (_t_w_curr.y() + 25.0 < 0) {
    centerCubeJ--;
  }
  if (_t_w_curr.z() + 25.0 < 0) {
    centerCubeK--;
  }

  while (centerCubeI < 3) {
    for (int j = 0; j < laserCloudHeight; j++) {
      for (int k = 0; k < laserCloudDepth; k++) {
        int                             i = laserCloudWidth - 1;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        for (; i >= 1; i--) {
          laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) =
              laserCloudCornerArray.at(i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
          laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) =
              laserCloudSurfArray.at(i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        }
        laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) = laserCloudCubeCornerPointer;
        laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k)   = laserCloudCubeSurfPointer;
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
        int                             i = 0;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        for (; i < laserCloudWidth - 1; i++) {
          laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) =
              laserCloudCornerArray.at(i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
          laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) =
              laserCloudSurfArray.at(i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        }
        laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) = laserCloudCubeCornerPointer;
        laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k)   = laserCloudCubeSurfPointer;
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
        int                             j = laserCloudHeight - 1;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        for (; j >= 1; j--) {
          laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) =
              laserCloudCornerArray.at(i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k);
          laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) =
              laserCloudSurfArray.at(i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k);
        }
        laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) = laserCloudCubeCornerPointer;
        laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k)   = laserCloudCubeSurfPointer;
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
        int                             j = 0;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        for (; j < laserCloudHeight - 1; j++) {
          laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) =
              laserCloudCornerArray.at(i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k);
          laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) =
              laserCloudSurfArray.at(i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k);
        }
        laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) = laserCloudCubeCornerPointer;
        laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k)   = laserCloudCubeSurfPointer;
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
        int                             k = laserCloudDepth - 1;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        for (; k >= 1; k--) {
          laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) =
              laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1));
          laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) =
              laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1));
        }
        laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) = laserCloudCubeCornerPointer;
        laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k)   = laserCloudCubeSurfPointer;
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
        int                             k = 0;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer = laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        for (; k < laserCloudDepth - 1; k++) {
          laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) =
              laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1));
          laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) =
              laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1));
        }
        laserCloudCornerArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k) = laserCloudCubeCornerPointer;
        laserCloudSurfArray.at(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k)   = laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeK--;
    laserCloudCenDepth--;
  }

  std::vector<int> laserCloudValidInd;

  for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
    for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
      for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++) {
        if (i >= 0 && i < laserCloudWidth && j >= 0 && j < laserCloudHeight && k >= 0 && k < laserCloudDepth) {
          laserCloudValidInd.push_back(i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k);
        }
      }
    }
  }
  /*//}*/

  pcl::PointCloud<PointType>::Ptr m_laserCloudCornerFromMap = boost::make_shared<pcl::PointCloud<PointType>>();
  pcl::PointCloud<PointType>::Ptr m_laserCloudSurfFromMap   = boost::make_shared<pcl::PointCloud<PointType>>();
  for (unsigned int i = 0; i < laserCloudValidInd.size(); i++) {
    *m_laserCloudCornerFromMap += *laserCloudCornerArray.at(laserCloudValidInd.at(i));
    *m_laserCloudSurfFromMap += *laserCloudSurfArray.at(laserCloudValidInd.at(i));
  }
  int laserCloudCornerFromMapNum = m_laserCloudCornerFromMap->points.size();
  int laserCloudSurfFromMapNum   = m_laserCloudSurfFromMap->points.size();

  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  downSizeFilterCorner.setLeafSize(_resolution_line, _resolution_line, _resolution_line);
  downSizeFilterSurf.setLeafSize(_resolution_plane, _resolution_plane, _resolution_plane);

  /*//{*/
  pcl::PointCloud<PointType>::Ptr laserCloudCornerStack = boost::make_shared<pcl::PointCloud<PointType>>();
  downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
  downSizeFilterCorner.filter(*laserCloudCornerStack);

  pcl::PointCloud<PointType>::Ptr laserCloudSurfStack = boost::make_shared<pcl::PointCloud<PointType>>();
  downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
  downSizeFilterSurf.filter(*laserCloudSurfStack);

  /* printf("map prepare time %f ms\n", t_shift.toc()); */
  /* printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum); */
  time_shift = t_shift.toc();

  if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50) {
    TicToc                           t_tree;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());
    kdtreeCornerFromMap->setInputCloud(m_laserCloudCornerFromMap);
    kdtreeSurfFromMap->setInputCloud(m_laserCloudSurfFromMap);
    time_build_tree = t_tree.toc();

    std::vector<int>   pointSearchInd;
    std::vector<float> pointSearchSqDis;

    TicToc t_opt;
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

      for (unsigned int i = 0; i < laserCloudCornerStack->points.size(); i++) {
        _pointOri = laserCloudCornerStack->points.at(i);
        // double sqrtDis = _pointOri.x * _pointOri.x + _pointOri.y * _pointOri.y + _pointOri.z * _pointOri.z;
        pointAssociateToMap(&_pointOri, &_pointSel);
        kdtreeCornerFromMap->nearestKSearch(_pointSel, 5, pointSearchInd, pointSearchSqDis);

        if (pointSearchSqDis.at(4) < 1.0) {
          std::vector<Eigen::Vector3d> nearCorners;
          Eigen::Vector3d              center(0, 0, 0);
          for (int j = 0; j < 5; j++) {
            Eigen::Vector3d tmp(m_laserCloudCornerFromMap->points.at(pointSearchInd.at(j)).x, m_laserCloudCornerFromMap->points.at(pointSearchInd.at(j)).y,
                                m_laserCloudCornerFromMap->points.at(pointSearchInd.at(j)).z);
            center = center + tmp;
            nearCorners.push_back(tmp);
          }
          center = center / 5.0;

          Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
          for (int j = 0; j < 5; j++) {
            Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners.at(j) - center;
            covMat                                  = covMat + tmpZeroMean * tmpZeroMean.transpose();
          }

          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

          // if is indeed line feature
          // note Eigen library sort eigenvalues in increasing order
          Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
          Eigen::Vector3d curr_point(_pointOri.x, _pointOri.y, _pointOri.z);
          if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
            Eigen::Vector3d point_a = 0.1 * unit_direction + center;
            Eigen::Vector3d point_b = -0.1 * unit_direction + center;

            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
            problem.AddResidualBlock(cost_function, loss_function, _parameters, _parameters + 4);
            corner_num++;
          }
        }
      }

      int surf_num = 0;
      for (unsigned int i = 0; i < laserCloudSurfStack->points.size(); i++) {
        _pointOri = laserCloudSurfStack->points.at(i);
        // double sqrtDis = _pointOri.x * _pointOri.x + _pointOri.y * _pointOri.y + _pointOri.z * _pointOri.z;
        pointAssociateToMap(&_pointOri, &_pointSel);
        kdtreeSurfFromMap->nearestKSearch(_pointSel, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis.at(4) < 1.0) {
          for (int j = 0; j < 5; j++) {
            matA0(j, 0) = m_laserCloudSurfFromMap->points.at(pointSearchInd.at(j)).x;
            matA0(j, 1) = m_laserCloudSurfFromMap->points.at(pointSearchInd.at(j)).y;
            matA0(j, 2) = m_laserCloudSurfFromMap->points.at(pointSearchInd.at(j)).z;
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
            if (fabs(norm(0) * m_laserCloudSurfFromMap->points.at(pointSearchInd.at(j)).x +
                     norm(1) * m_laserCloudSurfFromMap->points.at(pointSearchInd.at(j)).y +
                     norm(2) * m_laserCloudSurfFromMap->points.at(pointSearchInd.at(j)).z + negative_OA_dot_norm) > 0.2) {
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
      }
      /* printf("mapping data assosiation time %f ms \n", t_data.toc()); */
      time_association += t_data.toc();

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
      time_solver += t_solver.toc();

      // printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
      // printf("result q %f %f %f %f result t %f %f %f\n", _parameters[3], _parameters[0], _parameters[1], _parameters[2],
      //	   _parameters[4], _parameters[5], _parameters[6]);
    }
    /* printf("mapping optimization time %f \n", t_opt.toc()); */
    time_optimization = t_opt.toc();
  } else {
    ROS_WARN("[AloamMapping] Not enough map correspondences. Skipping mapping frame.");
  }
  /*//}*/

  transformUpdate();

  float  time_add = 0.0f;
  TicToc t_add;
  for (unsigned int i = 0; i < laserCloudCornerStack->points.size(); i++) {
    pointAssociateToMap(&laserCloudCornerStack->points.at(i), &_pointSel);

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
      laserCloudCornerArray.at(cubeInd)->push_back(_pointSel);
    }
  }

  for (unsigned int i = 0; i < laserCloudSurfStack->points.size(); i++) {
    pointAssociateToMap(&laserCloudSurfStack->points.at(i), &_pointSel);

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
      laserCloudSurfArray.at(cubeInd)->push_back(_pointSel);
    }
  }
  /* printf("add points time %f ms\n", t_add.toc()); */
  time_add = t_add.toc();

  float  time_filter = 0.0f;
  TicToc t_filter;
  for (unsigned int i = 0; i < laserCloudValidInd.size(); i++) {
    int ind = laserCloudValidInd.at(i);

    pcl::PointCloud<PointType>::Ptr tmpCorner = boost::make_shared<pcl::PointCloud<PointType>>();
    downSizeFilterCorner.setInputCloud(laserCloudCornerArray.at(ind));
    downSizeFilterCorner.filter(*tmpCorner);
    laserCloudCornerArray.at(ind) = tmpCorner;

    pcl::PointCloud<PointType>::Ptr tmpSurf = boost::make_shared<pcl::PointCloud<PointType>>();
    downSizeFilterSurf.setInputCloud(laserCloudSurfArray.at(ind));
    downSizeFilterSurf.filter(*tmpSurf);
    laserCloudSurfArray.at(ind) = tmpSurf;
  }
  time_filter = t_filter.toc();

  /*//{ Publish data */

  TicToc    t_pub;
  ros::Time stamp = aloam_odometry.header.stamp;

  // Publish TF
  // TF fcu -> aloam origin (_t_w_curr and _q_w_curr is in origin -> fcu frame, hence inversion)
  static tf::TransformBroadcaster br;
  tf::Transform                   transform;
  tf::Quaternion                  q;
  transform.setOrigin(tf::Vector3(_t_w_curr(0), _t_w_curr(1), _t_w_curr(2)));
  tf::quaternionEigenToTF(_q_w_curr, q);
  transform.setRotation(q);

  tf::Transform tf_fcu = transform * _tf_fcu_to_lidar;
  br.sendTransform(tf::StampedTransform(tf_fcu.inverse(), stamp, _frame_fcu, _frame_map));

  if (_pub_odom_global.getNumSubscribers() > 0 || _pub_path.getNumSubscribers() > 0) {
    // Publish nav_msgs::Odometry msg
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

    // Publish nav_msgs::Path msg
    geometry_msgs::PoseStamped laserAfterMappedPose;
    laserAfterMappedPose.header = odomAftMapped.header;
    laserAfterMappedPose.pose   = odomAftMapped.pose.pose;
    laserAfterMappedPath.header = odomAftMapped.header;
    laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
    _pub_path.publish(laserAfterMappedPath);
  }

  // Publish entire map
  if (_pub_laser_cloud_map.getNumSubscribers() > 0 && (ros::Time::now() - _time_last_map_publish).toSec() > _map_publish_period) {
    pcl::PointCloud<PointType> laserCloudMap;
    for (int i = 0; i < laserCloudNum; i++) {
      laserCloudMap += *laserCloudCornerArray.at(i);
      laserCloudMap += *laserCloudSurfArray.at(i);
    }
    sensor_msgs::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(laserCloudMap, laserCloudMsg);
    laserCloudMsg.header.stamp    = stamp;
    laserCloudMsg.header.frame_id = _frame_map;
    _pub_laser_cloud_map.publish(laserCloudMsg);

    _time_last_map_publish = ros::Time::now();
  }

  // Publish registered sensor data
  if (_pub_laser_cloud_registered.getNumSubscribers() > 0) {
    // TODO: this pcl might be published in lidar frame instead of map saving some load..
    // or it might not be published at all as the data are already published by sensor and TFs are published above
    int laserCloudFullResNum = laserCloudFullRes->points.size();
    for (int i = 0; i < laserCloudFullResNum; i++) {
      pointAssociateToMap(&laserCloudFullRes->points.at(i), &laserCloudFullRes->points.at(i));
    }

    sensor_msgs::PointCloud2::Ptr laserCloudFullResMsg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*laserCloudFullRes, *laserCloudFullResMsg);
    laserCloudFullResMsg->header.stamp    = stamp;
    laserCloudFullResMsg->header.frame_id = _frame_map;
    _pub_laser_cloud_registered.publish(laserCloudFullResMsg);
  }

  /*//}*/

  float time_whole = t_whole.toc();

  ROS_INFO_THROTTLE(1.0, "[AloamMapping] Run time: %0.1f ms (%0.1f Hz)", time_whole, std::min(_mapping_frequency, 1000.0f / time_whole));
  ROS_DEBUG_THROTTLE(
      1.0,
      "[AloamMapping] features -> edges: %d, surfs: %d; data preparation: %0.1f ms; kd-tree initialization: %0.1f ms; "
      "publishing time: %0.1f ms; map -> association: %0.1f ms, solver: %0.1f ms, optimization: %0.1f ms, adding points: %0.1f ms, filtering: %0.1f ms",
      laserCloudCornerFromMapNum, laserCloudSurfFromMapNum, time_shift, time_build_tree, t_pub.toc(), time_association, time_solver, time_optimization,
      time_add, time_filter);

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

}  // namespace aloam_slam

