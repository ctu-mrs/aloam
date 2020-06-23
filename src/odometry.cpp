#include "aloam_slam/odometry.h"
#include "aloam_slam/mapping.h"

namespace aloam_slam
{

/* //{ AloamOdometry() */
AloamOdometry::AloamOdometry(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_lib::Profiler> profiler, std::shared_ptr<AloamMapping> mapper, std::string frame_lidar,
                             std::string frame_map, float scan_period_sec, tf::Transform tf_lidar_to_fcu)
    : _profiler(profiler),
      _mapper(mapper),
      _frame_lidar(frame_lidar),
      _frame_map(frame_map),
      _scan_period_sec(scan_period_sec),
      _tf_lidar_to_fcu(tf_lidar_to_fcu),
      q_last_curr(para_q),
      t_last_curr(para_t) {

  ros::NodeHandle nh_(parent_nh);

  // Objects initialization
  laserCloudCornerLast = boost::make_shared<pcl::PointCloud<PointType>>();
  laserCloudSurfLast   = boost::make_shared<pcl::PointCloud<PointType>>();
  kdtreeCornerLast     = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
  kdtreeSurfLast       = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();

  q_w_curr = Eigen::Quaterniond(1, 0, 0, 0);  // eigen has qw, qx, qy, qz notation
  t_w_curr = Eigen::Vector3d(0, 0, 0);

  _sub_orientation_meas = nh_.subscribe("orientation_in", 1, &AloamOdometry::callbackOrientationMeas, this, ros::TransportHints().tcpNoDelay());

  _pub_odometry_local = nh_.advertise<nav_msgs::Odometry>("odom_local_out", 1);

  _timer_odometry_loop = nh_.createTimer(ros::Rate(1000), &AloamOdometry::timerOdometry, this, false, true);
}
//}

/*//{ timerOdometry() */
void AloamOdometry::timerOdometry([[maybe_unused]] const ros::TimerEvent &event) {
  if (!is_initialized) {
    return;
  }

  /*//{ Load latest features */
  bool has_new_data;
  {
    std::scoped_lock lock(_mutex_features_data);
    has_new_data = _has_new_data;
  }

  if (!has_new_data) {
    return;
  }

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;
  pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
  {
    std::scoped_lock lock(_mutex_features_data);
    _has_new_data         = false;
    cornerPointsSharp     = _cornerPointsSharp;
    cornerPointsLessSharp = _cornerPointsLessSharp;
    surfPointsFlat        = _surfPointsFlat;
    surfPointsLessFlat    = _surfPointsLessFlat;
    laserCloudFullRes     = _laserCloudFullRes;
  }
  /*//}*/

  mrs_lib::Routine profiler_routine = _profiler->createRoutine("timerOdometry", 1.0f / _scan_period_sec, 0.05, event);

  ros::Time stamp;
  pcl_conversions::fromPCL(laserCloudFullRes->header.stamp, stamp);

  TicToc t_whole;

  /*//{ Find features correspondences and compute local odometry */

  float time_data_association = 0.0f;
  float time_solver           = 0.0f;
  float time_opt              = 0.0f;

  if (_frame_count > 0) {
    int cornerPointsSharpNum = cornerPointsSharp->points.size();
    int surfPointsFlatNum    = surfPointsFlat->points.size();

    TicToc t_opt;
    for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {
      int corner_correspondence = 0;
      int plane_correspondence  = 0;

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
        TransformToStart(&(cornerPointsSharp->points.at(i)), &pointSel);
        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

        int closestPointInd = -1, minPointInd2 = -1;
        if (pointSearchSqDis.at(0) < DISTANCE_SQ_THRESHOLD) {
          closestPointInd        = pointSearchInd.at(0);
          int closestPointScanID = int(laserCloudCornerLast->points.at(closestPointInd).intensity);

          double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
          // search in the direction of increasing scan line
          for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast->points.size(); ++j) {
            // if in the same scan line, continue
            if (int(laserCloudCornerLast->points.at(j).intensity) <= closestPointScanID)
              continue;

            // if not in nearby scans, end the loop
            if (int(laserCloudCornerLast->points.at(j).intensity) > (closestPointScanID + NEARBY_SCAN))
              break;

            double pointSqDis = (laserCloudCornerLast->points.at(j).x - pointSel.x) * (laserCloudCornerLast->points.at(j).x - pointSel.x) +
                                (laserCloudCornerLast->points.at(j).y - pointSel.y) * (laserCloudCornerLast->points.at(j).y - pointSel.y) +
                                (laserCloudCornerLast->points.at(j).z - pointSel.z) * (laserCloudCornerLast->points.at(j).z - pointSel.z);

            if (pointSqDis < minPointSqDis2) {
              // find nearer point
              minPointSqDis2 = pointSqDis;
              minPointInd2   = j;
            }
          }

          // search in the direction of decreasing scan line
          for (int j = closestPointInd - 1; j >= 0; --j) {
            // if in the same scan line, continue
            if (int(laserCloudCornerLast->points.at(j).intensity) >= closestPointScanID) {
              continue;
            }

            // if not in nearby scans, end the loop
            if (int(laserCloudCornerLast->points.at(j).intensity) < (closestPointScanID - NEARBY_SCAN)) {
              break;
            }

            double pointSqDis = (laserCloudCornerLast->points.at(j).x - pointSel.x) * (laserCloudCornerLast->points.at(j).x - pointSel.x) +
                                (laserCloudCornerLast->points.at(j).y - pointSel.y) * (laserCloudCornerLast->points.at(j).y - pointSel.y) +
                                (laserCloudCornerLast->points.at(j).z - pointSel.z) * (laserCloudCornerLast->points.at(j).z - pointSel.z);

            if (pointSqDis < minPointSqDis2) {
              // find nearer point
              minPointSqDis2 = pointSqDis;
              minPointInd2   = j;
            }
          }
        }
        if (minPointInd2 >= 0)  // both closestPointInd and minPointInd2 is valid
        {
          Eigen::Vector3d curr_point(cornerPointsSharp->points.at(i).x, cornerPointsSharp->points.at(i).y, cornerPointsSharp->points.at(i).z);
          Eigen::Vector3d last_point_a(laserCloudCornerLast->points.at(closestPointInd).x, laserCloudCornerLast->points.at(closestPointInd).y,
                                       laserCloudCornerLast->points.at(closestPointInd).z);
          Eigen::Vector3d last_point_b(laserCloudCornerLast->points.at(minPointInd2).x, laserCloudCornerLast->points.at(minPointInd2).y,
                                       laserCloudCornerLast->points.at(minPointInd2).z);

          double s = 1.0;
          if (DISTORTION) {
            s = (cornerPointsSharp->points.at(i).intensity - int(cornerPointsSharp->points.at(i).intensity)) / _scan_period_sec;
          }
          ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
          problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
          corner_correspondence++;
        }
      }

      // find correspondence for plane features
      for (int i = 0; i < surfPointsFlatNum; ++i) {
        TransformToStart(&(surfPointsFlat->points.at(i)), &pointSel);
        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
        if (pointSearchSqDis.at(0) < DISTANCE_SQ_THRESHOLD) {
          closestPointInd = pointSearchInd.at(0);

          // get closest point's scan ID
          int    closestPointScanID = int(laserCloudSurfLast->points.at(closestPointInd).intensity);
          double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

          // search in the direction of increasing scan line
          for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast->points.size(); ++j) {
            // if not in nearby scans, end the loop
            if (int(laserCloudSurfLast->points.at(j).intensity) > (closestPointScanID + NEARBY_SCAN))
              break;

            double pointSqDis = (laserCloudSurfLast->points.at(j).x - pointSel.x) * (laserCloudSurfLast->points.at(j).x - pointSel.x) +
                                (laserCloudSurfLast->points.at(j).y - pointSel.y) * (laserCloudSurfLast->points.at(j).y - pointSel.y) +
                                (laserCloudSurfLast->points.at(j).z - pointSel.z) * (laserCloudSurfLast->points.at(j).z - pointSel.z);

            // if in the same or lower scan line
            if (int(laserCloudSurfLast->points.at(j).intensity) <= closestPointScanID && pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2   = j;
            }
            // if in the higher scan line
            else if (int(laserCloudSurfLast->points.at(j).intensity) > closestPointScanID && pointSqDis < minPointSqDis3) {
              minPointSqDis3 = pointSqDis;
              minPointInd3   = j;
            }
          }

          // search in the direction of decreasing scan line
          for (int j = closestPointInd - 1; j >= 0; --j) {
            // if not in nearby scans, end the loop
            if (int(laserCloudSurfLast->points.at(j).intensity) < (closestPointScanID - NEARBY_SCAN))
              break;

            double pointSqDis = (laserCloudSurfLast->points.at(j).x - pointSel.x) * (laserCloudSurfLast->points.at(j).x - pointSel.x) +
                                (laserCloudSurfLast->points.at(j).y - pointSel.y) * (laserCloudSurfLast->points.at(j).y - pointSel.y) +
                                (laserCloudSurfLast->points.at(j).z - pointSel.z) * (laserCloudSurfLast->points.at(j).z - pointSel.z);

            // if in the same or higher scan line
            if (int(laserCloudSurfLast->points.at(j).intensity) >= closestPointScanID && pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2   = j;
            } else if (int(laserCloudSurfLast->points.at(j).intensity) < closestPointScanID && pointSqDis < minPointSqDis3) {
              // find nearer point
              minPointSqDis3 = pointSqDis;
              minPointInd3   = j;
            }
          }

          if (minPointInd2 >= 0 && minPointInd3 >= 0) {

            Eigen::Vector3d curr_point(surfPointsFlat->points.at(i).x, surfPointsFlat->points.at(i).y, surfPointsFlat->points.at(i).z);
            Eigen::Vector3d last_point_a(laserCloudSurfLast->points.at(closestPointInd).x, laserCloudSurfLast->points.at(closestPointInd).y,
                                         laserCloudSurfLast->points.at(closestPointInd).z);
            Eigen::Vector3d last_point_b(laserCloudSurfLast->points.at(minPointInd2).x, laserCloudSurfLast->points.at(minPointInd2).y,
                                         laserCloudSurfLast->points.at(minPointInd2).z);
            Eigen::Vector3d last_point_c(laserCloudSurfLast->points.at(minPointInd3).x, laserCloudSurfLast->points.at(minPointInd3).y,
                                         laserCloudSurfLast->points.at(minPointInd3).z);

            double s = 1.0;
            if (DISTORTION) {
              s = (surfPointsFlat->points.at(i).intensity - int(surfPointsFlat->points.at(i).intensity)) / _scan_period_sec;
            }
            ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
            plane_correspondence++;
          }
        }
      }

      // printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
      time_data_association = t_data.toc();

      if ((corner_correspondence + plane_correspondence) < 10) {
        ROS_WARN_STREAM("[AloamOdometry] low number of correspondence!");
      }

      TicToc                 t_solver;
      ceres::Solver::Options options;
      options.linear_solver_type           = ceres::DENSE_QR;
      options.max_num_iterations           = 4;
      options.minimizer_progress_to_stdout = false;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      /* printf("solver time %f ms \n", t_solver.toc()); */
      time_solver = t_solver.toc();
    }
    /* printf("optimization twice time %f \n", t_opt.toc()); */
    time_opt = t_opt.toc();

    t_w_curr = t_w_curr + q_w_curr * t_last_curr;
    q_w_curr = q_w_curr * q_last_curr;
  }

  /*//}*/

  TicToc t_pub;

  /*//{ Correct orientation using mavros*/

  geometry_msgs::Quaternion ori;
  bool                      got_orientation_meas = false;
  {
    std::scoped_lock lock(_mutex_orientation_meas);
    if (_got_orientation_meas && (_orientation_meas.header.stamp - stamp).toSec() < 0.5) {
      got_orientation_meas = true;
      ori                  = _orientation_meas.pose.pose.orientation;
    }
  }

  if (got_orientation_meas) {
    mrs_lib::AttitudeConverter attitude_mavros(ori);
    mrs_lib::AttitudeConverter attitude_aloam(q_w_curr);

    // Transform mavros attitude (fcu frame) to lidar frame
    tf::Vector3 rpy = tf::quatRotate(_tf_lidar_to_fcu.getRotation(), tf::Vector3(attitude_mavros.getRoll(), attitude_mavros.getPitch(), 0));
    ori             = tf::createQuaternionMsgFromRollPitchYaw(rpy.x(), rpy.y(), attitude_aloam.getHeading());

    /* ROS_WARN("Lidar orientation: (%0.2f, %0.2f, %0.2f), odom orientation: (%0.2f, %0.2f, %0.2f)", rpy.x(), rpy.y(), rpy.z(), attitude_aloam.getRoll(), */
    /* attitude_aloam.getPitch(), attitude_aloam.getHeading()); */
  } else {
    tf::quaternionEigenToMsg(q_w_curr, ori);
  }

  /*//}*/

  /*//{ Publish local odometry */

  // Publish odometry as PoseStamped
  nav_msgs::Odometry laserOdometry;
  laserOdometry.header.frame_id = _frame_map;
  laserOdometry.header.stamp    = stamp;
  laserOdometry.child_frame_id  = _frame_lidar;

  tf::pointEigenToMsg(t_w_curr, laserOdometry.pose.pose.position);
  laserOdometry.pose.pose.orientation = ori;
  _pub_odometry_local.publish(laserOdometry);

  pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
  cornerPointsLessSharp                          = laserCloudCornerLast;
  laserCloudCornerLast                           = laserCloudTemp;

  laserCloudTemp     = surfPointsLessFlat;
  surfPointsLessFlat = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudTemp;

  kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
  kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

  laserCloudCornerLast->header.stamp = laserCloudFullRes->header.stamp;
  laserCloudSurfLast->header.stamp   = laserCloudFullRes->header.stamp;
  laserCloudFullRes->header.stamp    = laserCloudFullRes->header.stamp;

  laserCloudCornerLast->header.frame_id = _frame_lidar;
  laserCloudSurfLast->header.frame_id   = _frame_lidar;
  laserCloudFullRes->header.frame_id    = _frame_lidar;

  /* printf("publication time %f ms \n", t_pub.toc()); */
  /* printf("whole laserOdometry time %f ms \n \n", t_whole.toc()); */

  float time_whole = t_whole.toc();
  ROS_INFO_THROTTLE(1.0, "[AloamOdometry] Run time: %0.1f ms (%0.1f Hz)", time_whole, std::min(1.0f / _scan_period_sec, 1000.0f / time_whole));
  ROS_DEBUG_THROTTLE(1.0,
                     "[AloamOdometry] feature registration: %0.1f ms; solver time: %0.1f ms; double optimization time: %0.1f ms; publishing time: %0.1f ms",
                     time_data_association, time_solver, time_opt, t_pub.toc());
  ROS_WARN_COND(time_whole > _scan_period_sec * 1000.0f, "[AloamOdometry] Odometry process took over %0.2f ms", _scan_period_sec * 1000.0f);

  _mapper->setData(laserOdometry, laserCloudCornerLast, laserCloudSurfLast, laserCloudFullRes);

  /*//}*/

  _frame_count++;
}
/*//}*/

/*//{ setData() */
void AloamOdometry::setData(pcl::PointCloud<PointType>::Ptr cornerPointsSharp, pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp,
                            pcl::PointCloud<PointType>::Ptr surfPointsFlat, pcl::PointCloud<PointType>::Ptr surfPointsLessFlat,
                            pcl::PointCloud<PointType>::Ptr laserCloudFullRes) {
  
  mrs_lib::Routine profiler_routine = _profiler->createRoutine("aloamOdometrySetData");

  std::scoped_lock lock(_mutex_features_data);
  _has_new_data          = true;
  _cornerPointsSharp     = cornerPointsSharp;
  _cornerPointsLessSharp = cornerPointsLessSharp;
  _surfPointsFlat        = surfPointsFlat;
  _surfPointsLessFlat    = surfPointsLessFlat;
  _laserCloudFullRes     = laserCloudFullRes;
}
/*//}*/

/*//{ TransformToStart() */
// undistort lidar point
void AloamOdometry::TransformToStart(PointType const *const pi, PointType *const po) {
  // interpolation ratio
  double s = 1.0;
  if (DISTORTION) {
    s = (pi->intensity - int(pi->intensity)) / _scan_period_sec;
  }
  Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
  Eigen::Vector3d    t_point_last = s * t_last_curr;
  Eigen::Vector3d    point(pi->x, pi->y, pi->z);
  Eigen::Vector3d    un_point = q_point_last * point + t_point_last;

  po->x         = un_point.x();
  po->y         = un_point.y();
  po->z         = un_point.z();
  po->intensity = pi->intensity;
}
/*//}*/

/*//{ TransformToEnd() */
// transform all lidar points to the start of the next frame
void AloamOdometry::TransformToEnd(PointType const *const pi, PointType *const po) {
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

/*//{ callbackOrientationMeas() */
void AloamOdometry::callbackOrientationMeas(const nav_msgs::OdometryConstPtr &msg) {
  if (!is_initialized) {
    return;
  }

  mrs_lib::Routine profiler_routine = _profiler->createRoutine("callbackOrientationMeas");

  ROS_INFO_ONCE("[AloamOdometry] Received first message with orientation measurement.");

  std::scoped_lock lock(_mutex_orientation_meas);
  _got_orientation_meas = true;
  _orientation_meas     = *msg;
}
/*//}*/

}  // namespace aloam_slam

