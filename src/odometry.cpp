#include "aloam_slam/odometry.h"
#include "aloam_slam/mapping.h"

namespace aloam_slam
{

/* //{ class AloamOdometry */
AloamOdometry::AloamOdometry(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::shared_ptr<AloamMapping> mapper, std::string frame_lidar,
                             std::string frame_map, float scan_period_sec, tf::Transform tf_lidar_to_fcu)
    : _mapper(mapper),
      _frame_lidar(frame_lidar),
      _frame_map(frame_map),
      _scan_period_sec(scan_period_sec),
      _tf_lidar_to_fcu(tf_lidar_to_fcu),
      q_last_curr(para_q),
      t_last_curr(para_t) {

  ros::NodeHandle nh_(parent_nh);

  param_loader.loadParam("mapping_skip_frame", _skip_mapping_frame_num, 1);

  // Objects initialization
  laserCloudCornerLast = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  laserCloudSurfLast   = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
  kdtreeCornerLast     = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  kdtreeSurfLast       = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>);

  q_w_curr = Eigen::Quaterniond(1, 0, 0, 0);
  t_w_curr = Eigen::Vector3d(0, 0, 0);

  /* q_last_curr = Eigen::Map<Eigen::Quaterniond>(para_q); */
  /* t_last_curr = Eigen::Map<Eigen::Vector3d>(para_t); */

  _sub_mavros_odom = nh_.subscribe("mavros_odom_in", 1, &AloamOdometry::callbackMavrosOdom, this, ros::TransportHints().tcpNoDelay());

  _pub_odometry_local = nh_.advertise<nav_msgs::Odometry>("odom_local_out", 1);

  /* TODO: print ROS_INFO("[AloamOdometry] Mapping at %0.2f Hz.", 1.0 / _scan_period_sec / skipFrameNum); */
}
//}

/*//{ TODO: compute_local_odometry() */
void AloamOdometry::compute_local_odometry(pcl::PointCloud<PointType>::Ptr cornerPointsSharp, pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp,
                                           pcl::PointCloud<PointType>::Ptr surfPointsFlat, pcl::PointCloud<PointType>::Ptr surfPointsLessFlat,
                                           pcl::PointCloud<PointType>::Ptr laserCloudFullRes) {

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
    ROS_ERROR_STREAM("[AloamOdometry] unsync message!");
    ROS_BREAK();
  }

  TicToc t_whole;
  // initializing
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
            if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID) {
              continue;
            }

            // if not in nearby scans, end the loop
            if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN)) {
              break;
            }

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

          double s = 1.0;
          if (DISTORTION) {
            s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / _scan_period_sec;
          }
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

            double s = 1.0;
            if (DISTORTION) {
              s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / _scan_period_sec;
            }
            ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
            plane_correspondence++;
          }
        }
      }

      // printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
      ROS_INFO_STREAM_THROTTLE(1.0, "[AloamOdometry] data association time " << t_data.toc() << " ms");

      if ((corner_correspondence + plane_correspondence) < 10) {
        ROS_INFO_STREAM("[AloamOdometry] less correspondence! *************************************************");
      }

      TicToc                 t_solver;
      ceres::Solver::Options options;
      options.linear_solver_type           = ceres::DENSE_QR;
      options.max_num_iterations           = 4;
      options.minimizer_progress_to_stdout = false;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      /* printf("solver time %f ms \n", t_solver.toc()); */
      ROS_INFO_STREAM_THROTTLE(1.0, "[AloamOdometry] solver time " << t_solver.toc() << " ms");
    }
    /* printf("optimization twice time %f \n", t_opt.toc()); */
    ROS_INFO_STREAM_THROTTLE(1.0, "[AloamOdometry] optimization twice time " << t_opt.toc() << " ms");

    t_w_curr = t_w_curr + q_w_curr * t_last_curr;
    q_w_curr = q_w_curr * q_last_curr;
  }

  TicToc t_pub;

  geometry_msgs::Quaternion ori;
  bool                      got_mavros = false;
  {
    std::scoped_lock lock(_mutex_odom_mavros);
    if (_got_mavros_odom && (_mavros_odom.header.stamp - stamp).toSec() < 0.5) {
      got_mavros = true;
      ori        = _mavros_odom.pose.pose.orientation;
    }
  }

  ROS_WARN("A");
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
    tf::Vector3 rpy = tf::quatRotate(_tf_lidar_to_fcu.getRotation(), tf::Vector3(roll, pitch, 0));

    /* ROS_WARN("Lidar orientation: (%0.2f, %0.2f, %0.2f), odom orientation: (%0.2f, %0.2f, %0.2f)", rpy.x(), rpy.y(), rpy.z(), r, p, yaw); */
    ori = tf::createQuaternionMsgFromRollPitchYaw(rpy.x(), rpy.y(), yaw);
  } else {
    tf::quaternionEigenToMsg(q_w_curr, ori);
  }
  ROS_WARN("B");

  // Publish odometry as PoseStamped
  nav_msgs::Odometry laserOdometry;
  laserOdometry.header.frame_id = _frame_map;
  laserOdometry.header.stamp    = stamp;
  laserOdometry.child_frame_id  = _frame_lidar;

  tf::pointEigenToMsg(t_w_curr, laserOdometry.pose.pose.position);
  laserOdometry.pose.pose.orientation = ori;
  _pub_odometry_local.publish(laserOdometry);

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

  kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
  kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

  if (_frame_count % _skip_mapping_frame_num == 0) {

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
    ROS_INFO_STREAM_THROTTLE(1.0, "[AloamOdometry] publication time " << t_pub.toc() << " ms");
    ROS_INFO_STREAM_THROTTLE(1.0, "[AloamOdometry] whole laserOdometry time " << t_whole.toc() << " ms");
    if (t_whole.toc() > _scan_period_sec * 1000) {
      ROS_WARN("[AloamOdometry] odometry process over %0.2f ms", _scan_period_sec * 1000);
    }

    _mapper->compute_mapping(laserOdometry, laserCloudCornerLast, laserCloudSurfLast, laserCloudFullRes);
  }

  _frame_count++;
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

/*//{ callbackMavrosOdom() */
void AloamOdometry::callbackMavrosOdom(const nav_msgs::OdometryConstPtr &msg) {
  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(_mutex_odom_mavros);
  _got_mavros_odom = true;
  _mavros_odom     = *msg;
}
/*//}*/

}  // namespace aloam_slam

