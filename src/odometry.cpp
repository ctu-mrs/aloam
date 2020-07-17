#include "aloam_slam/odometry.h"
#include "aloam_slam/mapping.h"

namespace aloam_slam
{

/* //{ AloamOdometry() */
AloamOdometry::AloamOdometry(const ros::NodeHandle &parent_nh, std::shared_ptr<mrs_lib::Profiler> profiler, std::shared_ptr<AloamMapping> aloam_mapping,
                             std::string frame_fcu, std::string frame_lidar, std::string frame_odom, float scan_period_sec, tf::Transform tf_lidar_to_fcu)
    : _profiler(profiler),
      _aloam_mapping(aloam_mapping),
      _frame_fcu(frame_fcu),
      _frame_lidar(frame_lidar),
      _frame_odom(frame_odom),
      _scan_period_sec(scan_period_sec),
      _tf_lidar_to_fcu(tf_lidar_to_fcu),
      _q_last_curr(_para_q),
      _t_last_curr(_para_t) {

  ros::NodeHandle nh_(parent_nh);

  // Objects initialization
  _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>();

  {
    std::scoped_lock lock(_mutex_odometry_process);
    _features_corners_last = boost::make_shared<pcl::PointCloud<PointType>>();
    _features_surfs_last   = boost::make_shared<pcl::PointCloud<PointType>>();
  }

  _q_w_curr = Eigen::Quaterniond(1, 0, 0, 0);  // eigen has qw, qx, qy, qz notation
  _t_w_curr = Eigen::Vector3d(0, 0, 0);


  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh         = nh_;
  shopts.node_name  = "AloamOdometry";
  shopts.threadsafe = true;

  /* _sub_handler_orientation = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "orientation_in", ros::Duration(0.5)); */

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
    std::scoped_lock lock(_mutex_extracted_features);
    has_new_data = _has_new_data;
  }

  if (!has_new_data) {
    return;
  }

  pcl::PointCloud<PointType>::Ptr corner_points_sharp;
  pcl::PointCloud<PointType>::Ptr corner_points_less_sharp;
  pcl::PointCloud<PointType>::Ptr surf_points_flat;
  pcl::PointCloud<PointType>::Ptr surf_points_less_flat;
  pcl::PointCloud<PointType>::Ptr laser_cloud_full_res;
  {
    std::scoped_lock lock(_mutex_extracted_features);
    _has_new_data            = false;
    corner_points_sharp      = _corner_points_sharp;
    corner_points_less_sharp = _corner_points_less_sharp;
    surf_points_flat         = _surf_points_flat;
    surf_points_less_flat    = _surf_points_less_flat;
    laser_cloud_full_res     = _cloud_full_ress;
  }
  /*//}*/

  mrs_lib::Routine profiler_routine = _profiler->createRoutine("timerOdometry", 1.0f / _scan_period_sec, 0.05, event);

  ros::Time stamp;
  pcl_conversions::fromPCL(laser_cloud_full_res->header.stamp, stamp);

  TicToc t_whole;

  /*//{ Find features correspondences and compute local odometry */

  float time_data_association = 0.0f;
  float time_solver           = 0.0f;
  float time_opt              = 0.0f;

  if (_frame_count > 0) {
    std::scoped_lock lock(_mutex_odometry_process);

    int cornerPointsSharpNum = corner_points_sharp->points.size();
    int surfPointsFlatNum    = surf_points_flat->points.size();

    pcl::KdTreeFLANN<pcl::PointXYZI> _kdtree_corners_last;
    pcl::KdTreeFLANN<pcl::PointXYZI> _kdtree_surfs_last;

    _kdtree_corners_last.setInputCloud(_features_corners_last);
    _kdtree_surfs_last.setInputCloud(_features_surfs_last);

    TicToc t_opt;
    for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {
      int corner_correspondence = 0;
      int plane_correspondence  = 0;

      // ceres::LossFunction *loss_function = NULL;
      ceres::LossFunction *         loss_function      = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
      ceres::Problem::Options       problem_options;

      ceres::Problem problem(problem_options);
      problem.AddParameterBlock(_para_q, 4, q_parameterization);
      problem.AddParameterBlock(_para_t, 3);

      pcl::PointXYZI     pointSel;
      std::vector<int>   pointSearchInd;
      std::vector<float> pointSearchSqDis;

      TicToc t_data;

      // find correspondence for corner features
      for (int i = 0; i < cornerPointsSharpNum; ++i) {
        TransformToStart(&(corner_points_sharp->points.at(i)), &pointSel);
        _kdtree_corners_last.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

        int closestPointInd = -1, minPointInd2 = -1;
        if (pointSearchSqDis.at(0) < DISTANCE_SQ_THRESHOLD) {
          closestPointInd        = pointSearchInd.at(0);
          int closestPointScanID = int(_features_corners_last->points.at(closestPointInd).intensity);

          double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
          // search in the direction of increasing scan line
          for (int j = closestPointInd + 1; j < (int)_features_corners_last->points.size(); ++j) {
            // if in the same scan line, continue
            if (int(_features_corners_last->points.at(j).intensity) <= closestPointScanID)
              continue;

            // if not in nearby scans, end the loop
            if (int(_features_corners_last->points.at(j).intensity) > (closestPointScanID + NEARBY_SCAN))
              break;

            double pointSqDis = (_features_corners_last->points.at(j).x - pointSel.x) * (_features_corners_last->points.at(j).x - pointSel.x) +
                                (_features_corners_last->points.at(j).y - pointSel.y) * (_features_corners_last->points.at(j).y - pointSel.y) +
                                (_features_corners_last->points.at(j).z - pointSel.z) * (_features_corners_last->points.at(j).z - pointSel.z);

            if (pointSqDis < minPointSqDis2) {
              // find nearer point
              minPointSqDis2 = pointSqDis;
              minPointInd2   = j;
            }
          }

          // search in the direction of decreasing scan line
          for (int j = closestPointInd - 1; j >= 0; --j) {
            // if in the same scan line, continue
            if (int(_features_corners_last->points.at(j).intensity) >= closestPointScanID) {
              continue;
            }

            // if not in nearby scans, end the loop
            if (int(_features_corners_last->points.at(j).intensity) < (closestPointScanID - NEARBY_SCAN)) {
              break;
            }

            double pointSqDis = (_features_corners_last->points.at(j).x - pointSel.x) * (_features_corners_last->points.at(j).x - pointSel.x) +
                                (_features_corners_last->points.at(j).y - pointSel.y) * (_features_corners_last->points.at(j).y - pointSel.y) +
                                (_features_corners_last->points.at(j).z - pointSel.z) * (_features_corners_last->points.at(j).z - pointSel.z);

            if (pointSqDis < minPointSqDis2) {
              // find nearer point
              minPointSqDis2 = pointSqDis;
              minPointInd2   = j;
            }
          }
        }
        if (minPointInd2 >= 0)  // both closestPointInd and minPointInd2 is valid
        {
          Eigen::Vector3d curr_point(corner_points_sharp->points.at(i).x, corner_points_sharp->points.at(i).y, corner_points_sharp->points.at(i).z);
          Eigen::Vector3d last_point_a(_features_corners_last->points.at(closestPointInd).x, _features_corners_last->points.at(closestPointInd).y,
                                       _features_corners_last->points.at(closestPointInd).z);
          Eigen::Vector3d last_point_b(_features_corners_last->points.at(minPointInd2).x, _features_corners_last->points.at(minPointInd2).y,
                                       _features_corners_last->points.at(minPointInd2).z);

          double s = 1.0;
          if (DISTORTION) {
            s = (corner_points_sharp->points.at(i).intensity - int(corner_points_sharp->points.at(i).intensity)) / _scan_period_sec;
          }
          ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
          problem.AddResidualBlock(cost_function, loss_function, _para_q, _para_t);
          corner_correspondence++;
        }
      }

      // find correspondence for plane features
      for (int i = 0; i < surfPointsFlatNum; ++i) {
        TransformToStart(&(surf_points_flat->points.at(i)), &pointSel);
        _kdtree_surfs_last.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
        if (pointSearchSqDis.at(0) < DISTANCE_SQ_THRESHOLD) {
          closestPointInd = pointSearchInd.at(0);

          // get closest point's scan ID
          int    closestPointScanID = int(_features_surfs_last->points.at(closestPointInd).intensity);
          double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

          // search in the direction of increasing scan line
          for (int j = closestPointInd + 1; j < (int)_features_surfs_last->points.size(); ++j) {
            // if not in nearby scans, end the loop
            if (int(_features_surfs_last->points.at(j).intensity) > (closestPointScanID + NEARBY_SCAN))
              break;

            double pointSqDis = (_features_surfs_last->points.at(j).x - pointSel.x) * (_features_surfs_last->points.at(j).x - pointSel.x) +
                                (_features_surfs_last->points.at(j).y - pointSel.y) * (_features_surfs_last->points.at(j).y - pointSel.y) +
                                (_features_surfs_last->points.at(j).z - pointSel.z) * (_features_surfs_last->points.at(j).z - pointSel.z);

            // if in the same or lower scan line
            if (int(_features_surfs_last->points.at(j).intensity) <= closestPointScanID && pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2   = j;
            }
            // if in the higher scan line
            else if (int(_features_surfs_last->points.at(j).intensity) > closestPointScanID && pointSqDis < minPointSqDis3) {
              minPointSqDis3 = pointSqDis;
              minPointInd3   = j;
            }
          }

          // search in the direction of decreasing scan line
          for (int j = closestPointInd - 1; j >= 0; --j) {
            // if not in nearby scans, end the loop
            if (int(_features_surfs_last->points.at(j).intensity) < (closestPointScanID - NEARBY_SCAN))
              break;

            double pointSqDis = (_features_surfs_last->points.at(j).x - pointSel.x) * (_features_surfs_last->points.at(j).x - pointSel.x) +
                                (_features_surfs_last->points.at(j).y - pointSel.y) * (_features_surfs_last->points.at(j).y - pointSel.y) +
                                (_features_surfs_last->points.at(j).z - pointSel.z) * (_features_surfs_last->points.at(j).z - pointSel.z);

            // if in the same or higher scan line
            if (int(_features_surfs_last->points.at(j).intensity) >= closestPointScanID && pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2   = j;
            } else if (int(_features_surfs_last->points.at(j).intensity) < closestPointScanID && pointSqDis < minPointSqDis3) {
              // find nearer point
              minPointSqDis3 = pointSqDis;
              minPointInd3   = j;
            }
          }

          if (minPointInd2 >= 0 && minPointInd3 >= 0) {

            Eigen::Vector3d curr_point(surf_points_flat->points.at(i).x, surf_points_flat->points.at(i).y, surf_points_flat->points.at(i).z);
            Eigen::Vector3d last_point_a(_features_surfs_last->points.at(closestPointInd).x, _features_surfs_last->points.at(closestPointInd).y,
                                         _features_surfs_last->points.at(closestPointInd).z);
            Eigen::Vector3d last_point_b(_features_surfs_last->points.at(minPointInd2).x, _features_surfs_last->points.at(minPointInd2).y,
                                         _features_surfs_last->points.at(minPointInd2).z);
            Eigen::Vector3d last_point_c(_features_surfs_last->points.at(minPointInd3).x, _features_surfs_last->points.at(minPointInd3).y,
                                         _features_surfs_last->points.at(minPointInd3).z);

            double s = 1.0;
            if (DISTORTION) {
              s = (surf_points_flat->points.at(i).intensity - int(surf_points_flat->points.at(i).intensity)) / _scan_period_sec;
            }
            ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
            problem.AddResidualBlock(cost_function, loss_function, _para_q, _para_t);
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

    _t_w_curr = _t_w_curr + _q_w_curr * _t_last_curr;
    _q_w_curr = _q_w_curr * _q_last_curr;
  }

  /*//}*/

  TicToc t_pub;

  geometry_msgs::Quaternion ori;
  tf::quaternionEigenToMsg(_q_w_curr, ori);

  /*//{ TODO: Correct orientation using inertial measurements */

  /* if (_sub_handler_orientation.hasMsg()) { */
  /*   auto odom_msg = _sub_handler_orientation.getMsg(); */

  /*   // Find transform from control frame to aloam map frame */
  /*   auto ret = _transformer->getTransform(_frame_map, odom_msg->header.frame_id, stamp); */

  /*   if (ret) { */
  /*     tf::Quaternion control_to_map_att; */
  /*     tf::Quaternion fcu_in_control_att; */

  /*     // Get rotation transform from control frame to the aloam map frame */
  /*     tf::quaternionMsgToTF(ret.value().getTransform().transform.rotation, control_to_map_att); */
  /*     /1* std::cout << "gps -> aloam: " << ret.value().getTransform().transform << std::endl; *1/ */
  /*     ROS_INFO("%0.2f %0.2f %0.2f %0.2f", control_to_map_att.x(), control_to_map_att.y(), control_to_map_att.z(), control_to_map_att.w()); */

  /*     // Get attitude given by orientation from odometry */
  /*     tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, fcu_in_control_att); */

  /*     // Transform the odometry attitude to aloam map frame */
  /*     tf::Quaternion fcu_in_map_att = control_to_map_att * fcu_in_control_att; */

  /*     // Transform measured fcu odom to lidar odom */
  /*     mrs_lib::AttitudeConverter lidar_in_map_meas_att = fcu_in_map_att * _tf_lidar_to_fcu.getRotation(); */
  /*     // Set measured lidar odom roll/pitch to the estimated aloam attitude */
  /*     mrs_lib::AttitudeConverter lidar_in_map_esti_att = _q_w_curr; */
  /*     lidar_in_map_meas_att.setHeading(lidar_in_map_esti_att.getHeading()); */

  /*     tf::quaternionTFToMsg(lidar_in_map_meas_att, ori); */

  /*     mrs_lib::AttitudeConverter ac_fcu_in_map     = fcu_in_map_att; */
  /*     mrs_lib::AttitudeConverter ac_control_to_map = control_to_map_att; */
  /*     mrs_lib::AttitudeConverter ac_fcu_in_control = fcu_in_control_att; */
  /*     ROS_INFO("control to map:   %0.2f %0.2f %0.2f", ac_control_to_map.getRoll(), ac_control_to_map.getPitch(), ac_control_to_map.getHeading()); */
  /*     ROS_INFO("fcu in control:   %0.2f %0.2f %0.2f", ac_fcu_in_control.getRoll(), ac_fcu_in_control.getPitch(), ac_fcu_in_control.getHeading()); */
  /*     ROS_INFO("fcu in map:       %0.2f %0.2f %0.2f", ac_fcu_in_map.getRoll(), ac_fcu_in_map.getPitch(), ac_fcu_in_map.getHeading()); */
  /*     ROS_INFO("measured  in map: %0.2f %0.2f %0.2f", lidar_in_map_meas_att.getRoll(), lidar_in_map_meas_att.getPitch(), lidar_in_map_meas_att.getHeading());
   */
  /*     ROS_INFO("estimated in map: %0.2f %0.2f %0.2f", lidar_in_map_esti_att.getRoll(), lidar_in_map_esti_att.getPitch(), lidar_in_map_esti_att.getHeading());
   */
  /*   } */
  /* } */

  /*//}*/

  // Transform odometry from lidar frame to fcu frame
  tf::Transform tf_lidar;
  tf_lidar.setOrigin(tf::Vector3(_t_w_curr.x(), _t_w_curr.y(), _t_w_curr.z()));
  tf::Quaternion tf_q;
  tf::quaternionMsgToTF(ori, tf_q);
  tf_lidar.setRotation(tf_q);

  /*//{ Save odometry data to AloamMapping */
  {
    std::scoped_lock                lock(_mutex_odometry_process);
    pcl::PointCloud<PointType>::Ptr laserCloudTemp = corner_points_less_sharp;
    corner_points_less_sharp                       = _features_corners_last;
    _features_corners_last                         = laserCloudTemp;

    laserCloudTemp        = surf_points_less_flat;
    surf_points_less_flat = _features_surfs_last;
    _features_surfs_last  = laserCloudTemp;

    _features_corners_last->header.stamp = laser_cloud_full_res->header.stamp;
    _features_surfs_last->header.stamp   = laser_cloud_full_res->header.stamp;
    laser_cloud_full_res->header.stamp   = laser_cloud_full_res->header.stamp;

    _features_corners_last->header.frame_id = _frame_lidar;
    _features_surfs_last->header.frame_id   = _frame_lidar;
    laser_cloud_full_res->header.frame_id   = _frame_lidar;

    _aloam_mapping->setData(stamp, tf_lidar, _features_corners_last, _features_surfs_last, laser_cloud_full_res);
  }
  /*//}*/

  /*//{ Publish odometry */

  if (_frame_count > 0) {
    // Publish inverse TF transform (lidar -> odom)
    tf::Transform tf_fcu = tf_lidar * _tf_lidar_to_fcu;

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp    = stamp;
    tf_msg.header.frame_id = _frame_fcu;
    tf_msg.child_frame_id  = _frame_odom;
    tf::transformTFToMsg(tf_fcu.inverse(), tf_msg.transform);

    try {
      _tf_broadcaster->sendTransform(tf_msg);
    }
    catch (...) {
      ROS_ERROR("[AloamOdometry]: Exception caught during publishing TF: %s - %s.", tf_msg.child_frame_id.c_str(), tf_msg.header.frame_id.c_str());
    }

    // Publish nav_msgs::Odometry msg in odom frame
    if (_pub_odometry_local.getNumSubscribers() > 0) {
      // Publish nav_msgs::Odometry msg
      nav_msgs::Odometry::Ptr laser_odometry_msg = boost::make_shared<nav_msgs::Odometry>();
      laser_odometry_msg->header.stamp           = stamp;
      laser_odometry_msg->header.frame_id        = _frame_odom;
      laser_odometry_msg->child_frame_id         = _frame_fcu;
      tf::pointTFToMsg(tf_fcu.getOrigin(), laser_odometry_msg->pose.pose.position);
      tf::quaternionTFToMsg(tf_fcu.getRotation(), laser_odometry_msg->pose.pose.orientation);

      try {
        _pub_odometry_local.publish(laser_odometry_msg);
      }
      catch (...) {
        ROS_ERROR("[AloamOdometry]: Exception caught during publishing topic %s.", _pub_odometry_local.getTopic().c_str());
      }
    }
  }

  /*//}*/

  _frame_count++;

  // Print diagnostics
  float time_whole = t_whole.toc();
  ROS_INFO_THROTTLE(1.0, "[AloamOdometry] Run time: %0.1f ms (%0.1f Hz)", time_whole, std::min(1.0f / _scan_period_sec, 1000.0f / time_whole));
  ROS_DEBUG_THROTTLE(1.0,
                     "[AloamOdometry] feature registration: %0.1f ms; solver time: %0.1f ms; double optimization time: %0.1f ms; publishing time: %0.1f ms",
                     time_data_association, time_solver, time_opt, t_pub.toc());
  ROS_WARN_COND(time_whole > _scan_period_sec * 1000.0f, "[AloamOdometry] Odometry process took over %0.2f ms", _scan_period_sec * 1000.0f);
}
/*//}*/

/*//{ setData() */
void AloamOdometry::setData(pcl::PointCloud<PointType>::Ptr corner_points_sharp, pcl::PointCloud<PointType>::Ptr corner_points_less_sharp,
                            pcl::PointCloud<PointType>::Ptr surf_points_flat, pcl::PointCloud<PointType>::Ptr surf_points_less_flat,
                            pcl::PointCloud<PointType>::Ptr laser_cloud_full_res) {

  mrs_lib::Routine profiler_routine = _profiler->createRoutine("aloamOdometrySetData");

  std::scoped_lock lock(_mutex_extracted_features);
  _has_new_data             = true;
  _corner_points_sharp      = corner_points_sharp;
  _corner_points_less_sharp = corner_points_less_sharp;
  _surf_points_flat         = surf_points_flat;
  _surf_points_less_flat    = surf_points_less_flat;
  _cloud_full_ress          = laser_cloud_full_res;
}
/*//}*/

/*//{ TransformToStart() */
void AloamOdometry::TransformToStart(PointType const *const pi, PointType *const po) {
  // interpolation ratio
  double s = 1.0;
  if (DISTORTION) {
    s = (pi->intensity - int(pi->intensity)) / _scan_period_sec;
  }
  Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, _q_last_curr);
  Eigen::Vector3d    t_point_last = s * _t_last_curr;
  Eigen::Vector3d    point(pi->x, pi->y, pi->z);
  Eigen::Vector3d    un_point = q_point_last * point + t_point_last;

  po->x         = un_point.x();
  po->y         = un_point.y();
  po->z         = un_point.z();
  po->intensity = pi->intensity;
}
/*//}*/

}  // namespace aloam_slam

