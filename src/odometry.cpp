#include "aloam_slam/odometry.h"

namespace aloam_slam
{

/* //{ AloamOdometry() */
template <class T_pt>
AloamOdometry<T_pt>::AloamOdometry(const std::shared_ptr<CommonHandlers_t> handlers, const std::shared_ptr<AloamMapping<T_pt>> aloam_mapping)
    : _q_last_curr(_para_q), _t_last_curr(_para_t) {

  _handlers        = handlers;
  _aloam_mapping   = aloam_mapping;
  _scan_period_sec = 1.0f / handlers->frequency;

  ros::NodeHandle nh_(handlers->nh);

  ros::Time::waitForValid();

  // Objects initialization
  _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>();

  {
    std::scoped_lock lock(_mutex_odometry_process);
    _features_corners_last = boost::make_shared<pcl::PointCloud<pt_I_t>>();
    _features_surfs_last   = boost::make_shared<pcl::PointCloud<pt_I_t>>();
  }

  _q_w_curr = Eigen::Quaterniond::Identity();  // eigen has qw, qx, qy, qz notation
  _t_w_curr = Eigen::Vector3d(0, 0, 0);

  _transformer = std::make_shared<mrs_lib::Transformer>("AloamOdometry");
  _transformer->setDefaultPrefix(_handlers->uav_name);

  /* _sub_handler_orientation = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "orientation_in", mrs_lib::no_timeout); */

  _pub_odometry_local = nh_.advertise<nav_msgs::Odometry>("odom_local_out", 1);

  if (!handlers->offline_run) {
    _timer_odometry_loop = nh_.createTimer(ros::Rate(_handlers->frequency), &AloamOdometry::timerOdometry, this, false, true);
  }
}
//}

/*//{ computeOdometry() */
template <class T_pt>
bool AloamOdometry<T_pt>::computeOdometry(geometry_msgs::TransformStamped &tf_msg_out) {

  if (!is_initialized) {
    ROS_WARN_THROTTLE(1.0, "[AloamOdometry] Calling uninitialized object.");
    return false;
  }

  std::unique_ptr<mrs_lib::ScopeTimer> timer;
  pcl::PointCloud<pt_I_t>::Ptr         corner_points_sharp;
  pcl::PointCloud<pt_I_t>::Ptr         surf_points_flat;
  unsigned int                         proc_points_count;

  feature_selection::FSCloudManagerPtr<T_pt> manager_corners_extracted;
  feature_selection::FSCloudManagerPtr<T_pt> manager_surfs_extracted;

  ros::Time                         stamp;
  std::uint64_t                     stamp_pcl;
  feature_extraction::FEDiagnostics diagnostics_fe;

  /*//{ Load latest features */
  {
    std::unique_lock lock(_mutex_odometry_data);

    bool has_new_data = false;

    if (_handlers->offline_run) {

      has_new_data = _has_new_data;

    } else {

      const std::chrono::duration<float> timeout(_scan_period_sec);

      if (_cv_odometry_data.wait_for(lock, timeout) == std::cv_status::no_timeout) {
        has_new_data = _has_new_data;
      }
    }

    if (!has_new_data) {
      ROS_WARN_THROTTLE(1.0, "[AloamOdometry] Waiting for data.");
      return false;
    }

    timer = std::make_unique<mrs_lib::ScopeTimer>("ALOAM::AloamOdometry::timerOdometry", _handlers->scope_timer_logger, _handlers->enable_scope_timer);

    proc_points_count = _odometry_data->finite_points_count;
    stamp             = _odometry_data->stamp_ros;
    stamp_pcl         = _odometry_data->stamp_pcl;

    manager_corners_extracted = _odometry_data->manager_corners_extracted;
    manager_surfs_extracted   = _odometry_data->manager_surfs_extracted;

    // We have to convert because we use the intensity field in searching for associations
    corner_points_sharp = cloudPointTtoCloudIPoint(_odometry_data->manager_corners_salient->getCloudPtr(),
                                                   _odometry_data->manager_corners_salient->getIndicesPtr(), _scan_period_sec);
    surf_points_flat    = cloudPointTtoCloudIPoint(_odometry_data->manager_surfs_salient->getCloudPtr(), _odometry_data->manager_surfs_salient->getIndicesPtr(),
                                                   _scan_period_sec);

    diagnostics_fe = _odometry_data->diagnostics_fe;

    _has_new_data = false;
  }

  /*//}*/

  if (proc_points_count == 0) {
    ROS_WARN_THROTTLE(1.0, "[AloamOdometry]: Received an empty input cloud, skipping!");
    return false;
  }

  const std::shared_ptr<MappingData<T_pt>> mapping_data = std::make_shared<MappingData<T_pt>>();

  timer->checkpoint("loading data");
  mapping_data->diagnostics_odometry.ms_loading_data = timer->getLifetime();

  mrs_lib::Routine profiler_routine = _handlers->profiler->createRoutine("timerOdometry");

  /*//{ Find features correspondences and compute local odometry */

  if (_frame_count > 0) {
    std::scoped_lock lock(_mutex_odometry_process);

    pcl::KdTreeFLANN<pt_I_t> _kdtree_corners_last;
    pcl::KdTreeFLANN<pt_I_t> _kdtree_surfs_last;

    _kdtree_corners_last.setInputCloud(_features_corners_last);
    _kdtree_surfs_last.setInputCloud(_features_surfs_last);

    for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {
      int corner_correspondence = 0;
      int plane_correspondence  = 0;

      // ceres::LossFunction *loss_function = NULL;
      ceres::LossFunction          *loss_function      = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
      ceres::Problem::Options       problem_options;

      ceres::Problem problem(problem_options);
      problem.AddParameterBlock(_para_q, 4, q_parameterization);
      problem.AddParameterBlock(_para_t, 3);

      std::vector<int>   pointSearchInd;
      std::vector<float> pointSearchSqDis;

      // find correspondence for corner features
      for (const auto &point_ori : corner_points_sharp->points) {

        const auto pointSel = TransformToStart(point_ori);
        _kdtree_corners_last.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

        int closestPointInd = -1, minPointInd2 = -1;
        if (pointSearchSqDis.at(0) < DISTANCE_SQ_THRESHOLD) {
          closestPointInd              = pointSearchInd.at(0);
          const int closestPointScanID = int(_features_corners_last->points.at(closestPointInd).intensity);

          double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
          // search in the direction of increasing scan line
          for (int j = closestPointInd + 1; j < (int)_features_corners_last->points.size(); ++j) {

            const auto &point_j = _features_surfs_last->points.at(j);

            // if in the same scan line, continue
            if (int(point_j.intensity) <= closestPointScanID) {
              continue;
            }

            // if not in nearby scans, end the loop
            if (int(point_j.intensity) > (closestPointScanID + NEARBY_SCAN)) {
              break;
            }

            const double pointSqDis = std::pow(point_j.x - pointSel.x, 2.0) + std::pow(point_j.y - pointSel.y, 2.0) + std::pow(point_j.z - pointSel.z, 2.0);

            if (pointSqDis < minPointSqDis2) {
              // find nearer point
              minPointSqDis2 = pointSqDis;
              minPointInd2   = j;
            }
          }

          // search in the direction of decreasing scan line
          for (int j = closestPointInd - 1; j >= 0; --j) {

            const auto &point_j = _features_surfs_last->points.at(j);

            // if in the same scan line, continue
            if (int(point_j.intensity) >= closestPointScanID) {
              continue;
            }

            // if not in nearby scans, end the loop
            if (int(point_j.intensity) < (closestPointScanID - NEARBY_SCAN)) {
              break;
            }

            const double pointSqDis = std::pow(point_j.x - pointSel.x, 2.0) + std::pow(point_j.y - pointSel.y, 2.0) + std::pow(point_j.z - pointSel.z, 2.0);

            if (pointSqDis < minPointSqDis2) {
              // find nearer point
              minPointSqDis2 = pointSqDis;
              minPointInd2   = j;
            }
          }
        }
        if (minPointInd2 >= 0)  // both closestPointInd and minPointInd2 is valid
        {
          const Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
          const Eigen::Vector3d last_point_a(_features_corners_last->points.at(closestPointInd).x, _features_corners_last->points.at(closestPointInd).y,
                                             _features_corners_last->points.at(closestPointInd).z);
          const Eigen::Vector3d last_point_b(_features_corners_last->points.at(minPointInd2).x, _features_corners_last->points.at(minPointInd2).y,
                                             _features_corners_last->points.at(minPointInd2).z);

          double s = 1.0;
          if (DISTORTION) {
            s = (point_ori.intensity - int(point_ori.intensity)) / _scan_period_sec;
          }
          LidarEdgeFactor     *factor        = new LidarEdgeFactor(curr_point, last_point_a, last_point_b, s);
          ceres::CostFunction *cost_function = LidarEdgeFactor::Create(factor);
          problem.AddResidualBlock(cost_function, loss_function, _para_q, _para_t);
          corner_correspondence++;
        }
      }

      // find correspondence for plane features
      for (const auto &point_ori : surf_points_flat->points) {
        const auto pointSel = TransformToStart(point_ori);
        _kdtree_surfs_last.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
        if (pointSearchSqDis.at(0) < DISTANCE_SQ_THRESHOLD) {
          closestPointInd = pointSearchInd.at(0);

          // get closest point's scan ID
          const int closestPointScanID = int(_features_surfs_last->points.at(closestPointInd).intensity);
          double    minPointSqDis2     = DISTANCE_SQ_THRESHOLD;
          double    minPointSqDis3     = DISTANCE_SQ_THRESHOLD;

          // search in the direction of increasing scan line
          for (int j = closestPointInd + 1; j < (int)_features_surfs_last->points.size(); ++j) {

            const auto &point_j = _features_surfs_last->points.at(j);

            // if not in nearby scans, end the loop
            if (int(point_j.intensity) > (closestPointScanID + NEARBY_SCAN)) {
              break;
            }

            const double pointSqDis = std::pow(point_j.x - pointSel.x, 2.0) + std::pow(point_j.y - pointSel.y, 2.0) + std::pow(point_j.z - pointSel.z, 2.0);

            // if in the same or lower scan line
            if (int(point_j.intensity) <= closestPointScanID && pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2   = j;
            }
            // if in the higher scan line
            else if (int(point_j.intensity) > closestPointScanID && pointSqDis < minPointSqDis3) {
              minPointSqDis3 = pointSqDis;
              minPointInd3   = j;
            }
          }

          // search in the direction of decreasing scan line
          for (int j = closestPointInd - 1; j >= 0; --j) {

            const auto &point_j = _features_surfs_last->points.at(j);

            // if not in nearby scans, end the loop
            if (int(point_j.intensity) < (closestPointScanID - NEARBY_SCAN)) {
              break;
            }

            const double pointSqDis = std::pow(point_j.x - pointSel.x, 2.0) + std::pow(point_j.y - pointSel.y, 2.0) + std::pow(point_j.z - pointSel.z, 2.0);

            // if in the same or higher scan line
            if (int(point_j.intensity) >= closestPointScanID && pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2   = j;
            } else if (int(point_j.intensity) < closestPointScanID && pointSqDis < minPointSqDis3) {
              // find nearer point
              minPointSqDis3 = pointSqDis;
              minPointInd3   = j;
            }
          }

          if (minPointInd2 >= 0 && minPointInd3 >= 0) {

            const auto &fp_a = _features_surfs_last->points.at(closestPointInd);
            const auto &fp_b = _features_surfs_last->points.at(minPointInd2);
            const auto &fp_c = _features_surfs_last->points.at(minPointInd3);

            const Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
            const Eigen::Vector3d last_point_a(fp_a.x, fp_a.y, fp_a.z);
            const Eigen::Vector3d last_point_b(fp_b.x, fp_b.y, fp_b.z);
            const Eigen::Vector3d last_point_c(fp_c.x, fp_c.y, fp_c.z);

            double s = 1.0;
            if (DISTORTION) {
              s = (point_ori.intensity - int(point_ori.intensity)) / _scan_period_sec;
            }
            LidarPlaneNormFactorFromPoints *factor        = new LidarPlaneNormFactorFromPoints(curr_point, last_point_a, last_point_b, last_point_c, s);
            ceres::CostFunction            *cost_function = LidarPlaneNormFactorFromPoints::Create(factor);
            problem.AddResidualBlock(cost_function, loss_function, _para_q, _para_t);
            plane_correspondence++;
          }
        }
      }

      if ((corner_correspondence + plane_correspondence) < 10) {
        ROS_WARN_STREAM("[AloamOdometry] low number of correspondence!");
      }

      ceres::Solver::Options options;
      options.linear_solver_type           = ceres::DENSE_QR;
      options.max_num_iterations           = 4;
      options.minimizer_progress_to_stdout = false;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      /* printf("solver time %f ms \n", t_solver.toc()); */
    }

    _t_w_curr = _t_w_curr + _q_w_curr * _t_last_curr;
    _q_w_curr = _q_w_curr * _q_last_curr;
  }

  /*//}*/

  timer->checkpoint("computing local odometry");
  mapping_data->diagnostics_odometry.ms_optimization = timer->getLifetime();

  geometry_msgs::Quaternion ori;
  tf::quaternionEigenToMsg(_q_w_curr, ori);

  // Transform odometry from lidar frame to fcu frame
  tf::Transform tf_lidar;
  tf_lidar.setOrigin(tf::Vector3(_t_w_curr.x(), _t_w_curr.y(), _t_w_curr.z()));
  tf::Quaternion tf_q;
  tf::quaternionMsgToTF(ori, tf_q);
  tf_lidar.setRotation(tf_q);

  /*//{ Save odometry data to AloamMapping */
  {
    std::scoped_lock lock(_mutex_odometry_process);
    _features_corners_last = cloudPointTtoCloudIPoint(manager_corners_extracted->getCloudPtr(), manager_corners_extracted->getIndicesPtr(), _scan_period_sec);
    _features_surfs_last   = cloudPointTtoCloudIPoint(manager_surfs_extracted->getCloudPtr(), manager_surfs_extracted->getIndicesPtr(), _scan_period_sec);

    _features_corners_last->header.stamp = stamp_pcl;
    _features_surfs_last->header.stamp   = stamp_pcl;

    _features_corners_last->header.frame_id = _handlers->frame_lidar;
    _features_surfs_last->header.frame_id   = _handlers->frame_lidar;

    mapping_data->stamp_ros       = stamp;
    mapping_data->odometry        = tf_lidar;
    mapping_data->manager_corners = manager_corners_extracted;
    mapping_data->manager_surfs   = manager_surfs_extracted;

    mapping_data->diagnostics_fe                = diagnostics_fe;
    mapping_data->diagnostics_odometry.ms_total = timer->getLifetime();

    _aloam_mapping->setData(mapping_data);
  }
  /*//}*/

  /*//{ Publish odometry */

  if (_frame_count > 0) {
    // Publish inverse TF transform (lidar -> odom)
    const tf::Transform tf_fcu = tf_lidar * _handlers->tf_lidar_in_fcu_frame;

    tf_msg_out.header.stamp    = stamp;
    tf_msg_out.header.frame_id = _handlers->frame_fcu;
    tf_msg_out.child_frame_id  = _handlers->frame_odom;
    tf::transformTFToMsg(tf_fcu.inverse(), tf_msg_out.transform);

    try {
      _tf_broadcaster->sendTransform(tf_msg_out);
    }
    catch (...) {
      ROS_ERROR("[AloamOdometry]: Exception caught during publishing TF: %s - %s.", tf_msg_out.child_frame_id.c_str(), tf_msg_out.header.frame_id.c_str());
    }

    // Publish nav_msgs::Odometry msg in odom frame
    if (_pub_odometry_local.getNumSubscribers() > 0) {
      // Publish nav_msgs::Odometry msg
      const nav_msgs::Odometry::Ptr laser_odometry_msg = boost::make_shared<nav_msgs::Odometry>();
      laser_odometry_msg->header.stamp                 = stamp;
      laser_odometry_msg->header.frame_id              = _handlers->frame_odom;
      laser_odometry_msg->child_frame_id               = _handlers->frame_fcu;
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
  /* ROS_INFO_THROTTLE(1.0, "[AloamOdometry] Run time: %0.1f ms (%0.1f Hz)", time_whole, std::min(1.0f / _scan_period_sec, 1000.0f / time_whole)); */
  /* ROS_DEBUG_THROTTLE(1.0, */
  /*                    "[AloamOdometry] feature registration: %0.1f ms; solver time: %0.1f ms; double optimization time: %0.1f ms; publishing time: %0.1f ms",
   */
  /*                    time_data_association, time_solver, time_opt, t_pub.toc()); */
  /* ROS_WARN_COND(time_whole > _scan_period_sec * 1000.0f, "[AloamOdometry] Odometry process took over %0.2f ms", _scan_period_sec * 1000.0f); */

  return true;
}
/*//}*/

/*//{ timerOdometry() */
template <class T_pt>
void AloamOdometry<T_pt>::timerOdometry([[maybe_unused]] const ros::TimerEvent &event) {
  geometry_msgs::TransformStamped tf_msg;
  computeOdometry(tf_msg);
}
/*//}*/

/*//{ setData() */
template <class T_pt>
void AloamOdometry<T_pt>::setData(const std::shared_ptr<OdometryData<T_pt>> data) {

  mrs_lib::Routine profiler_routine = _handlers->profiler->createRoutine("aloamOdometrySetData");

  {
    std::unique_lock lock(_mutex_odometry_data);
    _has_new_data  = true;
    _odometry_data = data;
  }

  _cv_odometry_data.notify_all();
}
/*//}*/

/* setTransform() //{ */

template <class T_pt>
void AloamOdometry<T_pt>::setTransform(const Eigen::Vector3d &t, const Eigen::Quaterniond &q, const ros::Time &stamp) {
  _q_w_curr = q;
  _t_w_curr = t;

  // Transform odometry from lidar frame to fcu frame
  tf::Transform tf_lidar;
  tf_lidar.setOrigin(tf::Vector3(_t_w_curr.x(), _t_w_curr.y(), _t_w_curr.z()));
  geometry_msgs::Quaternion ori;
  tf::quaternionEigenToMsg(_q_w_curr, ori);
  tf::Quaternion tf_q;
  tf::quaternionMsgToTF(ori, tf_q);
  tf_lidar.setRotation(tf_q);

  /*//{ Publish odometry */

  // Publish inverse TF transform (lidar -> odom)
  tf::Transform tf_fcu = tf_lidar * _handlers->tf_lidar_in_fcu_frame;

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp    = stamp;
  tf_msg.header.frame_id = _handlers->frame_fcu;
  tf_msg.child_frame_id  = _handlers->frame_odom;
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
    laser_odometry_msg->header.frame_id        = _handlers->frame_odom;
    laser_odometry_msg->child_frame_id         = _handlers->frame_fcu;
    tf::pointTFToMsg(tf_fcu.getOrigin(), laser_odometry_msg->pose.pose.position);
    tf::quaternionTFToMsg(tf_fcu.getRotation(), laser_odometry_msg->pose.pose.orientation);

    try {
      _pub_odometry_local.publish(laser_odometry_msg);
    }
    catch (...) {
      ROS_ERROR("[AloamOdometry]: Exception caught during publishing topic %s.", _pub_odometry_local.getTopic().c_str());
    }
  }

  /*//}*/
}

//}

/*//{ TransformToStart() */
template <class T_pt>
pt_I_t AloamOdometry<T_pt>::TransformToStart(const pt_I_t &pi) {
  // interpolation ratio
  double s = 1.0;
  if (DISTORTION) {
    s = (pi.intensity - int(pi.intensity)) / _scan_period_sec;
  }

  const Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, _q_last_curr);
  const Eigen::Vector3d    t_point_last = s * _t_last_curr;
  const Eigen::Vector3d    point(pi.x, pi.y, pi.z);
  const Eigen::Vector3d    un_point = q_point_last * point + t_point_last;

  pt_I_t po;
  po.x         = un_point.x();
  po.y         = un_point.y();
  po.z         = un_point.z();
  po.intensity = pi.intensity;
  return po;
}
/*//}*/

/*//{ setFrequency() */
template <class T_pt>
void AloamOdometry<T_pt>::setFrequency(const float frequency) {
  _scan_period_sec = 1.0f / frequency;
}
/*//}*/

// Explicit template instantiation
template struct OdometryData<ouster_ros::Point>;
template struct OdometryData<pandar_pointcloud::PointXYZIT>;

template class AloamOdometry<ouster_ros::Point>;
template class AloamOdometry<pandar_pointcloud::PointXYZIT>;

}  // namespace aloam_slam

