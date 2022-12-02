#include "aloam_slam/odometry.h"
#include "aloam_slam/mapping.h"

namespace aloam_slam
{

/* //{ AloamOdometry() */
AloamOdometry::AloamOdometry(const std::shared_ptr<CommonHandlers_t> handlers, const std::shared_ptr<AloamMapping> aloam_mapping)
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
    _features_corners_last = boost::make_shared<pcl::PointCloud<PointType>>();
    _features_surfs_last   = boost::make_shared<pcl::PointCloud<PointType>>();
  }

  _q_w_curr = Eigen::Quaterniond::Identity();  // eigen has qw, qx, qy, qz notation
  _t_w_curr = Eigen::Vector3d(0, 0, 0);

  _transformer = std::make_shared<mrs_lib::Transformer>("AloamOdometry");
  _transformer->setDefaultPrefix(_handlers->uav_name);

  /* _sub_handler_orientation = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "orientation_in", mrs_lib::no_timeout); */

  _pub_odometry_local  = nh_.advertise<nav_msgs::Odometry>("odom_local_out", 1);
  _timer_odometry_loop = nh_.createTimer(ros::Rate(_handlers->frequency), &AloamOdometry::timerOdometry, this, false, true);
}
//}

/*//{ timerOdometry() */
void AloamOdometry::timerOdometry([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized) {
    return;
  }

  std::unique_ptr<mrs_lib::ScopeTimer> timer;
  pcl::PointCloud<PointType>::Ptr      corner_points_sharp;
  pcl::PointCloud<PointType>::Ptr      corner_points_less_sharp;
  pcl::PointCloud<PointType>::Ptr      surf_points_flat;
  pcl::PointCloud<PointType>::Ptr      surf_points_less_flat;
  unsigned int                         proc_points_count;

  ros::Time                         stamp;
  std::uint64_t                     stamp_pcl;
  feature_extraction::FEDiagnostics diagnostics_fe;

  /*//{ Load latest features */
  {
    std::unique_lock lock(_mutex_odometry_data);

    const std::chrono::duration<float> timeout(_scan_period_sec);
    bool                               has_new_data = false;

    if (_cv_odometry_data.wait_for(lock, timeout) == std::cv_status::no_timeout) {
      has_new_data = _has_new_data;
    }

    if (!has_new_data) {
      return;
    }

    timer = std::make_unique<mrs_lib::ScopeTimer>("ALOAM::FeatureExtraction::timerOdometry", _handlers->scope_timer_logger, _handlers->enable_scope_timer);

    proc_points_count = _odometry_data->finite_points_count;
    stamp             = _odometry_data->stamp_ros;
    stamp_pcl         = _odometry_data->stamp_pcl;

    corner_points_sharp =
        cloudPointOStoCloudPoint(_odometry_data->manager_corners_sharp->getCloudPtr(), _odometry_data->manager_corners_sharp->getIndicesPtr());
    corner_points_less_sharp =
        cloudPointOStoCloudPoint(_odometry_data->manager_corners_less_sharp->getCloudPtr(), _odometry_data->manager_corners_less_sharp->getIndicesPtr());
    surf_points_flat = cloudPointOStoCloudPoint(_odometry_data->manager_surfs_flat->getCloudPtr(), _odometry_data->manager_surfs_flat->getIndicesPtr());
    surf_points_less_flat =
        cloudPointOStoCloudPoint(_odometry_data->manager_surfs_less_flat->getCloudPtr(), _odometry_data->manager_surfs_less_flat->getIndicesPtr());

    diagnostics_fe = _odometry_data->diagnostics_fe;
  }

  /*//}*/

  if (proc_points_count == 0) {
    ROS_WARN_THROTTLE(1.0, "[AloamOdometry]: Received an empty input cloud, skipping!");
    return;
  }

  const std::shared_ptr<MappingData> mapping_data = std::make_shared<MappingData>();

  timer->checkpoint("loading data");
  mapping_data->diagnostics_odometry.ms_loading_data = timer->getLifetime();

  mrs_lib::Routine profiler_routine = _handlers->profiler->createRoutine("timerOdometry", 1.0f / _scan_period_sec, 0.05, event);

  /*//{ Find features correspondences and compute local odometry */

  if (_frame_count > 0) {
    std::scoped_lock lock(_mutex_odometry_process);

    const int cornerPointsSharpNum = corner_points_sharp->points.size();
    const int surfPointsFlatNum    = surf_points_flat->points.size();

    pcl::KdTreeFLANN<pcl::PointXYZI> _kdtree_corners_last;
    pcl::KdTreeFLANN<pcl::PointXYZI> _kdtree_surfs_last;

    _kdtree_corners_last.setInputCloud(_features_corners_last);
    _kdtree_surfs_last.setInputCloud(_features_surfs_last);

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

      // find correspondence for corner features
      for (int i = 0; i < cornerPointsSharpNum; ++i) {
        TransformToStart(&(corner_points_sharp->points.at(i)), &pointSel);
        _kdtree_corners_last.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

        int closestPointInd = -1, minPointInd2 = -1;
        if (pointSearchSqDis.at(0) < DISTANCE_SQ_THRESHOLD) {
          closestPointInd              = pointSearchInd.at(0);
          const int closestPointScanID = int(_features_corners_last->points.at(closestPointInd).intensity);

          double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
          // search in the direction of increasing scan line
          for (int j = closestPointInd + 1; j < (int)_features_corners_last->points.size(); ++j) {
            // if in the same scan line, continue
            if (int(_features_corners_last->points.at(j).intensity) <= closestPointScanID) {
              continue;
            }

            // if not in nearby scans, end the loop
            if (int(_features_corners_last->points.at(j).intensity) > (closestPointScanID + NEARBY_SCAN)) {
              break;
            }

            const double pointSqDis = (_features_corners_last->points.at(j).x - pointSel.x) * (_features_corners_last->points.at(j).x - pointSel.x) +
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

            const double pointSqDis = (_features_corners_last->points.at(j).x - pointSel.x) * (_features_corners_last->points.at(j).x - pointSel.x) +
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
          const Eigen::Vector3d curr_point(corner_points_sharp->points.at(i).x, corner_points_sharp->points.at(i).y, corner_points_sharp->points.at(i).z);
          const Eigen::Vector3d last_point_a(_features_corners_last->points.at(closestPointInd).x, _features_corners_last->points.at(closestPointInd).y,
                                             _features_corners_last->points.at(closestPointInd).z);
          const Eigen::Vector3d last_point_b(_features_corners_last->points.at(minPointInd2).x, _features_corners_last->points.at(minPointInd2).y,
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
          const int closestPointScanID = int(_features_surfs_last->points.at(closestPointInd).intensity);
          double    minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

          // search in the direction of increasing scan line
          for (int j = closestPointInd + 1; j < (int)_features_surfs_last->points.size(); ++j) {
            // if not in nearby scans, end the loop
            if (int(_features_surfs_last->points.at(j).intensity) > (closestPointScanID + NEARBY_SCAN)) {
              break;
            }

            const double pointSqDis = (_features_surfs_last->points.at(j).x - pointSel.x) * (_features_surfs_last->points.at(j).x - pointSel.x) +
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
            if (int(_features_surfs_last->points.at(j).intensity) < (closestPointScanID - NEARBY_SCAN)) {
              break;
            }

            const double pointSqDis = (_features_surfs_last->points.at(j).x - pointSel.x) * (_features_surfs_last->points.at(j).x - pointSel.x) +
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

            const Eigen::Vector3d curr_point(surf_points_flat->points.at(i).x, surf_points_flat->points.at(i).y, surf_points_flat->points.at(i).z);
            const Eigen::Vector3d last_point_a(_features_surfs_last->points.at(closestPointInd).x, _features_surfs_last->points.at(closestPointInd).y,
                                               _features_surfs_last->points.at(closestPointInd).z);
            const Eigen::Vector3d last_point_b(_features_surfs_last->points.at(minPointInd2).x, _features_surfs_last->points.at(minPointInd2).y,
                                               _features_surfs_last->points.at(minPointInd2).z);
            const Eigen::Vector3d last_point_c(_features_surfs_last->points.at(minPointInd3).x, _features_surfs_last->points.at(minPointInd3).y,
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

  /*//{ Correct orientation using inertial measurements */

  /* if (_sub_handler_orientation.hasMsg()) { */
  /*   // Get orientation msg */
  /*   auto odom_msg = _sub_handler_orientation.getMsg(); */

  /*   // Convert orientation to the odometry frame */
  /*   geometry_msgs::QuaternionStamped msg_ori; */
  /*   msg_ori.header     = odom_msg->header; */
  /*   msg_ori.quaternion = odom_msg->pose.pose.orientation; */
  /*   auto ret           = _transformer->transformSingle(, msg_ori); */

  /*   if (ret) { */
  /*     // Set heading of odometry msg to be the aloam odometry estimated heading */
  /*     mrs_lib::AttitudeConverter q_aloam = _q_w_curr; */
  /*     mrs_lib::AttitudeConverter q_odom  = ret.value().quaternion; */
  /*     mrs_lib::AttitudeConverter q_ret   = q_odom.setHeading(q_aloam.getHeading()); */
  /*     tf::quaternionEigenToMsg(q_ret, ori); */

  /*     /1* ROS_DEBUG("q_aloam: %0.2f %0.2f %0.2f", q_aloam.getRoll(), q_aloam.getPitch(), q_aloam.getHeading()); *1/ */
  /*     /1* ROS_DEBUG("q_odom: %0.2f %0.2f %0.2f", q_odom.getRoll(), q_odom.getPitch(), q_odom.getHeading()); *1/ */
  /*     /1* ROS_DEBUG("q_ret (q_aloam heading: %0.2f): %0.2f %0.2f %0.2f", q_aloam.getHeading(), q_ret.getRoll(), q_ret.getPitch(), q_ret.getHeading()); *1/ */
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
    std::scoped_lock lock(_mutex_odometry_process);
    _features_corners_last = corner_points_less_sharp;
    _features_surfs_last   = surf_points_less_flat;

    _features_corners_last->header.stamp = stamp_pcl;
    _features_surfs_last->header.stamp   = stamp_pcl;

    _features_corners_last->header.frame_id = _handlers->frame_lidar;
    _features_surfs_last->header.frame_id   = _handlers->frame_lidar;

    mapping_data->stamp_ros          = stamp;
    mapping_data->odometry           = tf_lidar;
    mapping_data->cloud_corners_last = _features_corners_last;
    mapping_data->cloud_surfs_last   = _features_surfs_last;

    mapping_data->diagnostics_fe                = diagnostics_fe;
    mapping_data->diagnostics_odometry.ms_total = timer->getLifetime();

    _aloam_mapping->setData(mapping_data);
  }
  /*//}*/

  /*//{ Publish odometry */

  if (_frame_count > 0) {
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
}
/*//}*/

/*//{ setData() */
void AloamOdometry::setData(const std::shared_ptr<OdometryData> data) {

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

void AloamOdometry::setTransform(const Eigen::Vector3d &t, const Eigen::Quaterniond &q, const ros::Time &stamp) {
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
void AloamOdometry::TransformToStart(PointType const *const pi, PointType *const po) {
  // interpolation ratio
  double s = 1.0;
  if (DISTORTION) {
    s = (pi->intensity - int(pi->intensity)) / _scan_period_sec;
  }
  const Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, _q_last_curr);
  const Eigen::Vector3d    t_point_last = s * _t_last_curr;
  const Eigen::Vector3d    point(pi->x, pi->y, pi->z);
  const Eigen::Vector3d    un_point = q_point_last * point + t_point_last;

  po->x         = un_point.x();
  po->y         = un_point.y();
  po->z         = un_point.z();
  po->intensity = pi->intensity;
}
/*//}*/

/*//{ setFrequency() */
void AloamOdometry::setFrequency(const float frequency) {
  _scan_period_sec = 1.0f / frequency;
}
/*//}*/

/*//{ cloudPointOStoCloudPoint() */
pcl::PointCloud<PointType>::Ptr AloamOdometry::cloudPointOStoCloudPoint(const pcl::PointCloud<PointTypeOS>::Ptr cloud_in,
                                                                        const feature_selection::IndicesPtr_t   indices) {

  if (!cloud_in || !indices) {
    ROS_ERROR("[AloamOdometry::cloudPointOStoCloudPoint]: Null pointer to input cloud or input indices given, returning nullptr.");
    return nullptr;
  } else if (!cloud_in->isOrganized()) {
    ROS_ERROR("[AloamOdometry::cloudPointOStoCloudPoint]: Input cloud is NOT organized, returning nullptr.");
    return nullptr;
  }

  const pcl::PointCloud<PointType>::Ptr cloud_out = boost::make_shared<pcl::PointCloud<PointType>>();
  cloud_out->header                               = cloud_in->header;
  cloud_out->width                                = indices->size();
  cloud_out->height                               = 1;
  cloud_out->is_dense                             = true;

  cloud_out->reserve(cloud_out->width);
  for (const auto &idx : *indices) {
    const auto &point_os = cloud_in->at(idx.col, idx.row);

    PointType point;
    point.x         = point_os.x;
    point.y         = point_os.y;
    point.z         = point_os.z;
    point.intensity = float(point_os.ring) + _scan_period_sec * (float(idx.col) / float(cloud_in->width));

    cloud_out->push_back(point);
  }

  return cloud_out;
}
/*//}*/

}  // namespace aloam_slam

