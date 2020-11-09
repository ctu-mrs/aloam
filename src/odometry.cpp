#include "aloam_slam/odometry.h"
#include "aloam_slam/mapping.h"

namespace aloam_slam
{

/* //{ AloamOdometry() */
AloamOdometry::AloamOdometry(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::string uav_name,
                             std::shared_ptr<mrs_lib::Profiler> profiler, std::shared_ptr<AloamMapping> aloam_mapping, std::string frame_fcu,
                             std::string frame_lidar, std::string frame_odom, float scan_period_sec, tf::Transform tf_lidar_to_fcu)
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

  ros::Time::waitForValid();

  param_loader.loadParam("mapping_line_resolution", _features_corners_resolution, 0.4f);
  param_loader.loadParam("mapping_plane_resolution", _features_surfs_resolution, 0.8f);

  param_loader.loadParam("feature_selection/enable", _features_selection_enabled, false);
  param_loader.loadParam("feature_selection/corners/gradient/limit/upper", _features_corners_gradient_limit_upper, 1.0f);
  param_loader.loadParam("feature_selection/corners/gradient/limit/bottom", _features_corners_gradient_limit_bottom, 0.0f);
  param_loader.loadParam("feature_selection/surfs/gradient/limit/upper", _features_surfs_gradient_limit_upper, 1.0f);
  param_loader.loadParam("feature_selection/surfs/gradient/limit/bottom", _features_surfs_gradient_limit_bottom, 0.0f);

  // Objects initialization
  _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>();

  {
    std::scoped_lock lock(_mutex_odometry_process);
    _features_corners_last = boost::make_shared<pcl::PointCloud<PointType>>();
    _features_surfs_last   = boost::make_shared<pcl::PointCloud<PointType>>();
  }

  _q_w_curr = Eigen::Quaterniond(1, 0, 0, 0);  // eigen has qw, qx, qy, qz notation
  _t_w_curr = Eigen::Vector3d(0, 0, 0);

  _transformer = std::make_shared<mrs_lib::Transformer>("AloamOdometry", uav_name);

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh         = nh_;
  shopts.node_name  = "AloamOdometry";
  shopts.threadsafe = true;

  /* _sub_handler_orientation = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "orientation_in", mrs_lib::no_timeout); */

  _pub_odometry_local = nh_.advertise<nav_msgs::Odometry>("odom_local_out", 1);

  _pub_features_corners_sharp      = nh_.advertise<sensor_msgs::PointCloud2>("features_corners_sharp_out", 1);
  _pub_features_corners_less_sharp = nh_.advertise<sensor_msgs::PointCloud2>("features_corners_less_sharp_out", 1);
  _pub_features_surfs_flat         = nh_.advertise<sensor_msgs::PointCloud2>("features_surfs_flat_out", 1);
  _pub_features_surfs_less_flat    = nh_.advertise<sensor_msgs::PointCloud2>("features_surfs_less_flat_out", 1);
  _pub_features_corners_selected   = nh_.advertise<sensor_msgs::PointCloud2>("features_corners_selected_out", 1);
  _pub_features_surfs_selected     = nh_.advertise<sensor_msgs::PointCloud2>("features_surfs_selected_out", 1);

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

    const int cornerPointsSharpNum = corner_points_sharp->points.size();
    const int surfPointsFlatNum    = surf_points_flat->points.size();

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
            if (int(_features_surfs_last->points.at(j).intensity) > (closestPointScanID + NEARBY_SCAN))
              break;

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
            if (int(_features_surfs_last->points.at(j).intensity) < (closestPointScanID - NEARBY_SCAN))
              break;

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

      // printf("coner_correspondance %d, plane_correspondence %d \n", corner_correspondence, plane_correspondence);
      time_data_association = t_data.toc();

      if ((corner_correspondence + plane_correspondence) < 10) {
        ROS_WARN("[AloamOdometry] low number of correspondence! corners: %d, planes: %d", corner_correspondence, plane_correspondence);
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

  /*//{ Correct orientation using inertial measurements */

  /* if (_sub_handler_orientation.hasMsg()) { */
  /*   // Get orientation msg */
  /*   auto odom_msg = _sub_handler_orientation.getMsg(); */

  /*   // Convert orientation to the odometry frame */
  /*   geometry_msgs::QuaternionStamped msg_ori; */
  /*   msg_ori.header     = odom_msg->header; */
  /*   msg_ori.quaternion = odom_msg->pose.pose.orientation; */
  /*   auto ret           = _transformer->transformSingle(_frame_odom, msg_ori); */

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

  const auto selected_features = selectFeatures(corner_points_less_sharp, surf_points_less_flat);

  /*//{ Save odometry data to AloamMapping */
  {
    std::scoped_lock lock(_mutex_odometry_process);

    _features_corners_last = corner_points_less_sharp;
    _features_surfs_last   = surf_points_less_flat;

    _features_corners_last->header.stamp = laser_cloud_full_res->header.stamp;
    _features_surfs_last->header.stamp   = laser_cloud_full_res->header.stamp;
    laser_cloud_full_res->header.stamp   = laser_cloud_full_res->header.stamp;

    _features_corners_last->header.frame_id = _frame_lidar;
    _features_surfs_last->header.frame_id   = _frame_lidar;
    laser_cloud_full_res->header.frame_id   = _frame_lidar;

    /* _aloam_mapping->setData(stamp, tf_lidar, _features_corners_last, _features_surfs_last, laser_cloud_full_res); */
    _aloam_mapping->setData(stamp, tf_lidar, selected_features.first, selected_features.second, laser_cloud_full_res);
  }
  /*//}*/

  /*//{ Publish features */
  publishCloud(_pub_features_corners_sharp, corner_points_sharp);
  publishCloud(_pub_features_corners_less_sharp, corner_points_less_sharp);
  publishCloud(_pub_features_surfs_flat, surf_points_flat);
  publishCloud(_pub_features_surfs_less_flat, surf_points_less_flat);
  publishCloud(_pub_features_corners_selected, selected_features.first);
  publishCloud(_pub_features_surfs_selected, selected_features.second);
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

/*//{ selectFeatures() */
std::pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr> AloamOdometry::selectFeatures(pcl::PointCloud<PointType>::Ptr corner_points,
                                                                                                          pcl::PointCloud<PointType>::Ptr surf_points) {

  if (!_features_selection_enabled) {
    return std::make_pair(corner_points, surf_points);
  }

  mrs_lib::Routine profiler_routine = _profiler->createRoutine("aloamOdometrySelectFeatures");

  TicToc t_routine;

  pcl::PointCloud<PointType>::Ptr selected_corners = selectFeaturesFromCloudByGradient(
      corner_points, _features_corners_resolution, _features_corners_gradient_limit_bottom, _features_corners_gradient_limit_upper);
  pcl::PointCloud<PointType>::Ptr selected_surfs =
      selectFeaturesFromCloudByGradient(surf_points, _features_surfs_resolution, _features_surfs_gradient_limit_bottom, _features_surfs_gradient_limit_upper);

  ROS_WARN_THROTTLE(0.5, "[AloamOdometry] feature selection of corners and surfs: %0.1f ms", t_routine.toc());

  return std::make_pair(selected_corners, selected_surfs);
}
/*//}*/

/*//{ selectFeaturesFromCloudByGradient() */
pcl::PointCloud<PointType>::Ptr AloamOdometry::selectFeaturesFromCloudByGradient(const pcl::PointCloud<PointType>::Ptr cloud, const float search_radius,
                                                                                 const float grad_min, const float grad_max) {

  pcl::PointCloud<PointType>::Ptr selected_features = boost::make_shared<pcl::PointCloud<PointType>>();

  if (cloud->size() == 0) {
    return selected_features;
  }

  std::vector<float> gradient_norms(cloud->size());
  double             grad_norm_max = -1.0;

  // Create kd tree for fast search
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_surfs_last;
  kdtree_surfs_last.setInputCloud(cloud);

  for (unsigned int i = 0; i < cloud->size(); i++) {

    const pcl::PointXYZI point = cloud->at(i);
    std::vector<int>     indices;
    std::vector<float>   squared_distances;

    // Find all features in given radius
    kdtree_surfs_last.radiusSearch(*cloud, i, search_radius, indices, squared_distances);

    Eigen::Vector3f gradient  = Eigen::Vector3f::Zero();
    double          grad_norm = 0.0;

    // Compute gradient
    if (indices.size() > 0) {

      Eigen::Vector3f point_xyz = Eigen::Vector3f(point.x, point.y, point.z);
      for (const int &ind : indices) {
        const pcl::PointXYZI neighbor = cloud->at(ind);
        gradient += Eigen::Vector3f(neighbor.x, neighbor.y, neighbor.z) - point_xyz;
      }

      gradient /= indices.size();
      grad_norm = gradient.norm();
    }

    gradient_norms.at(i) = grad_norm;

    grad_norm_max = std::max(grad_norm_max, grad_norm);
  }

  // Filter by gradient value
  for (unsigned int i = 0; i < cloud->size(); i++) {

    const float grad_norm = gradient_norms.at(i) / grad_norm_max;

    if (grad_norm >= grad_min && grad_norm <= grad_max) {
      pcl::PointXYZI point_grad = cloud->at(i);
      point_grad.intensity      = grad_norm;
      selected_features->push_back(point_grad);
    }
  }

  selected_features->header   = cloud->header;
  selected_features->width    = 1;
  selected_features->height   = selected_features->points.size();
  selected_features->is_dense = cloud->is_dense;

  return selected_features;
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

/*//{ publishCloud() */
void AloamOdometry::publishCloud(ros::Publisher publisher, const pcl::PointCloud<PointType>::Ptr cloud) {
  if (publisher.getNumSubscribers() > 0) {

    sensor_msgs::PointCloud2::Ptr cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloud, *cloud_msg);

    try {
      publisher.publish(cloud_msg);
    }
    catch (...) {
      ROS_ERROR("exception caught during publishing on topic: %s", publisher.getTopic().c_str());
    }
  }
}
/*//}*/

}  // namespace aloam_slam

