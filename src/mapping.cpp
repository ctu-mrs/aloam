#include "aloam_slam/mapping.h"

namespace aloam_slam
{

/*//{ AloamMapping() */
AloamMapping::AloamMapping(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::shared_ptr<mrs_lib::Profiler> profiler,
                           std::string frame_fcu, std::string frame_map, float scan_frequency, tf::Transform tf_lidar_to_fcu)
    : _profiler(profiler),
      _frame_fcu(frame_fcu),
      _frame_map(frame_map),
      _scan_frequency(scan_frequency),
      _tf_lidar_to_fcu(tf_lidar_to_fcu),
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

  {
    std::scoped_lock lock(_mutex_cloud_features);
    _cloud_corners.resize(_cloud_volume);
    _cloud_surfs.resize(_cloud_volume);
    for (int i = 0; i < _cloud_volume; i++) {
      _cloud_corners.at(i) = boost::make_shared<pcl::PointCloud<PointType>>();
      _cloud_surfs.at(i)   = boost::make_shared<pcl::PointCloud<PointType>>();
    }
  }

  _pub_laser_cloud_map        = nh_.advertise<sensor_msgs::PointCloud2>("map_out", 1);
  _pub_laser_cloud_registered = nh_.advertise<sensor_msgs::PointCloud2>("scan_registered_out", 1);
  _pub_odom_global            = nh_.advertise<nav_msgs::Odometry>("odom_global_out", 1);
  _pub_path                   = nh_.advertise<nav_msgs::Path>("path_out", 1);

  if (_mapping_frequency > 0.0f) {
    _timer_mapping_loop = nh_.createTimer(ros::Rate(_mapping_frequency), &AloamMapping::timerMapping, this, false, true);
  }

  _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>();

  _srv_reset_mapping = nh_.advertiseService("srv_reset_mapping_in", &AloamMapping::callbackResetMapping, this);

  _time_last_map_publish = ros::Time::now();
}
/*//}*/

/*//{ setData() */
void AloamMapping::setData(ros::Time time_of_data, tf::Transform aloam_odometry, pcl::PointCloud<PointType>::Ptr features_corners_last,
                           pcl::PointCloud<PointType>::Ptr features_surfs_last, pcl::PointCloud<PointType>::Ptr cloud_full_res) {

  mrs_lib::Routine profiler_routine = _profiler->createRoutine("aloamMappingSetData");

  std::scoped_lock lock(_mutex_odometry_data);
  _has_new_data          = true;
  _time_aloam_odometry   = time_of_data;
  _aloam_odometry        = aloam_odometry;
  _features_corners_last = features_corners_last;
  _features_surfs_last   = features_surfs_last;
  _cloud_full_res        = cloud_full_res;
}
/*//}*/

/*//{ timerMapping() */
void AloamMapping::timerMapping([[maybe_unused]] const ros::TimerEvent &event) {
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

  ros::Time                       time_aloam_odometry;
  tf::Transform                   aloam_odometry;
  pcl::PointCloud<PointType>::Ptr features_corners_last;
  pcl::PointCloud<PointType>::Ptr features_surfs_last;
  pcl::PointCloud<PointType>::Ptr cloud_full_res;
  {
    std::scoped_lock lock(_mutex_odometry_data);
    _has_new_data         = false;
    time_aloam_odometry   = _time_aloam_odometry;
    aloam_odometry        = _aloam_odometry;
    features_corners_last = _features_corners_last;
    features_surfs_last   = _features_surfs_last;
    cloud_full_res        = _cloud_full_res;
  }
  /*//}*/

  mrs_lib::Routine profiler_routine = _profiler->createRoutine("timerMapping", _mapping_frequency, 0.1, event);

  tf::vectorTFToEigen(aloam_odometry.getOrigin(), _t_wodom_curr);
  tf::quaternionTFToEigen(aloam_odometry.getRotation(), _q_wodom_curr);

  pcl::PointCloud<PointType>::Ptr map_features_corners = boost::make_shared<pcl::PointCloud<PointType>>();
  pcl::PointCloud<PointType>::Ptr map_features_surfs   = boost::make_shared<pcl::PointCloud<PointType>>();

  std::vector<int> cloud_valid_indices;

  /*//{ Associate odometry features to map features */
  TicToc t_whole;

  float time_build_tree   = 0.0f;
  float time_association  = 0.0f;
  float time_optimization = 0.0f;
  float time_solver       = 0.0f;

  TicToc t_shift;
  int    center_cube_I = int((_t_w_curr.x() + 25.0) / 50.0) + _cloud_center_width;
  int    center_cube_J = int((_t_w_curr.y() + 25.0) / 50.0) + _cloud_center_height;
  int    center_cube_K = int((_t_w_curr.z() + 25.0) / 50.0) + _cloud_center_depth;

  if (_t_w_curr.x() + 25.0 < 0) {
    center_cube_I--;
  }
  if (_t_w_curr.y() + 25.0 < 0) {
    center_cube_J--;
  }
  if (_t_w_curr.z() + 25.0 < 0) {
    center_cube_K--;
  }

  {
    std::scoped_lock lock(_mutex_cloud_features);

    transformAssociateToMap();

    while (center_cube_I < 3) {
      for (int j = 0; j < _cloud_height; j++) {
        for (int k = 0; k < _cloud_depth; k++) {
          int                             i             = _cloud_width - 1;
          pcl::PointCloud<PointType>::Ptr local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          pcl::PointCloud<PointType>::Ptr local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          for (; i >= 1; i--) {
            _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_corners.at(i - 1 + _cloud_width * j + _cloud_width * _cloud_height * k);
            _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_surfs.at(i - 1 + _cloud_width * j + _cloud_width * _cloud_height * k);
          }
          _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) = local_corners;
          _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k)   = local_surfs;
          local_corners->clear();
          local_surfs->clear();
        }
      }

      center_cube_I++;
      _cloud_center_width++;
    }

    while (center_cube_I >= _cloud_width - 3) {
      for (int j = 0; j < _cloud_height; j++) {
        for (int k = 0; k < _cloud_depth; k++) {
          int                             i             = 0;
          pcl::PointCloud<PointType>::Ptr local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          pcl::PointCloud<PointType>::Ptr local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          for (; i < _cloud_width - 1; i++) {
            _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_corners.at(i + 1 + _cloud_width * j + _cloud_width * _cloud_height * k);
            _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_surfs.at(i + 1 + _cloud_width * j + _cloud_width * _cloud_height * k);
          }
          _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) = local_corners;
          _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k)   = local_surfs;
          local_corners->clear();
          local_surfs->clear();
        }
      }

      center_cube_I--;
      _cloud_center_width--;
    }

    while (center_cube_J < 3) {
      for (int i = 0; i < _cloud_width; i++) {
        for (int k = 0; k < _cloud_depth; k++) {
          int                             j             = _cloud_height - 1;
          pcl::PointCloud<PointType>::Ptr local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          pcl::PointCloud<PointType>::Ptr local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          for (; j >= 1; j--) {
            _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_corners.at(i + _cloud_width * (j - 1) + _cloud_width * _cloud_height * k);
            _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_surfs.at(i + _cloud_width * (j - 1) + _cloud_width * _cloud_height * k);
          }
          _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) = local_corners;
          _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k)   = local_surfs;
          local_corners->clear();
          local_surfs->clear();
        }
      }

      center_cube_J++;
      _cloud_center_height++;
    }

    while (center_cube_J >= _cloud_height - 3) {
      for (int i = 0; i < _cloud_width; i++) {
        for (int k = 0; k < _cloud_depth; k++) {
          int                             j             = 0;
          pcl::PointCloud<PointType>::Ptr local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          pcl::PointCloud<PointType>::Ptr local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          for (; j < _cloud_height - 1; j++) {
            _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_corners.at(i + _cloud_width * (j + 1) + _cloud_width * _cloud_height * k);
            _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_surfs.at(i + _cloud_width * (j + 1) + _cloud_width * _cloud_height * k);
          }
          _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) = local_corners;
          _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k)   = local_surfs;
          local_corners->clear();
          local_surfs->clear();
        }
      }

      center_cube_J--;
      _cloud_center_height--;
    }

    while (center_cube_K < 3) {
      for (int i = 0; i < _cloud_width; i++) {
        for (int j = 0; j < _cloud_height; j++) {
          int                             k             = _cloud_depth - 1;
          pcl::PointCloud<PointType>::Ptr local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          pcl::PointCloud<PointType>::Ptr local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          for (; k >= 1; k--) {
            _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * (k - 1));
            _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * (k - 1));
          }
          _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) = local_corners;
          _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k)   = local_surfs;
          local_corners->clear();
          local_surfs->clear();
        }
      }

      center_cube_K++;
      _cloud_center_depth++;
    }

    while (center_cube_K >= _cloud_depth - 3) {
      for (int i = 0; i < _cloud_width; i++) {
        for (int j = 0; j < _cloud_height; j++) {
          int                             k             = 0;
          pcl::PointCloud<PointType>::Ptr local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          pcl::PointCloud<PointType>::Ptr local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          for (; k < _cloud_depth - 1; k++) {
            _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * (k + 1));
            _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * (k + 1));
          }
          _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) = local_corners;
          _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k)   = local_surfs;
          local_corners->clear();
          local_surfs->clear();
        }
      }

      center_cube_K--;
      _cloud_center_depth--;
    }

    for (int i = center_cube_I - 2; i <= center_cube_I + 2; i++) {
      for (int j = center_cube_J - 2; j <= center_cube_J + 2; j++) {
        for (int k = center_cube_K - 1; k <= center_cube_K + 1; k++) {
          if (i >= 0 && i < _cloud_width && j >= 0 && j < _cloud_height && k >= 0 && k < _cloud_depth) {
            cloud_valid_indices.push_back(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          }
        }
      }
    }
    /*//}*/

    for (unsigned int i = 0; i < cloud_valid_indices.size(); i++) {
      *map_features_corners += *_cloud_corners.at(cloud_valid_indices.at(i));
      *map_features_surfs += *_cloud_surfs.at(cloud_valid_indices.at(i));
    }
  }

  pcl::VoxelGrid<PointType> filter_downsize_corners;
  pcl::VoxelGrid<PointType> filter_downsize_surfs;
  filter_downsize_corners.setLeafSize(_resolution_line, _resolution_line, _resolution_line);
  filter_downsize_surfs.setLeafSize(_resolution_plane, _resolution_plane, _resolution_plane);

  /*//{*/
  pcl::PointCloud<PointType>::Ptr features_corners_stack = boost::make_shared<pcl::PointCloud<PointType>>();
  filter_downsize_corners.setInputCloud(features_corners_last);
  filter_downsize_corners.filter(*features_corners_stack);

  pcl::PointCloud<PointType>::Ptr features_surfs_stack = boost::make_shared<pcl::PointCloud<PointType>>();
  filter_downsize_surfs.setInputCloud(features_surfs_last);
  filter_downsize_surfs.filter(*features_surfs_stack);

  /* printf("map prepare time %f ms\n", t_shift.toc()); */
  /* printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum); */
  const float time_shift = t_shift.toc();

  if (map_features_corners->points.size() > 10 && map_features_surfs->points.size() > 50) {
    TicToc                           t_tree;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_map_corners(new pcl::KdTreeFLANN<PointType>());
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_map_surfs(new pcl::KdTreeFLANN<PointType>());
    kdtree_map_corners->setInputCloud(map_features_corners);
    kdtree_map_surfs->setInputCloud(map_features_surfs);
    time_build_tree = t_tree.toc();

    std::vector<int>   point_search_indices;
    std::vector<float> point_search_sq_dist;

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

      for (unsigned int i = 0; i < features_corners_stack->points.size(); i++) {
        const PointType point_ori = features_corners_stack->points.at(i);
        PointType       point_sel;
        // double sqrtDis = point_ori.x * point_ori.x + point_ori.y * point_ori.y + point_ori.z * point_ori.z;
        pointAssociateToMap(&point_ori, &point_sel);
        kdtree_map_corners->nearestKSearch(point_sel, 5, point_search_indices, point_search_sq_dist);

        if (point_search_sq_dist.at(4) < 1.0) {
          std::vector<Eigen::Vector3d> nearCorners;
          Eigen::Vector3d              center(0, 0, 0);
          for (int j = 0; j < 5; j++) {
            const Eigen::Vector3d tmp(map_features_corners->points.at(point_search_indices.at(j)).x,
                                      map_features_corners->points.at(point_search_indices.at(j)).y,
                                      map_features_corners->points.at(point_search_indices.at(j)).z);
            center = center + tmp;
            nearCorners.push_back(tmp);
          }
          center = center / 5.0;

          Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
          for (int j = 0; j < 5; j++) {
            const Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners.at(j) - center;
            covMat                                        = covMat + tmpZeroMean * tmpZeroMean.transpose();
          }

          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

          // if is indeed line feature
          // note Eigen library sort eigenvalues in increasing order
          if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
            const Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            const Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
            const Eigen::Vector3d point_a = 0.1 * unit_direction + center;
            const Eigen::Vector3d point_b = -0.1 * unit_direction + center;

            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
            problem.AddResidualBlock(cost_function, loss_function, _parameters, _parameters + 4);
            corner_num++;
          }
        }
      }

      int surf_num = 0;
      for (unsigned int i = 0; i < features_surfs_stack->points.size(); i++) {
        const PointType point_ori = features_surfs_stack->points.at(i);
        PointType       point_sel;
        pointAssociateToMap(&point_ori, &point_sel);
        kdtree_map_surfs->nearestKSearch(point_sel, 5, point_search_indices, point_search_sq_dist);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (point_search_sq_dist.at(4) < 1.0) {
          for (int j = 0; j < 5; j++) {
            matA0(j, 0) = map_features_surfs->points.at(point_search_indices.at(j)).x;
            matA0(j, 1) = map_features_surfs->points.at(point_search_indices.at(j)).y;
            matA0(j, 2) = map_features_surfs->points.at(point_search_indices.at(j)).z;
            // printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
          }
          // find the norm of plane
          Eigen::Vector3d norm                 = matA0.colPivHouseholderQr().solve(matB0);
          const double    negative_OA_dot_norm = 1 / norm.norm();
          norm.normalize();

          // Here n(pa, pb, pc) is unit norm of plane
          bool planeValid = true;
          for (int j = 0; j < 5; j++) {
            // if OX * n > 0.2, then plane is not fit well
            if (fabs(norm(0) * map_features_surfs->points.at(point_search_indices.at(j)).x +
                     norm(1) * map_features_surfs->points.at(point_search_indices.at(j)).y +
                     norm(2) * map_features_surfs->points.at(point_search_indices.at(j)).z + negative_OA_dot_norm) > 0.2) {
              planeValid = false;
              break;
            }
          }
          if (planeValid) {
            const Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
            ceres::CostFunction * cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
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
      time_solver += t_solver.toc();
    }
    time_optimization = t_opt.toc();
  } else {
    ROS_WARN("[AloamMapping] Not enough map correspondences. Skipping mapping frame.");
  }
  /*//}*/

  float time_add    = 0.0f;
  float time_filter = 0.0f;
  {
    std::scoped_lock lock(_mutex_cloud_features);

    transformUpdate();

    TicToc t_add;

    for (unsigned int i = 0; i < features_corners_stack->points.size(); i++) {
      PointType point_sel;
      pointAssociateToMap(&features_corners_stack->points.at(i), &point_sel);

      int cubeI = int((point_sel.x + 25.0) / 50.0) + _cloud_center_width;
      int cubeJ = int((point_sel.y + 25.0) / 50.0) + _cloud_center_height;
      int cubeK = int((point_sel.z + 25.0) / 50.0) + _cloud_center_depth;

      if (point_sel.x + 25.0 < 0)
        cubeI--;
      if (point_sel.y + 25.0 < 0)
        cubeJ--;
      if (point_sel.z + 25.0 < 0)
        cubeK--;

      if (cubeI >= 0 && cubeI < _cloud_width && cubeJ >= 0 && cubeJ < _cloud_height && cubeK >= 0 && cubeK < _cloud_depth) {
        const int cubeInd = cubeI + _cloud_width * cubeJ + _cloud_width * _cloud_height * cubeK;
        _cloud_corners.at(cubeInd)->push_back(point_sel);
      }
    }

    for (unsigned int i = 0; i < features_surfs_stack->points.size(); i++) {
      PointType point_sel;
      pointAssociateToMap(&features_surfs_stack->points.at(i), &point_sel);

      int cubeI = int((point_sel.x + 25.0) / 50.0) + _cloud_center_width;
      int cubeJ = int((point_sel.y + 25.0) / 50.0) + _cloud_center_height;
      int cubeK = int((point_sel.z + 25.0) / 50.0) + _cloud_center_depth;

      if (point_sel.x + 25.0 < 0)
        cubeI--;
      if (point_sel.y + 25.0 < 0)
        cubeJ--;
      if (point_sel.z + 25.0 < 0)
        cubeK--;

      if (cubeI >= 0 && cubeI < _cloud_width && cubeJ >= 0 && cubeJ < _cloud_height && cubeK >= 0 && cubeK < _cloud_depth) {
        const int cubeInd = cubeI + _cloud_width * cubeJ + _cloud_width * _cloud_height * cubeK;
        _cloud_surfs.at(cubeInd)->push_back(point_sel);
      }
    }
    /* printf("add points time %f ms\n", t_add.toc()); */
    time_add = t_add.toc();

    TicToc t_filter;
    for (unsigned int i = 0; i < cloud_valid_indices.size(); i++) {
      const int ind = cloud_valid_indices.at(i);

      pcl::PointCloud<PointType>::Ptr tmpCorner = boost::make_shared<pcl::PointCloud<PointType>>();
      filter_downsize_corners.setInputCloud(_cloud_corners.at(ind));
      filter_downsize_corners.filter(*tmpCorner);
      _cloud_corners.at(ind) = tmpCorner;

      pcl::PointCloud<PointType>::Ptr tmpSurf = boost::make_shared<pcl::PointCloud<PointType>>();
      filter_downsize_surfs.setInputCloud(_cloud_surfs.at(ind));
      filter_downsize_surfs.filter(*tmpSurf);
      _cloud_surfs.at(ind) = tmpSurf;
    }
    time_filter = t_filter.toc();
  }

  /*//{ Publish data */

  TicToc t_pub;

  // Publish TF
  // TF fcu -> aloam origin (_t_w_curr and _q_w_curr is in origin -> fcu frame, hence inversion)
  tf::Transform  tf_lidar;
  tf::Quaternion tf_q;
  tf_lidar.setOrigin(tf::Vector3(_t_w_curr(0), _t_w_curr(1), _t_w_curr(2)));
  tf::quaternionEigenToTF(_q_w_curr, tf_q);
  tf_lidar.setRotation(tf_q);

  tf::Transform tf_fcu = tf_lidar * _tf_lidar_to_fcu;

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp    = time_aloam_odometry;
  tf_msg.header.frame_id = _frame_fcu;
  tf_msg.child_frame_id  = _frame_map;
  tf::transformTFToMsg(tf_fcu.inverse(), tf_msg.transform);

  try {
    _tf_broadcaster->sendTransform(tf_msg);
  }
  catch (...) {
    ROS_ERROR("[AloamMapping]: Exception caught during publishing TF: %s - %s.", tf_msg.child_frame_id.c_str(), tf_msg.header.frame_id.c_str());
  }

  // Create nav_msgs::Odometry msg
  nav_msgs::Odometry::Ptr fcu_odom_msg = boost::make_shared<nav_msgs::Odometry>();
  fcu_odom_msg->header.frame_id        = _frame_map;
  fcu_odom_msg->header.stamp           = time_aloam_odometry;
  tf::pointTFToMsg(tf_fcu.getOrigin(), fcu_odom_msg->pose.pose.position);
  tf::quaternionTFToMsg(tf_fcu.getRotation(), fcu_odom_msg->pose.pose.orientation);


  // add points distant only 10cm (idea: throttle bandwidth)
  if ((_t_w_curr - _path_last_added_pos).norm() > 0.1) {
    _path_last_added_pos = _t_w_curr;

    // Create geometry_msgs::PoseStamped msg
    geometry_msgs::PoseStamped fcu_pose_msg;
    fcu_pose_msg.header     = fcu_odom_msg->header;
    fcu_pose_msg.pose       = fcu_odom_msg->pose.pose;
    _laser_path_msg->header = fcu_odom_msg->header;
    _laser_path_msg->poses.push_back(fcu_pose_msg);

    // Publish nav_msgs::Path msg
    if (_pub_path.getNumSubscribers() > 0) {
      try {
        _pub_path.publish(_laser_path_msg);
      }
      catch (...) {
        ROS_ERROR("[AloamMapping]: Exception caught during publishing topic %s.", _pub_path.getTopic().c_str());
      }
    }
  }

  // Publish nav_msgs::Odometry msg
  if (_pub_odom_global.getNumSubscribers() > 0) {
    fcu_odom_msg->child_frame_id = _frame_fcu;
    try {
      _pub_odom_global.publish(fcu_odom_msg);
    }
    catch (...) {
      ROS_ERROR("[AloamMapping]: Exception caught during publishing topic %s.", _pub_odom_global.getTopic().c_str());
    }
  }

  // Publish entire map
  if (_pub_laser_cloud_map.getNumSubscribers() > 0 && (ros::Time::now() - _time_last_map_publish).toSec() > _map_publish_period) {
    pcl::PointCloud<PointType> map_pcl;
    {
      std::scoped_lock lock(_mutex_cloud_features);
      for (int i = 0; i < _cloud_volume; i++) {
        map_pcl += *_cloud_corners.at(i);
        map_pcl += *_cloud_surfs.at(i);
      }
    }
    sensor_msgs::PointCloud2::Ptr map_msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(map_pcl, *map_msg);
    map_msg->header.stamp    = time_aloam_odometry;
    map_msg->header.frame_id = _frame_map;

    try {
      _pub_laser_cloud_map.publish(map_msg);
      _time_last_map_publish = ros::Time::now();
    }
    catch (...) {
      ROS_ERROR("[AloamMapping]: Exception caught during publishing topic %s.", _pub_laser_cloud_map.getTopic().c_str());
    }
  }

  // Publish registered sensor data
  if (_pub_laser_cloud_registered.getNumSubscribers() > 0) {
    // TODO: this pcl might be published in lidar frame instead of map saving some load..
    // or it might not be published at all as the data are already published by sensor and TFs are published above
    int cloud_full_resNum = cloud_full_res->points.size();
    for (int i = 0; i < cloud_full_resNum; i++) {
      pointAssociateToMap(&cloud_full_res->points.at(i), &cloud_full_res->points.at(i));
    }

    sensor_msgs::PointCloud2::Ptr cloud_full_res_msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloud_full_res, *cloud_full_res_msg);
    cloud_full_res_msg->header.stamp    = time_aloam_odometry;
    cloud_full_res_msg->header.frame_id = _frame_map;

    try {
      _pub_laser_cloud_registered.publish(cloud_full_res_msg);
    }
    catch (...) {
      ROS_ERROR("[AloamMapping]: Exception caught during publishing topic %s.", _pub_laser_cloud_registered.getTopic().c_str());
    }
  }

  /*//}*/

  _frame_count++;

  // Print diagnostics
  const float time_whole = t_whole.toc();

  ROS_INFO_THROTTLE(1.0, "[AloamMapping] Run time: %0.1f ms (%0.1f Hz)", time_whole, std::min(_mapping_frequency, 1000.0f / time_whole));
  ROS_DEBUG_THROTTLE(
      1.0,
      "[AloamMapping] features -> edges: %ld, surfs: %ld; data preparation: %0.1f ms; kd-tree initialization: %0.1f ms; "
      "publishing time: %0.1f ms; map -> association: %0.1f ms, solver: %0.1f ms, optimization: %0.1f ms, adding points: %0.1f ms, filtering: %0.1f ms",
      map_features_corners->points.size(), map_features_surfs->points.size(), time_shift, time_build_tree, t_pub.toc(), time_association, time_solver,
      time_optimization, time_add, time_filter);
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
  const Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  const Eigen::Vector3d point_w = _q_w_curr * point_curr + _t_w_curr;
  po->x                         = point_w.x();
  po->y                         = point_w.y();
  po->z                         = point_w.z();
  po->intensity                 = pi->intensity;
  // po->intensity = 1.0;
}
/*//}*/

/*//{ callbackResetMapping() */
bool AloamMapping::callbackResetMapping([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  std::scoped_lock lock(_mutex_cloud_features);
  _cloud_corners.resize(_cloud_volume);
  _cloud_surfs.resize(_cloud_volume);
  for (int i = 0; i < _cloud_volume; i++) {
    _cloud_corners.at(i) = boost::make_shared<pcl::PointCloud<PointType>>();
    _cloud_surfs.at(i)   = boost::make_shared<pcl::PointCloud<PointType>>();
  }
  ROS_INFO("[AloamMapping] Reset: map features were cleared.");

  res.success = true;
  res.message = "AloamMapping features were cleared.";
  return true;
}
/*//}*/

}  // namespace aloam_slam

