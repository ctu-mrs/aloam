#include "aloam_slam/mapping.h"

namespace aloam_slam
{

/*//{ AloamMapping() */
AloamMapping::AloamMapping(const std::shared_ptr<CommonHandlers_t> handlers) : _q_w_curr(_parameters), _t_w_curr(_parameters + 4) {

  _handlers        = handlers;
  _scan_period_sec = 1.0f / handlers->frequency;

  ros::Time::waitForValid();

  _handlers->param_loader->loadParam("mapping/remap_tf", _remap_tf, false);
  _handlers->param_loader->loadParam("mapping/line_resolution", _resolution_line, 0.2f);
  _handlers->param_loader->loadParam("mapping/plane_resolution", _resolution_plane, 0.4f);
  _handlers->param_loader->loadParam("mapping/rate", _mapping_frequency, 5.0f);
  _handlers->param_loader->loadParam("mapping/publish_rate", _map_publish_period, 0.5f);

  _filter_downsize_corners.setLeafSize(_resolution_line, _resolution_line, _resolution_line);
  _filter_downsize_surfs.setLeafSize(_resolution_plane, _resolution_plane, _resolution_plane);

  _map_publish_period = 1.0f / _map_publish_period;

  _q_wmap_wodom = Eigen::Quaterniond::Identity();
  _t_wmap_wodom = Eigen::Vector3d(0, 0, 0);

  _q_wodom_curr = Eigen::Quaterniond::Identity();
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

  _pub_diag            = handlers->nh.advertise<aloam_slam::AloamDiagnostics>("diag_out", 1);
  _pub_laser_cloud_map = handlers->nh.advertise<sensor_msgs::PointCloud2>("map_out", 1);
  _pub_odom_global     = handlers->nh.advertise<nav_msgs::Odometry>("odom_global_out", 1);
  _pub_path            = handlers->nh.advertise<nav_msgs::Path>("path_out", 1);
  _pub_eigenvalue      = handlers->nh.advertise<mrs_msgs::Float64ArrayStamped>("eigenvalues_out", 1);

  if (!handlers->offline_run && _mapping_frequency > 0.0f) {

    _timer_mapping_loop = handlers->nh.createTimer(ros::Rate(_mapping_frequency), &AloamMapping::timerMapping, this, false, true);
  }

  _tf_broadcaster    = std::make_shared<tf2_ros::TransformBroadcaster>();
  _srv_reset_mapping = handlers->nh.advertiseService("srv_reset_mapping_in", &AloamMapping::callbackResetMapping, this);

  _time_last_map_publish = ros::Time::now();
}
/*//}*/

/*//{ setData() */
void AloamMapping::setData(const std::shared_ptr<MappingData> data) {

  mrs_lib::Routine profiler_routine = _handlers->profiler->createRoutine("aloamMappingSetData");

  {
    std::unique_lock lock(_mutex_mapping_data);
    _has_new_data = true;
    _mapping_data = data;
  }

  _cv_mapping_data.notify_all();
}
/*//}*/

/*//{ computeMapping() */
bool AloamMapping::computeMapping(geometry_msgs::TransformStamped &tf_msg_out, aloam_slam::AloamDiagnostics::Ptr &diag_msg_out) {

  if (!is_initialized) {
    return false;
  }

  std::unique_ptr<mrs_lib::ScopeTimer> timer;

  /*//{ Load latest odometry data (blocks for _cv_mapping_data and timeouts if blocking takes longer than 1.0/_mapping_frequency) */
  ros::Time                       time_aloam_odometry;
  tf::Transform                   aloam_odometry;
  pcl::PointCloud<PointType>::Ptr features_corners_last;
  pcl::PointCloud<PointType>::Ptr features_surfs_last;

  {
    std::unique_lock lock(_mutex_mapping_data);

    bool has_new_data = false;

    if (_handlers->offline_run) {

      has_new_data = _has_new_data;

    } else {

      const std::chrono::duration<float> timeout(1.0 / _mapping_frequency);
      if (_cv_mapping_data.wait_for(lock, timeout) == std::cv_status::no_timeout) {
        has_new_data = _has_new_data;
      }
    }

    if (!has_new_data) {
      return false;
    }

    timer = std::make_unique<mrs_lib::ScopeTimer>("ALOAM::FeatureExtraction::timerMapping", _handlers->scope_timer_logger, _handlers->enable_scope_timer);
    const float t_start = timer->getLifetime();

    time_aloam_odometry = _mapping_data->stamp_ros;
    aloam_odometry      = _mapping_data->odometry;
    features_corners_last =
        cloudPointOStoCloudPoint(_mapping_data->manager_corners->getCloudPtr(), _mapping_data->manager_corners->getIndicesPtr(), _scan_period_sec);
    features_surfs_last =
        cloudPointOStoCloudPoint(_mapping_data->manager_surfs->getCloudPtr(), _mapping_data->manager_surfs->getIndicesPtr(), _scan_period_sec);

    diag_msg_out                     = boost::make_shared<aloam_slam::AloamDiagnostics>();
    diag_msg_out->feature_extraction = _mapping_data->diagnostics_fe;
    diag_msg_out->odometry           = _mapping_data->diagnostics_odometry;

    _has_new_data = false;
  }
  /*//}*/

  timer->checkpoint("loading data");
  diag_msg_out->mapping.ms_loading_data = timer->getLifetime();

  mrs_lib::Routine profiler_routine = _handlers->profiler->createRoutine("timerMapping");

  tf::vectorTFToEigen(aloam_odometry.getOrigin(), _t_wodom_curr);
  tf::quaternionTFToEigen(aloam_odometry.getRotation(), _q_wodom_curr);

  const pcl::PointCloud<PointType>::Ptr map_features_corners = boost::make_shared<pcl::PointCloud<PointType>>();
  const pcl::PointCloud<PointType>::Ptr map_features_surfs   = boost::make_shared<pcl::PointCloud<PointType>>();

  std::vector<int> cloud_valid_indices;

  /*//{ Associate odometry features to map features */

  int center_cube_I = int((_t_w_curr.x() + 25.0) / 50.0) + _cloud_center_width;
  int center_cube_J = int((_t_w_curr.y() + 25.0) / 50.0) + _cloud_center_height;
  int center_cube_K = int((_t_w_curr.z() + 25.0) / 50.0) + _cloud_center_depth;

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
          int                                   i             = _cloud_width - 1;
          const pcl::PointCloud<PointType>::Ptr local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          const pcl::PointCloud<PointType>::Ptr local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
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
          int                                   i             = 0;
          const pcl::PointCloud<PointType>::Ptr local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          const pcl::PointCloud<PointType>::Ptr local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
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
          int                                   j             = _cloud_height - 1;
          const pcl::PointCloud<PointType>::Ptr local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          const pcl::PointCloud<PointType>::Ptr local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
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
          int                                   j             = 0;
          const pcl::PointCloud<PointType>::Ptr local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          const pcl::PointCloud<PointType>::Ptr local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
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
          int                                   k             = _cloud_depth - 1;
          const pcl::PointCloud<PointType>::Ptr local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          const pcl::PointCloud<PointType>::Ptr local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
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
          int                                   k             = 0;
          const pcl::PointCloud<PointType>::Ptr local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          const pcl::PointCloud<PointType>::Ptr local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
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

    for (unsigned int i = 0; i < cloud_valid_indices.size(); i++) {
      *map_features_corners += *_cloud_corners.at(cloud_valid_indices.at(i));
      *map_features_surfs += *_cloud_surfs.at(cloud_valid_indices.at(i));
    }
  }
  /*//}*/

  /*//{*/
  // TODO: test voxelization with custom filter
  const pcl::PointCloud<PointType>::Ptr features_corners_stack = boost::make_shared<pcl::PointCloud<PointType>>();
  _filter_downsize_corners.setInputCloud(features_corners_last);
  _filter_downsize_corners.filter(*features_corners_stack);

  const pcl::PointCloud<PointType>::Ptr features_surfs_stack = boost::make_shared<pcl::PointCloud<PointType>>();
  _filter_downsize_surfs.setInputCloud(features_surfs_last);
  _filter_downsize_surfs.filter(*features_surfs_stack);

  /* printf("map prepare time %f ms\n", t_shift.toc()); */
  /* printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum); */
  if (map_features_corners->points.size() > 10 && map_features_surfs->points.size() > 50) {
    const pcl::KdTreeFLANN<PointType>::Ptr kdtree_map_corners(new pcl::KdTreeFLANN<PointType>());
    const pcl::KdTreeFLANN<PointType>::Ptr kdtree_map_surfs(new pcl::KdTreeFLANN<PointType>());
    kdtree_map_corners->setInputCloud(map_features_corners);
    kdtree_map_surfs->setInputCloud(map_features_surfs);

    for (int iterCount = 0; iterCount < 2; iterCount++) {
      // ceres::LossFunction *loss_function = NULL;
      ceres::LossFunction          *loss_function      = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
      ceres::Problem::Options       problem_options;

      ceres::Problem problem(problem_options);
      problem.AddParameterBlock(_parameters, 4, q_parameterization);
      problem.AddParameterBlock(_parameters + 4, 3);


      /* mrs_lib::ScopeTimer timer("finding factors", nullptr, true); */
      auto factors = findFactors(kdtree_map_corners, kdtree_map_surfs, features_corners_stack, features_surfs_stack, map_features_corners, map_features_surfs);
      /* timer.checkpoint("found"); */

      /* if (false) { */
      /* factors = selectFactorsGreedy(factors); */
      /* } */

      /* ROS_ERROR("After selectFactorsGreedy()"); */

      for (const auto &factor : factors.first) {
        problem.AddResidualBlock(LidarEdgeFactor::Create(factor), loss_function, _parameters, _parameters + 4);
      }
      for (const auto &factor : factors.second) {
        problem.AddResidualBlock(LidarPlaneFactor::Create(factor), loss_function, _parameters, _parameters + 4);
      }
      /* ROS_ERROR("After adding blocks to problem."); */

      ceres::Solver::Options options;
      options.linear_solver_type                = ceres::DENSE_QR;
      options.max_num_iterations                = 4;
      options.minimizer_progress_to_stdout      = false;
      options.check_gradients                   = false;
      options.gradient_check_relative_precision = 1e-4;
      ceres::Solver::Summary summary;

      // Get eigenvalues to measure problem degradation //{
      /* TicToc t_eigenvalues; */

      if (_pub_eigenvalue.getNumSubscribers() > 0) {

        // Get Jacobian
        ceres::CRSMatrix jacobian;
        problem.Evaluate(ceres::Problem::EvaluateOptions(), nullptr, nullptr, nullptr, &jacobian);

        // Convert it to eigen matrix
        Eigen::MatrixXd eigen_matrix;
        eigen_matrix.resize(jacobian.num_rows, jacobian.num_cols);
        eigen_matrix.setZero();

        for (int row = 0; row < jacobian.num_rows; ++row) {
          const int start = jacobian.rows.at(row);
          const int end   = jacobian.rows.at(row + 1) - 1;

          for (int i = start; i <= end; i++) {
            const int    col   = jacobian.cols.at(i);
            const double value = jacobian.values.at(i);

            eigen_matrix(row, col) = value;
          }
        }

        // calculate matrix J^T*J
        eigen_matrix = eigen_matrix.transpose() * eigen_matrix;

        // get eigenvalues and eigenvectors
        Eigen::EigenSolver<Eigen::MatrixXd> eigensolver;
        eigensolver.compute(eigen_matrix);
        const Eigen::VectorXd eigen_values  = eigensolver.eigenvalues().real();
        const Eigen::MatrixXd eigen_vectors = eigensolver.eigenvectors().real();

        std::cout << "\n" << eigen_values << std::endl;
        std::cout << eigen_vectors << "\n" << std::endl;

        const mrs_msgs::Float64ArrayStamped::Ptr msg_eigenvalue = boost::make_shared<mrs_msgs::Float64ArrayStamped>();
        msg_eigenvalue->header.stamp                            = time_aloam_odometry;
        msg_eigenvalue->values.resize(6, 0.0);

        // Find eigenvalues corresponding to respective state elements
        // the order should be theta_x, theta_y, theta_z, x, y, z
        for (int i = 0; i < 6; i++) {
          double max_eig_vec = 0;
          for (int j = 0; j < eigen_vectors.cols(); j++) {
            const double eig_vec = abs(eigen_vectors(i, j));
            if (eig_vec > max_eig_vec) {
              max_eig_vec                  = eig_vec;
              msg_eigenvalue->values.at(i) = eigen_values(j);
            }
          }
        }

        // Publish eigenvalues
        try {
          _pub_eigenvalue.publish(msg_eigenvalue);
        }
        catch (...) {
          ROS_ERROR("[AloamMapping]: Exception caught during publishing topic: %s.", _pub_eigenvalue.getTopic().c_str());
        }
      }

      /* float time_eigenvalues = t_eigenvalues.toc(); */
      /* ROS_INFO("[AloamMapping] Eigenvalue calculation took %.3f ms.", time_eigenvalues); */
      /*//}*/

      /* ROS_ERROR("Before solving"); */
      ceres::Solve(options, &problem, &summary);
      /* ROS_ERROR("After solving"); */
    }
  } else {
    ROS_WARN("[AloamMapping] Not enough map correspondences. Skipping mapping frame.");
  }
  /*//}*/

  timer->checkpoint("associating features with map");
  diag_msg_out->mapping.ms_associating_features = timer->getLifetime();

  {
    std::scoped_lock lock(_mutex_cloud_features);

    transformUpdate();

    for (unsigned int i = 0; i < features_corners_stack->points.size(); i++) {
      PointType point_sel;
      pointAssociateToMap(&features_corners_stack->points.at(i), &point_sel);

      int cubeI = int((point_sel.x + 25.0) / 50.0) + _cloud_center_width;
      int cubeJ = int((point_sel.y + 25.0) / 50.0) + _cloud_center_height;
      int cubeK = int((point_sel.z + 25.0) / 50.0) + _cloud_center_depth;

      if (point_sel.x + 25.0 < 0) {
        cubeI--;
      }
      if (point_sel.y + 25.0 < 0) {
        cubeJ--;
      }
      if (point_sel.z + 25.0 < 0) {
        cubeK--;
      }

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

      if (point_sel.x + 25.0 < 0) {
        cubeI--;
      }
      if (point_sel.y + 25.0 < 0) {
        cubeJ--;
      }
      if (point_sel.z + 25.0 < 0) {
        cubeK--;
      }

      if (cubeI >= 0 && cubeI < _cloud_width && cubeJ >= 0 && cubeJ < _cloud_height && cubeK >= 0 && cubeK < _cloud_depth) {
        const int cubeInd = cubeI + _cloud_width * cubeJ + _cloud_width * _cloud_height * cubeK;
        _cloud_surfs.at(cubeInd)->push_back(point_sel);
      }
    }

    for (unsigned int i = 0; i < cloud_valid_indices.size(); i++) {
      const int ind = cloud_valid_indices.at(i);

      const pcl::PointCloud<PointType>::Ptr tmpCorner = boost::make_shared<pcl::PointCloud<PointType>>();
      _filter_downsize_corners.setInputCloud(_cloud_corners.at(ind));
      _filter_downsize_corners.filter(*tmpCorner);
      _cloud_corners.at(ind) = tmpCorner;

      const pcl::PointCloud<PointType>::Ptr tmpSurf = boost::make_shared<pcl::PointCloud<PointType>>();
      _filter_downsize_surfs.setInputCloud(_cloud_surfs.at(ind));
      _filter_downsize_surfs.filter(*tmpSurf);
      _cloud_surfs.at(ind) = tmpSurf;
    }
  }

  timer->checkpoint("adding features to map");
  diag_msg_out->mapping.ms_adding_features_to_map = timer->getLifetime();

  /*//{ Publish data */

  // Publish TF
  // TF fcu -> aloam origin (_t_w_curr and _q_w_curr is in origin -> fcu frame, hence inversion)
  tf::Transform  tf_lidar;
  tf::Quaternion tf_q;
  tf_lidar.setOrigin(tf::Vector3(_t_w_curr(0), _t_w_curr(1), _t_w_curr(2)));
  tf::quaternionEigenToTF(_q_w_curr, tf_q);
  tf_lidar.setRotation(tf_q);

  const tf::Transform tf_fcu = tf_lidar * _handlers->tf_lidar_in_fcu_frame;

  tf_msg_out.header.stamp    = time_aloam_odometry;
  tf_msg_out.header.frame_id = _handlers->frame_fcu;
  if (_remap_tf) {
    tf_msg_out.child_frame_id = _handlers->frame_map + "_orig";
  } else {
    tf_msg_out.child_frame_id = _handlers->frame_map;
  }
  tf::transformTFToMsg(tf_fcu.inverse(), tf_msg_out.transform);

  try {
    _tf_broadcaster->sendTransform(tf_msg_out);
  }
  catch (...) {
    ROS_ERROR("[AloamMapping]: Exception caught during publishing TF: %s - %s.", tf_msg_out.child_frame_id.c_str(), tf_msg_out.header.frame_id.c_str());
  }

  // Create nav_msgs::Odometry msg
  const nav_msgs::Odometry::Ptr fcu_odom_msg = boost::make_shared<nav_msgs::Odometry>();
  fcu_odom_msg->header.frame_id              = _handlers->frame_map;
  fcu_odom_msg->header.stamp                 = time_aloam_odometry;
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
    fcu_odom_msg->child_frame_id = _handlers->frame_fcu;
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
    const sensor_msgs::PointCloud2::Ptr map_msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(map_pcl, *map_msg);
    map_msg->header.stamp    = time_aloam_odometry;
    map_msg->header.frame_id = _handlers->frame_map;

    try {
      _pub_laser_cloud_map.publish(map_msg);
      _time_last_map_publish = ros::Time::now();
    }
    catch (...) {
      ROS_ERROR("[AloamMapping]: Exception caught during publishing topic %s.", _pub_laser_cloud_map.getTopic().c_str());
    }
  }

  /*//}*/

  diag_msg_out->frame            = ++_frame_count;
  diag_msg_out->mapping.ms_total = timer->getLifetime();
  diag_msg_out->ms_total         = diag_msg_out->feature_extraction.total_processing_time_ms + diag_msg_out->odometry.ms_total + diag_msg_out->mapping.ms_total;
  diag_msg_out->data_stamp       = time_aloam_odometry;

  // Publish diagnostics
  if (_pub_diag.getNumSubscribers() > 0) {

    try {
      _pub_diag.publish(diag_msg_out);
    }
    catch (...) {
      ROS_ERROR("[AloamMapping]: Exception caught during publishing topic: %s.", _pub_diag.getTopic().c_str());
    }
  }

  ROS_INFO_THROTTLE(1.0, "[Aloam] Run time: %.1f ms (FE: %.0f | FS: %.0f | O: %.0f | M: %.0f)", diag_msg_out->ms_total,
                    diag_msg_out->feature_extraction.extraction_processing_time_ms, diag_msg_out->feature_extraction.selection_processing_time_ms,
                    diag_msg_out->odometry.ms_total, diag_msg_out->mapping.ms_total);

  return true;
}
/*//}*/

/*//{ timerMapping() */
void AloamMapping::timerMapping([[maybe_unused]] const ros::TimerEvent &event) {
  geometry_msgs::TransformStamped   tf_msg;
  aloam_slam::AloamDiagnostics::Ptr diag_msg;

  computeMapping(tf_msg, diag_msg);
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
  po->x                         = float(point_w.x());
  po->y                         = float(point_w.y());
  po->z                         = float(point_w.z());
  po->intensity                 = pi->intensity;
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

/* setTransform() //{ */

void AloamMapping::setTransform(const Eigen::Vector3d &t, const Eigen::Quaterniond &q, const ros::Time &stamp) {
  _q_wodom_curr = q;
  _t_wodom_curr = t;

  transformAssociateToMap();
  transformUpdate();

  // Publish TF
  // TF fcu -> aloam origin (_t_w_curr and _q_w_curr is in origin -> fcu frame, hence inversion)
  tf::Transform  tf_lidar;
  tf::Quaternion tf_q;
  tf_lidar.setOrigin(tf::Vector3(_t_w_curr(0), _t_w_curr(1), _t_w_curr(2)));
  tf::quaternionEigenToTF(_q_w_curr, tf_q);
  tf_lidar.setRotation(tf_q);

  tf::Transform tf_fcu = tf_lidar * _handlers->tf_lidar_in_fcu_frame;

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp    = stamp;
  tf_msg.header.frame_id = _handlers->frame_fcu;
  if (_remap_tf) {
    tf_msg.child_frame_id = _handlers->frame_map + "_orig";
  } else {
    tf_msg.child_frame_id = _handlers->frame_map;
  }
  tf::transformTFToMsg(tf_fcu.inverse(), tf_msg.transform);

  try {
    _tf_broadcaster->sendTransform(tf_msg);
  }
  catch (...) {
    ROS_ERROR("[AloamMapping]: Exception caught during publishing TF: %s - %s.", tf_msg.child_frame_id.c_str(), tf_msg.header.frame_id.c_str());
  }

  // Create nav_msgs::Odometry msg
  nav_msgs::Odometry::Ptr fcu_odom_msg = boost::make_shared<nav_msgs::Odometry>();
  fcu_odom_msg->header.frame_id        = _handlers->frame_map;
  fcu_odom_msg->header.stamp           = stamp;
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
    fcu_odom_msg->child_frame_id = _handlers->frame_fcu;
    try {
      _pub_odom_global.publish(fcu_odom_msg);
    }
    catch (...) {
      ROS_ERROR("[AloamMapping]: Exception caught during publishing topic %s.", _pub_odom_global.getTopic().c_str());
    }
  }
}

//}

/* findFactors() //{ */
std::pair<std::vector<LidarEdgeFactor *>, std::vector<LidarPlaneFactor *>> AloamMapping::findFactors(
    const pcl::KdTreeFLANN<PointType>::Ptr kdtree_map_corners, const pcl::KdTreeFLANN<PointType>::Ptr kdtree_map_surfs,
    const pcl::PointCloud<PointType>::Ptr features_corners_stack, const pcl::PointCloud<PointType>::Ptr features_surfs_stack,
    const pcl::PointCloud<PointType>::Ptr map_features_corners, const pcl::PointCloud<PointType>::Ptr map_features_surfs) {

  std::vector<LidarEdgeFactor *>  factors_corners;
  std::vector<LidarPlaneFactor *> factors_surfs;

  /*//{ Corners */
  for (unsigned int i = 0; i < features_corners_stack->points.size(); i++) {
    std::vector<int>   point_search_indices;
    std::vector<float> point_search_sq_dist;

    const PointType point_ori = features_corners_stack->points.at(i);
    PointType       point_sel;
    pointAssociateToMap(&point_ori, &point_sel);
    kdtree_map_corners->nearestKSearch(point_sel, 5, point_search_indices, point_search_sq_dist);

    // If the correspondence is found
    if (point_search_sq_dist.at(4) < 1.0) {
      Eigen::Vector3d              center(0, 0, 0);
      std::vector<Eigen::Vector3d> nearCorners;
      nearCorners.reserve(5);
      for (int j = 0; j < 5; j++) {
        const auto &point = map_features_corners->points.at(point_search_indices.at(j));

        const Eigen::Vector3d corner(point.x, point.y, point.z);
        center += corner;
        nearCorners.push_back(corner);
      }
      center = center / 5.0;

      Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
      for (auto &corner : nearCorners) {
        const auto tmpZeroMean = corner - center;
        covMat += tmpZeroMean * tmpZeroMean.transpose();
      }

      // if is indeed line feature
      // note Eigen library sort eigenvalues in increasing order
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);
      if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
        const Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
        const Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);
        const Eigen::Vector3d point_a = center + unit_direction;
        const Eigen::Vector3d point_b = center - unit_direction;

        LidarEdgeFactor *factor = new LidarEdgeFactor(curr_point, point_a, point_b, 1.0);
        factors_corners.push_back(factor);
      }
    }
  }
  /*//}*/

  /*//{ Surfs */
  for (unsigned int i = 0; i < features_surfs_stack->points.size(); i++) {
    std::vector<int>   point_search_indices;
    std::vector<float> point_search_sq_dist;

    const PointType point_ori = features_surfs_stack->points.at(i);
    PointType       point_sel;
    pointAssociateToMap(&point_ori, &point_sel);
    kdtree_map_surfs->nearestKSearch(point_sel, 5, point_search_indices, point_search_sq_dist);

    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
    if (point_search_sq_dist.at(4) < 1.0) {
      for (int j = 0; j < 5; j++) {
        const auto &point = map_features_surfs->points.at(point_search_indices.at(j));
        matA0(j, 0)       = point.x;
        matA0(j, 1)       = point.y;
        matA0(j, 2)       = point.z;
        // printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
      }
      // find the norm of plane
      Eigen::Vector3d n = matA0.colPivHouseholderQr().solve(matB0);
      const double    d = 1 / n.norm();
      n.normalize();

      // Here n(pa, pb, pc) is unit norm of plane
      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        const auto &point = map_features_surfs->points.at(point_search_indices.at(j));
        // if OX * n > 0.2, then plane is not fit well
        if (std::fabs(n(0) * point.x + n(1) * point.y + n(2) * point.z + d) > 0.2) {
          planeValid = false;
          break;
        }
      }
      if (planeValid) {
        const Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);

        LidarPlaneFactor *factor = new LidarPlaneFactor(curr_point, n, d);
        factors_surfs.push_back(factor);
      }
    }
  }
  /*//}*/

  return {factors_corners, factors_surfs};
}
//}

/* selectFactorsGreedy() //{ */
std::pair<std::vector<LidarEdgeFactor *>, std::vector<LidarPlaneFactor *>> AloamMapping::selectFactorsGreedy(
    const std::pair<std::vector<LidarEdgeFactor *>, std::vector<LidarPlaneFactor *>> &factors) {

  // TODO: Integrate into findFactors() so not all feature correspondences have to be found. This is for debugging.

  const auto &factors_corners = factors.first;
  const auto &factors_surfs   = factors.second;

  std::vector<LidarEdgeFactor *>  factors_corners_sel;
  std::vector<LidarPlaneFactor *> factors_surfs_sel;
  factors_corners_sel.reserve(factors_corners.size());
  factors_surfs_sel.reserve(factors_surfs.size());

  const size_t N_corners = factors_corners.size();
  const size_t N         = N_corners + factors_surfs.size();
  const size_t M         = 0.3 * N;
  const double eps       = 0.05;

  struct point_t
  {
    /* double                      residual[3]; */
    bool                        has_data = false;
    Eigen::Matrix<double, 3, 1> residual;
    Eigen::Matrix<double, 3, 6> jacobian;
    Eigen::Matrix<double, 6, 6> inf_mat;
  };
  std::vector<point_t> points(N);

  std::vector<size_t> indices(N);
  std::iota(std::begin(indices), std::end(indices), 0);  // Fill with 0 to N in increasing order

  double Q[4];
  double t[3];

  Q[0] = _q_w_curr.x();
  Q[1] = _q_w_curr.y();
  Q[2] = _q_w_curr.z();
  Q[3] = _q_w_curr.w();
  t[0] = _t_w_curr.x();
  t[1] = _t_w_curr.y();
  t[2] = _t_w_curr.z();

  Eigen::Matrix<double, 6, 6> inf_mat = Eigen::Matrix<double, 6, 6>::Zero();

  /* ROS_ERROR("Initialized. M=%ld, N=%ld; corners: %ld, surfs: %ld", M, N, factors_corners.size(), factors_surfs.size()); */

  size_t it = 0;

  const size_t R_card = (double(N) / double(M)) * std::log(1.0 / eps);
  /* const size_t R_card = double(N) / double(M); */
  while (factors_corners_sel.size() + factors_surfs_sel.size() < M) {

    const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::shuffle(indices.begin(), indices.end(), std::default_random_engine(seed));

    bool   best_is_surf = false;
    int    best_idx     = -1;
    size_t best_idx_raw = 0;
    double best_log_det = -std::numeric_limits<double>::max();

    ++it;
    /* ROS_ERROR("[%ld] R=%ld", it, R_card); */

    size_t i        = 0;
    size_t i_finite = 0;
    while (i_finite != R_card) {
      bool   is_surf  = false;
      size_t idx_feat = indices.at(i);

      // Ensure we index surfs from 0
      if (idx_feat >= N_corners) {
        idx_feat -= N_corners;
        is_surf = true;
      }
      /* ROS_ERROR("   [%ld::%ld] idx_feat=%ld, indices.at(i)=%ld, (%s)", it, i, idx_feat, indices.at(i), (is_surf) ? "surf" : "corner"); */

      // If not computed yet: compute residual and jacobian
      auto &point = points.at(idx_feat);
      if (!point.has_data) {

        /* ROS_ERROR("   [%ld::%ld] computing data for the point", it, i); */

        double res[3];

        if (is_surf) {

          factors_surfs.at(idx_feat)->getResidual(Q, t, res);
          factors_surfs.at(idx_feat)->getJacobian(Q, point.jacobian);

        } else {

          factors_corners.at(idx_feat)->getResidual(Q, t, res);
          factors_corners.at(idx_feat)->getJacobian(Q, point.jacobian);
          /* std::cout << point.jacobian << std::endl; */
        }

        point.residual[0] = res[0];
        point.residual[1] = res[1];
        point.residual[2] = res[2];
        /* ROS_ERROR(" [%ld] point %d jacobian:", it, indices.at(i)); */
        /* std::cout << point->jacobian << std::endl; */

        point.inf_mat = point.jacobian.transpose() * point.jacobian;

        /* ROS_ERROR(" [%ld] point %d information matrix:", it, indices.at(i)); */
        /* std::cout << point->inf_mat << std::endl; */
        point.has_data = true;
      }

      // Compute log det and store max
      /* ROS_ERROR("   [%ld::%ld] computing logdet for the point", it, i); */
      const double log_det = std::log((inf_mat + point.inf_mat).determinant());
      /* ROS_ERROR("   [%ld::%ld] det=%.3f, logdet=%.3f", it, i, (inf_mat + point.inf_mat).determinant(), log_det); */

      // TODO: debug why such feature is infinite
      if (std::isfinite(log_det)) {
        /* std::cout << "inf_mat:\n" << std::endl; */
        /* std::cout << inf_mat << std::endl; */
        /* std::cout << "point.inf_mat:\n" << std::endl; */
        /* std::cout << point.inf_mat << std::endl; */
        /* std::cout << "point.inf_mat.det:\n" << std::endl; */
        /* std::cout << point.inf_mat.determinant() << std::endl; */
        if (log_det > best_log_det) {
          best_is_surf = is_surf;
          best_idx     = idx_feat;
          best_idx_raw = indices.at(i);
          best_log_det = log_det;
        }
        i_finite++;
      }
      /* std::cout << inf_mat + point->inf_mat << std::endl; */

      i++;
    }

    inf_mat += points.at(best_idx_raw).inf_mat;

    if (best_is_surf) {
      factors_surfs_sel.push_back(factors_surfs.at(best_idx));
    } else {
      factors_corners_sel.push_back(factors_corners.at(best_idx));
    }

    /* ROS_ERROR("   [%ld] best_idx=%d, best_idx_raw=%ld, (%s) => logdet=%.1f", it, best_idx, best_idx_raw, (best_is_surf) ? "surf" : "corner", best_log_det);
     */

    /* ROS_ERROR(" [%ld] logdet=%.8f", it, std::log(inf_mat.determinant())); */

    // Do not select the same index again
    indices.erase(std::remove(indices.begin(), indices.end(), best_idx_raw), indices.end());
  }

  /* ROS_ERROR("  - selected %ld (%ld corners, %ld surfs)", factors_corners_sel.size() + factors_surfs_sel.size(), factors_corners_sel.size(), */
  /*           factors_surfs_sel.size()); */

  return {factors_corners_sel, factors_surfs_sel};
}
//}

}  // namespace aloam_slam

