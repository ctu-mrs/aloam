#include "aloam_slam/mapping.h"

namespace aloam_slam
{

/*//{ AloamMapping() */
template <class T_pt>
AloamMapping<T_pt>::AloamMapping(const std::shared_ptr<CommonHandlers_t> handlers, const std::shared_ptr<feature_selection::FeatureSelection> feature_selection)
    : _q_w_curr(_parameters), _t_w_curr(_parameters + 4) {

  _handlers          = handlers;
  _scan_period_sec   = 1.0f / handlers->frequency;
  _feature_selection = feature_selection;

  ros::Time::waitForValid();

  _handlers->param_loader->loadParam("mapping/remap_tf", _remap_tf, false);
  _handlers->param_loader->loadParam("mapping/line_resolution", _resolution_line, 0.2f);
  _handlers->param_loader->loadParam("mapping/plane_resolution", _resolution_plane, 0.4f);
  _handlers->param_loader->loadParam("mapping/rate", _mapping_frequency, 5.0f);
  _handlers->param_loader->loadParam("mapping/publish_rate", _map_publish_period, 0.5f);

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
      _cloud_corners.at(i) = boost::make_shared<PC>();
      _cloud_surfs.at(i)   = boost::make_shared<PC>();
    }
  }

  _filter_downsize_corners.setLeafSize(_resolution_line, _resolution_line, _resolution_line);
  _filter_downsize_surfs.setLeafSize(_resolution_plane, _resolution_plane, _resolution_plane);

  _pub_diag            = handlers->nh.advertise<aloam_slam::AloamDiagnostics>("diag_out", 1);
  _pub_laser_cloud_map = handlers->nh.advertise<sensor_msgs::PointCloud2>("map_out", 1);
  _pub_odom_global     = handlers->nh.advertise<nav_msgs::Odometry>("odom_global_out", 1);
  _pub_path            = handlers->nh.advertise<nav_msgs::Path>("path_out", 1);

  _pub_features_edges_selected_  = handlers->nh.advertise<sensor_msgs::PointCloud2>("edges_selected_out", 1);
  _pub_features_planes_selected_ = handlers->nh.advertise<sensor_msgs::PointCloud2>("planes_selected_out", 1);

  if (!handlers->offline_run && _mapping_frequency > 0.0f) {

    _timer_mapping_loop = handlers->nh.createTimer(ros::Rate(_mapping_frequency), &AloamMapping::timerMapping, this, false, true);
  }

  _tf_broadcaster    = std::make_shared<tf2_ros::TransformBroadcaster>();
  _srv_reset_mapping = handlers->nh.advertiseService("srv_reset_mapping_in", &AloamMapping::callbackResetMapping, this);

  _time_last_map_publish = ros::Time::now();
}
/*//}*/

/*//{ setData() */
template <class T_pt>
void AloamMapping<T_pt>::setData(const std::shared_ptr<MappingData<T_pt>> data) {

  const mrs_lib::Routine profiler_routine = _handlers->profiler->createRoutine("aloamMappingSetData");

  {
    std::unique_lock lock(_mutex_mapping_data);
    _has_new_data = true;
    _mapping_data = data;
  }

  _cv_mapping_data.notify_all();
}
/*//}*/

/*//{ computeMapping() */
template <class T_pt>
bool AloamMapping<T_pt>::computeMapping(geometry_msgs::TransformStamped &tf_msg_out, aloam_slam::AloamDiagnostics &diag_msg_out,
                                        const std::shared_ptr<MappingCloudManagersOut_t<T_pt>> managers_out,
                                        const std::shared_ptr<aloam_slam::FeaturesSignature>   sign_msg_out) {

  const bool dbg = sign_msg_out != nullptr;

  if (!is_initialized) {
    return false;
  }

  std::unique_ptr<mrs_lib::ScopeTimer> timer;

  /*//{ Load latest odometry data (blocks for _cv_mapping_data and timeouts if blocking takes longer than 1.0/_mapping_frequency) */
  ros::Time     time_aloam_odometry;
  tf::Transform aloam_odometry;

  feature_selection::FSCloudManagerPtr<T_pt> manager_corners_extracted;
  feature_selection::FSCloudManagerPtr<T_pt> manager_surfs_extracted;

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

    timer = std::make_unique<mrs_lib::ScopeTimer>("ALOAM::timerMapping", _handlers->scope_timer_logger, _handlers->enable_scope_timer);

    time_aloam_odometry       = _mapping_data->stamp_ros;
    aloam_odometry            = _mapping_data->odometry;
    manager_corners_extracted = _mapping_data->manager_corners;
    manager_surfs_extracted   = _mapping_data->manager_surfs;

    diag_msg_out.feature_extraction = _mapping_data->diagnostics_fe;
    diag_msg_out.odometry           = _mapping_data->diagnostics_odometry;

    _has_new_data = false;
  }
  /*//}*/

  /* timer->checkpoint("loading data"); */
  diag_msg_out.mapping.ms_loading_data = timer->getLifetime();

  // | -------------- Downsample extracted features ------------- |
  manager_corners_extracted = VoxelFilter(manager_corners_extracted, _resolution_line).filter();
  manager_surfs_extracted   = VoxelFilter(manager_surfs_extracted, _resolution_plane).filter();

  /*//{ Select features from the extracted features */

  std::shared_ptr<feature_selection::DebugData_t> dbg_edges;
  std::shared_ptr<feature_selection::DebugData_t> dbg_planes;
  if (dbg) {
    dbg_edges  = std::make_shared<feature_selection::DebugData_t>();
    dbg_planes = std::make_shared<feature_selection::DebugData_t>();
  }

  bool                                       fs_used                  = false;
  feature_selection::FSCloudManagerPtr<T_pt> manager_corners_selected = nullptr;
  feature_selection::FSCloudManagerPtr<T_pt> manager_surfs_selected   = nullptr;
  if (_feature_selection->enabled()) {

    LidarFeatureSelectionEdgePlaneIO_t selection_io;

    selection_io.cloud                    = manager_corners_extracted->getCloudPtr();
    selection_io.indices_edges_extracted  = manager_corners_extracted->getIndicesPtr();
    selection_io.indices_planes_extracted = manager_surfs_extracted->getIndicesPtr();

    diag_msg_out.feature_selection.stamp = time_aloam_odometry;

    if (dbg) {
      selection_io.dbg_edges  = dbg_edges;
      selection_io.dbg_planes = dbg_planes;
    }

    selectFeatures(selection_io, diag_msg_out.feature_selection);

    if (selection_io.selection_success) {

      fs_used                  = true;
      manager_corners_selected = std::make_shared<feature_selection::FSCloudManager<T_pt>>(selection_io.cloud, selection_io.indices_edges_selected);
      manager_surfs_selected   = std::make_shared<feature_selection::FSCloudManager<T_pt>>(selection_io.cloud, selection_io.indices_planes_selected);

      if (managers_out) {
        managers_out->man_edges_selected  = manager_corners_selected;
        managers_out->man_planes_selected = manager_surfs_selected;
      }

      publishCloud(_pub_features_edges_selected_, manager_corners_selected);
      publishCloud(_pub_features_planes_selected_, manager_surfs_selected);
    }

    /* timer->checkpoint("selecting features"); */
    diag_msg_out.feature_selection.ms_total = timer->getLifetime() - diag_msg_out.mapping.ms_loading_data;

  } else {

    diag_msg_out.feature_selection.ms_total = 0.0f;
  }
  /*//}*/

  const mrs_lib::Routine profiler_routine = _handlers->profiler->createRoutine("timerMapping");

  tf::vectorTFToEigen(aloam_odometry.getOrigin(), _t_wodom_curr);
  tf::quaternionTFToEigen(aloam_odometry.getRotation(), _q_wodom_curr);

  const PC_pt map_features_corners = boost::make_shared<PC>();
  const PC_pt map_features_surfs   = boost::make_shared<PC>();

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
          int         i             = _cloud_width - 1;
          const PC_pt local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          const PC_pt local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          for (; i >= 1; i--) {
            _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_corners.at(i - 1 + _cloud_width * j + _cloud_width * _cloud_height * k);
            _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_surfs.at(i - 1 + _cloud_width * j + _cloud_width * _cloud_height * k);
          }
          _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) = local_corners;
          _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k)   = local_surfs;
        }
      }

      center_cube_I++;
      _cloud_center_width++;
    }

    while (center_cube_I >= _cloud_width - 3) {
      for (int j = 0; j < _cloud_height; j++) {
        for (int k = 0; k < _cloud_depth; k++) {
          int         i             = 0;
          const PC_pt local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          const PC_pt local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          for (; i < _cloud_width - 1; i++) {
            _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_corners.at(i + 1 + _cloud_width * j + _cloud_width * _cloud_height * k);
            _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_surfs.at(i + 1 + _cloud_width * j + _cloud_width * _cloud_height * k);
          }
          _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) = local_corners;
          _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k)   = local_surfs;
        }
      }

      center_cube_I--;
      _cloud_center_width--;
    }

    while (center_cube_J < 3) {
      for (int i = 0; i < _cloud_width; i++) {
        for (int k = 0; k < _cloud_depth; k++) {
          int         j             = _cloud_height - 1;
          const PC_pt local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          const PC_pt local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          for (; j >= 1; j--) {
            _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_corners.at(i + _cloud_width * (j - 1) + _cloud_width * _cloud_height * k);
            _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_surfs.at(i + _cloud_width * (j - 1) + _cloud_width * _cloud_height * k);
          }
          _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) = local_corners;
          _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k)   = local_surfs;
        }
      }

      center_cube_J++;
      _cloud_center_height++;
    }

    while (center_cube_J >= _cloud_height - 3) {
      for (int i = 0; i < _cloud_width; i++) {
        for (int k = 0; k < _cloud_depth; k++) {
          int         j             = 0;
          const PC_pt local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          const PC_pt local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          for (; j < _cloud_height - 1; j++) {
            _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_corners.at(i + _cloud_width * (j + 1) + _cloud_width * _cloud_height * k);
            _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_surfs.at(i + _cloud_width * (j + 1) + _cloud_width * _cloud_height * k);
          }
          _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) = local_corners;
          _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k)   = local_surfs;
        }
      }

      center_cube_J--;
      _cloud_center_height--;
    }

    while (center_cube_K < 3) {
      for (int i = 0; i < _cloud_width; i++) {
        for (int j = 0; j < _cloud_height; j++) {
          int         k             = _cloud_depth - 1;
          const PC_pt local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          const PC_pt local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          for (; k >= 1; k--) {
            _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * (k - 1));
            _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * (k - 1));
          }
          _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) = local_corners;
          _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k)   = local_surfs;
        }
      }

      center_cube_K++;
      _cloud_center_depth++;
    }

    while (center_cube_K >= _cloud_depth - 3) {
      for (int i = 0; i < _cloud_width; i++) {
        for (int j = 0; j < _cloud_height; j++) {
          int         k             = 0;
          const PC_pt local_corners = _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          const PC_pt local_surfs   = _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k);
          for (; k < _cloud_depth - 1; k++) {
            _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * (k + 1));
            _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) =
                _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * (k + 1));
          }
          _cloud_corners.at(i + _cloud_width * j + _cloud_width * _cloud_height * k) = local_corners;
          _cloud_surfs.at(i + _cloud_width * j + _cloud_width * _cloud_height * k)   = local_surfs;
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

    for (const auto i : cloud_valid_indices) {
      *map_features_corners += *_cloud_corners.at(i);
      *map_features_surfs += *_cloud_surfs.at(i);
    }
  }

  /* timer->checkpoint("generating local feature map"); */
  /*//}*/

  /*//{ Find factors and optimize */

  /* if (manager_corners_selected && manager_surfs_selected) { */

  /*   _filter_downsize_corners.setInputCloud(manager_corners_selected->extractCloudByIndices()); */
  /*   _filter_downsize_surfs.setInputCloud(manager_surfs_selected->extractCloudByIndices()); */

  /* } else { */

  /*   _filter_downsize_corners.setInputCloud(manager_corners_extracted->extractCloudByIndices()); */
  /*   _filter_downsize_surfs.setInputCloud(manager_surfs_extracted->extractCloudByIndices()); */
  /* } */
  /* _filter_downsize_surfs.setIndices( */

  /* const PC_pt features_corners_opt_stack = boost::make_shared<PC>(); */
  /* const PC_pt features_surfs_opt_stack   = boost::make_shared<PC>(); */
  /* _filter_downsize_corners.filter(*features_corners_opt_stack); */
  /* _filter_downsize_surfs.filter(*features_surfs_opt_stack); */

  /* timer->checkpoint("voxelizing input features"); */

  /* printf("map prepare time %f ms\n", t_shift.toc()); */
  /* printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum); */
  if (map_features_corners->points.size() > 10 && map_features_surfs->points.size() > 50) {

    const feature_selection::FSCloudManagerPtr<T_pt> manager_corners =
        manager_corners_selected == nullptr ? manager_corners_extracted : manager_corners_selected;
    const feature_selection::FSCloudManagerPtr<T_pt> manager_surfs = manager_surfs_selected == nullptr ? manager_surfs_extracted : manager_surfs_selected;

    // | ---------------------- Build KD-Tree --------------------- |
    const std::shared_ptr<pcl::KdTreeFLANN<T_pt>> kdtree_map_corners = std::make_shared<pcl::KdTreeFLANN<T_pt>>(true);
    const std::shared_ptr<pcl::KdTreeFLANN<T_pt>> kdtree_map_surfs   = std::make_shared<pcl::KdTreeFLANN<T_pt>>(true);

    kdtree_map_corners->setInputCloud(map_features_corners);
    kdtree_map_surfs->setInputCloud(map_features_surfs);

    // | ------------ Extract features for optimization ----------- |
    const PC_pt features_corners_opt_stack = manager_corners->extractCloudByIndices();
    const PC_pt features_surfs_opt_stack   = manager_surfs->extractCloudByIndices();

    /* timer->checkpoint("computing KDTree from input features"); */

    // | ------------ Setup I/O for finding the factors ----------- |
    FactorizationIO_t factorization_edges;
    factorization_edges.kdtree     = kdtree_map_corners;
    factorization_edges.features   = features_corners_opt_stack;
    factorization_edges.map_points = map_features_corners;

    FactorizationIO_t factorization_planes;
    factorization_planes.kdtree     = kdtree_map_surfs;
    factorization_planes.features   = features_surfs_opt_stack;
    factorization_planes.map_points = map_features_surfs;

    factorization_edges.indices_features  = manager_corners->getIndicesPtr();
    factorization_planes.indices_features = manager_surfs->getIndicesPtr();

    // | ---------------- Optimize in 2 iterations ---------------- |
    for (int iterCount = 0; iterCount < 2; iterCount++) {

      /* timer->checkpoint("iter " + std::to_string(iterCount) + " init"); */

      ceres::LossFunction          *loss_function      = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
      ceres::Problem::Options       problem_options;

      ceres::Problem problem(problem_options);
      problem.AddParameterBlock(_parameters, 4, q_parameterization);
      problem.AddParameterBlock(_parameters + 4, 3);

      /* mrs_lib::ScopeTimer timer("finding factors", nullptr, true); */
      const auto &factors_edges  = findEdgeFactors(factorization_edges, dbg && iterCount == 0);
      const auto &factors_planes = findPlaneFactors(factorization_planes, dbg && iterCount == 0);
      /* timer.checkpoint("found"); */

      for (const auto &factor : factors_edges) {
        problem.AddResidualBlock(LidarEdgeFactor::Create(factor), loss_function, _parameters, _parameters + 4);
      }
      for (const auto &factor : factors_planes) {
        problem.AddResidualBlock(LidarPlaneFactor::Create(factor), loss_function, _parameters, _parameters + 4);
      }
      /* timer->checkpoint("iter " + std::to_string(iterCount) + " creating residuals"); */
      /* ROS_ERROR("After adding blocks to problem."); */

      // Get eigenvalues to measure problem degradation //{
      /* TicToc t_eigenvalues; */

      if (dbg && iterCount == 0) {

        sign_msg_out->stamp = time_aloam_odometry;

        // | -- Compute eigenvalues of the problem with SEL features -- |

        // Get Jacobian
        {
          const Eigen::MatrixXd J = parseJacobianFromCeresProblem(problem);
          // the order from CERES should be theta_x, theta_y, theta_z, x, y, z
          const auto              &eigen_values_sel = getEigenValuesOrdered(J);
          const std::vector<float> values_flt       = {float(eigen_values_sel[3]), float(eigen_values_sel[4]), float(eigen_values_sel[5]),
                                                       float(eigen_values_sel[0]), float(eigen_values_sel[1]), float(eigen_values_sel[2])};
          if (fs_used) {
            sign_msg_out->problem_eigenvalues_w_fs = values_flt;
          } else {
            sign_msg_out->problem_eigenvalues_wo_fs = values_flt;
          }
        }

        // | ---------- Find factors of removed features --------- |
        FactorizationIO_t fact_edge_rem;
        FactorizationIO_t fact_plane_rem;

        // Setup inputs to factorization
        {
          // Setup removed feature clouds
          const feature_extraction::indices_ptr_t indices_corners_rem = std::make_shared<feature_extraction::indices_t>();
          const feature_extraction::indices_ptr_t indices_surfs_rem   = std::make_shared<feature_extraction::indices_t>();
          indices_corners_rem->reserve(dbg_edges->data_rem.size());
          for (const auto &dato : dbg_edges->data_rem) {
            feature_extraction::index_azimuth_t idx_az;
            idx_az.val     = dato.cloud_idx;
            idx_az.azimuth = dato.value;
            indices_corners_rem->push_back(idx_az);
          }
          indices_surfs_rem->reserve(dbg_planes->data_rem.size());
          for (const auto &dato : dbg_planes->data_rem) {
            feature_extraction::index_azimuth_t idx_az;
            idx_az.val     = dato.cloud_idx;
            idx_az.azimuth = dato.value;
            indices_surfs_rem->push_back(idx_az);
          }

          // Convert to point cloud
          const PC_pt features_corners_rem =
              feature_selection::FSCloudManager<T_pt>(manager_corners_extracted->getCloudPtr(), indices_corners_rem).extractCloudByIndices();
          const PC_pt features_surfs_rem =
              feature_selection::FSCloudManager<T_pt>(manager_surfs_extracted->getCloudPtr(), indices_surfs_rem).extractCloudByIndices();

          // Setup factorization structs
          fact_edge_rem.kdtree           = kdtree_map_corners;
          fact_edge_rem.features         = features_corners_rem;
          fact_edge_rem.map_points       = map_features_corners;
          fact_edge_rem.indices_features = indices_corners_rem;

          fact_plane_rem.kdtree           = kdtree_map_surfs;
          fact_plane_rem.features         = features_surfs_rem;
          fact_plane_rem.map_points       = map_features_surfs;
          fact_plane_rem.indices_features = indices_surfs_rem;
        }

        // Find factors for both types
        const auto &factors_edges_rem  = findEdgeFactors(fact_edge_rem, true);
        const auto &factors_planes_rem = findPlaneFactors(fact_plane_rem, true);

        // If we used FS to remove some residuals
        /* if (fs_used) { */

        /*   // Define the problem with all the residuals */
        /*   ceres::LossFunction          *loss_function_all      = new ceres::HuberLoss(0.1); */
        /*   ceres::LocalParameterization *q_parameterization_all = new ceres::EigenQuaternionParameterization(); */
        /*   ceres::Problem::Options       problem_options_all; */


        /*   // Copy current estimate */
        /*   double parameters[7]; */
        /*   parameters[0] = _parameters[0]; */
        /*   parameters[1] = _parameters[1]; */
        /*   parameters[2] = _parameters[2]; */
        /*   parameters[3] = _parameters[3]; */
        /*   parameters[4] = _parameters[4]; */
        /*   parameters[5] = _parameters[5]; */
        /*   parameters[6] = _parameters[6]; */
        /*   Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters); */
        /*   Eigen::Map<Eigen::Vector3d>    t_w_curr(parameters + 4); */

        /*   // Add parameter blocks */
        /*   ceres::Problem problem_all(problem_options_all); */
        /*   problem_all.AddParameterBlock(parameters, 4, q_parameterization_all); */
        /*   problem_all.AddParameterBlock(parameters + 4, 3); */

        /*   // Add selected factors */
        /*   for (const auto &factor : factors_edges) { */
        /*     problem_all.AddResidualBlock(LidarEdgeFactor::Create(factor), loss_function_all, parameters, parameters + 4); */
        /*   } */
        /*   for (const auto &factor : factors_planes) { */
        /*     problem_all.AddResidualBlock(LidarPlaneFactor::Create(factor), loss_function_all, parameters, parameters + 4); */
        /*   } */

        /*   // Add removed factors */
        /*   for (const auto &factor : factors_edges_rem) { */
        /*     problem_all.AddResidualBlock(LidarEdgeFactor::Create(factor), loss_function_all, parameters, parameters + 4); */
        /*   } */
        /*   for (const auto &factor : factors_planes_rem) { */
        /*     problem_all.AddResidualBlock(LidarPlaneFactor::Create(factor), loss_function_all, parameters, parameters + 4); */
        /*   } */

        /*   // | -- Compute eigenvalues of the problem with ALL features -- | */
        /*   { */
        /*     const Eigen::MatrixXd J                = parseJacobianFromCeresProblem(problem_all); */
        /*     const auto           &eigen_values_all = getEigenValuesOrdered(J); */
        /*     // the order from CERES should be theta_x, theta_y, theta_z, x, y, z */
        /*     sign_msg_out->problem_eigenvalues_wo_fs = {float(eigen_values_all[3]), float(eigen_values_all[4]), float(eigen_values_all[5]), */
        /*                                                float(eigen_values_all[0]), float(eigen_values_all[1]), float(eigen_values_all[2])}; */
        /*   } */
        /* } */

        // | ------- Store factors and other info in output msg ------- |
        setSignatureVecMsg(dbg_edges, factorization_edges.associated_idx_eigenvalues, fact_edge_rem.associated_idx_eigenvalues, sign_msg_out->corners);
        setSignatureVecMsg(dbg_planes, factorization_planes.associated_idx_eigenvalues, fact_plane_rem.associated_idx_eigenvalues, sign_msg_out->planes);
      }
      /*//}*/

      ceres::Solver::Options options;
      options.linear_solver_type                = ceres::DENSE_QR;
      options.max_num_iterations                = 4;
      options.minimizer_progress_to_stdout      = false;
      options.check_gradients                   = false;
      options.gradient_check_relative_precision = 1e-4;
      ceres::Solver::Summary summary;

      /* ROS_ERROR("Before solving"); */
      ceres::Solve(options, &problem, &summary);
      /* ROS_ERROR("After solving"); */
      /* timer->checkpoint("iter " + std::to_string(iterCount) + " computing solution"); */
    }
  } else {
    ROS_WARN("[AloamMapping] Not enough map correspondences. Skipping mapping frame.");
  }
  /*//}*/

  /* timer->checkpoint("solving problem"); */
  diag_msg_out.mapping.ms_associating_features = timer->getLifetime() - diag_msg_out.mapping.ms_loading_data - diag_msg_out.feature_selection.ms_total;

  {
    std::scoped_lock lock(_mutex_cloud_features);

    transformUpdate();

    // Store all extracted corners to map
    const auto &corners_extracted = manager_corners_extracted->getCloudPtr();
    for (const auto &idx : *manager_corners_extracted->getIndicesPtr()) {

      const auto &point_ori = corners_extracted->at(idx.val);
      const auto &point_sel = pointAssociateToMap(point_ori);

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

    // Store all extracted surfs to map
    const auto &surfs_extracted = manager_surfs_extracted->getCloudPtr();
    for (const auto &idx : *manager_surfs_extracted->getIndicesPtr()) {

      const auto &point_ori = surfs_extracted->at(idx.val);
      const auto &point_sel = pointAssociateToMap(point_ori);

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

    for (const auto ind : cloud_valid_indices) {

      if (!_cloud_corners.at(ind)->empty()) {
        _filter_downsize_corners.setInputCloud(_cloud_corners.at(ind));
        _filter_downsize_corners.filter(*_cloud_corners.at(ind));

        /* const VoxelFilter filter_downsize_corners = VoxelFilter(_cloud_corners.at(ind), nullptr, _resolution_line); */
        /* _cloud_corners.at(ind)                    = filter_downsize_corners.filter(); */
      }

      if (!_cloud_surfs.at(ind)->empty()) {
        _filter_downsize_surfs.setInputCloud(_cloud_surfs.at(ind));
        _filter_downsize_surfs.filter(*_cloud_surfs.at(ind));
        /* const VoxelFilter filter_downsize_surfs = VoxelFilter(_cloud_surfs.at(ind), nullptr, _resolution_plane); */
        /* _cloud_surfs.at(ind)                    = filter_downsize_surfs.filter(); */
      }
    }
  }

  /* timer->checkpoint("adding output-transformed features to map"); */

  diag_msg_out.mapping.ms_adding_features_to_map =
      timer->getLifetime() - diag_msg_out.mapping.ms_loading_data - diag_msg_out.mapping.ms_associating_features - diag_msg_out.feature_selection.ms_total;

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
    PC map_pcl;
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

  diag_msg_out.frame            = ++_frame_count;
  diag_msg_out.mapping.ms_total = timer->getLifetime() - diag_msg_out.feature_selection.ms_total;
  diag_msg_out.ms_total =
      diag_msg_out.feature_extraction.ms_total + diag_msg_out.odometry.ms_total + diag_msg_out.feature_selection.ms_total + diag_msg_out.mapping.ms_total;
  diag_msg_out.data_stamp = time_aloam_odometry;

  // Publish diagnostics
  if (_pub_diag.getNumSubscribers() > 0) {

    try {
      _pub_diag.publish(diag_msg_out);
    }
    catch (...) {
      ROS_ERROR("[AloamMapping]: Exception caught during publishing topic: %s.", _pub_diag.getTopic().c_str());
    }
  }

  ROS_INFO_THROTTLE(1.0, "[Aloam] Run time: %.1f ms (FE: %.0f | FS: %.0f | O: %.0f | M: %.0f)", diag_msg_out.ms_total, diag_msg_out.feature_extraction.ms_total,
                    diag_msg_out.feature_selection.ms_total, diag_msg_out.odometry.ms_total, diag_msg_out.mapping.ms_total);

  _runtime_total += diag_msg_out.ms_total;
  ROS_INFO_THROTTLE(5.0, "[Aloam] Average run time: %.1f ms", _runtime_total / diag_msg_out.frame);

  return true;
}
/*//}*/

/*//{ timerMapping() */
template <class T_pt>
void AloamMapping<T_pt>::timerMapping([[maybe_unused]] const ros::TimerEvent &event) {
  geometry_msgs::TransformStamped tf_msg;
  aloam_slam::AloamDiagnostics    diag_msg;

  computeMapping(tf_msg, diag_msg, nullptr, nullptr);
}
/*//}*/

/*//{ transformAssociateToMap() */
template <class T_pt>
void AloamMapping<T_pt>::transformAssociateToMap() {
  _q_w_curr = _q_wmap_wodom * _q_wodom_curr;
  _t_w_curr = _q_wmap_wodom * _t_wodom_curr + _t_wmap_wodom;
}
/*//}*/

/*//{ transformUpdate() */
template <class T_pt>
void AloamMapping<T_pt>::transformUpdate() {
  _q_wmap_wodom = _q_w_curr * _q_wodom_curr.inverse();
  _t_wmap_wodom = _t_w_curr - _q_wmap_wodom * _t_wodom_curr;
}
/*//}*/

/*//{ pointAssociateToMap() */
template <class T_pt>
T_pt AloamMapping<T_pt>::pointAssociateToMap(const T_pt &pi) const {

  const Eigen::Vector3d point_curr(pi.x, pi.y, pi.z);
  const Eigen::Vector3d point_w = _q_w_curr * point_curr + _t_w_curr;

  T_pt po = pi;
  po.x    = float(point_w.x());
  po.y    = float(point_w.y());
  po.z    = float(point_w.z());
  return po;
}
/*//}*/

/*//{ callbackResetMapping() */
template <class T_pt>
bool AloamMapping<T_pt>::callbackResetMapping([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  std::scoped_lock lock(_mutex_cloud_features);
  _cloud_corners.resize(_cloud_volume);
  _cloud_surfs.resize(_cloud_volume);
  for (int i = 0; i < _cloud_volume; i++) {
    _cloud_corners.at(i) = boost::make_shared<PC>();
    _cloud_surfs.at(i)   = boost::make_shared<PC>();
  }
  ROS_INFO("[AloamMapping] Reset: map features were cleared.");

  res.success = true;
  res.message = "AloamMapping features were cleared.";
  return true;
}
/*//}*/

/* setTransform() //{ */

template <class T_pt>
void AloamMapping<T_pt>::setTransform(const Eigen::Vector3d &t, const Eigen::Quaterniond &q, const ros::Time &stamp) {
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

/* findEdgeFactors() //{ */
template <class T_pt>
std::vector<LidarEdgeFactor *> AloamMapping<T_pt>::findEdgeFactors(FactorizationIO_t &factorization, const bool store_dbg_info) {

  std::vector<LidarEdgeFactor *> factors;

  double Q[4];

  if (store_dbg_info) {
    Q[0] = _q_w_curr.x();
    Q[1] = _q_w_curr.y();
    Q[2] = _q_w_curr.z();
    Q[3] = _q_w_curr.w();
  }

  for (size_t i = 0; i < factorization.features->size(); i++) {
    const auto &point_ori = factorization.features->at(i);
    const auto &point_sel = pointAssociateToMap(point_ori);
    const auto &idx       = factorization.indices_features->at(i);

    std::vector<int>   point_search_indices;
    std::vector<float> point_search_sq_dist;
    factorization.kdtree->nearestKSearch(point_sel, 5, point_search_indices, point_search_sq_dist);

    // If the correspondence is found
    if (point_search_sq_dist.at(4) < 1.0) {
      Eigen::Vector3d              center(0, 0, 0);
      std::vector<Eigen::Vector3d> nearCorners;
      nearCorners.reserve(5);
      for (int j = 0; j < 5; j++) {
        const auto &point = factorization.map_points->points.at(point_search_indices.at(j));

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

        // | ---------------------- Create factor --------------------- |
        LidarEdgeFactor *factor = new LidarEdgeFactor(curr_point, point_a, point_b, 1.0);
        factors.push_back(factor);

        // | --- Perform eigendecomposition of the J^T * J matrix -- |
        if (store_dbg_info) {

          // Find Jacobian for 3-dim state (xyz) and 6-DoF variable (xyzrpy)
          Eigen::Matrix<double, 3, 6> J;
          factor->getJacobian(Q, J);
          const auto &eigen_values = getEigenValuesOrdered(J);

          // Store results
          factorization.associated_idx_eigenvalues.push_back({idx.val, eigen_values});
        }
      }
    }
  }

  return factors;
}
//}

/* findPlaneFactors() //{ */
template <class T_pt>
std::vector<LidarPlaneFactor *> AloamMapping<T_pt>::findPlaneFactors(FactorizationIO_t &factorization, const bool store_dbg_info) {

  std::vector<LidarPlaneFactor *> factors;

  double Q[4];

  if (store_dbg_info) {
    Q[0] = _q_w_curr.x();
    Q[1] = _q_w_curr.y();
    Q[2] = _q_w_curr.z();
    Q[3] = _q_w_curr.w();
  }

  for (size_t i = 0; i < factorization.features->size(); i++) {
    const auto &point_ori = factorization.features->at(i);
    const auto &point_sel = pointAssociateToMap(point_ori);
    const auto &idx       = factorization.indices_features->at(i);

    std::vector<int>   point_search_indices;
    std::vector<float> point_search_sq_dist;
    factorization.kdtree->nearestKSearch(point_sel, 5, point_search_indices, point_search_sq_dist);

    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
    if (point_search_sq_dist.at(4) < 1.0) {
      for (int j = 0; j < 5; j++) {
        const auto &point = factorization.map_points->points.at(point_search_indices.at(j));
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
        const auto &point = factorization.map_points->points.at(point_search_indices.at(j));
        // if OX * n > 0.2, then plane is not fit well
        if (std::fabs(n(0) * point.x + n(1) * point.y + n(2) * point.z + d) > 0.2) {
          planeValid = false;
          break;
        }
      }
      if (planeValid) {
        const Eigen::Vector3d curr_point(point_ori.x, point_ori.y, point_ori.z);

        LidarPlaneFactor *factor = new LidarPlaneFactor(curr_point, n, d);
        factors.push_back(factor);

        // | --- Perform eigendecomposition of the J^T * J matrix -- |
        if (store_dbg_info) {

          // Find Jacobian for 3-dim state (xyz) and 6-DoF variable (xyzrpy)
          Eigen::Matrix<double, 3, 6> J;
          factor->getJacobian(Q, J);
          const auto &eigen_values = getEigenValuesOrdered(J);

          // Store results
          factorization.associated_idx_eigenvalues.push_back({idx.val, eigen_values});
        }
      }
    }
  }

  return factors;
}
//}

/*//{ selectFeatures() */
template <class T_pt>
void AloamMapping<T_pt>::selectFeatures(LidarFeatureSelectionEdgePlaneIO_t &selection_io, feature_selection::FSDiagnostics &msg_diag) {

  if (!_feature_selection) {
    selection_io.selection_enabled = false;
    return;
  }

  selection_io.selection_enabled = true;

  if (selection_io.selection_enabled) {

    std::vector<feature_selection::SelectionIO_t<T_pt>> selection_vec(2);

    selection_vec.at(0).feature_name_in = "edges";
    selection_vec.at(0).cloud_manager_ptr_in =
        std::make_shared<feature_selection::FSCloudManager<T_pt>>(selection_io.cloud, selection_io.indices_edges_extracted);
    selection_vec.at(0).dbg_data = selection_io.dbg_edges;

    selection_vec.at(1).feature_name_in = "planes";
    selection_vec.at(1).cloud_manager_ptr_in =
        std::make_shared<feature_selection::FSCloudManager<T_pt>>(selection_io.cloud, selection_io.indices_planes_extracted);
    selection_vec.at(1).dbg_data = selection_io.dbg_planes;

    selection_io.selection_success = _feature_selection->select(selection_vec, msg_diag);

    if (selection_io.selection_success) {
      selection_io.indices_edges_selected  = selection_vec.at(0).indices_out;
      selection_io.indices_planes_selected = selection_vec.at(1).indices_out;
    }
  }
}
/*//}*/

/*//{ eigenDecomposition() */
template <class T_pt>
std::pair<Eigen::VectorXd, std::vector<Eigen::VectorXd>> AloamMapping<T_pt>::eigenDecomposition(const Eigen::MatrixXd &mat) {

  // Compute
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(mat);

  // Get real values
  const Eigen::VectorXd lambda = solver.eigenvalues().real();
  const Eigen::MatrixXd v_mat  = solver.eigenvectors().real();

  Eigen::VectorXd              eigen_values;
  std::vector<Eigen::VectorXd> eigen_vectors;

  eigen_values.resize(v_mat.cols());
  eigen_vectors.resize(v_mat.cols());

  // Order vectors/values per input order
  for (int j = 0; j < v_mat.cols(); j++) {

    int    max_i     = 0;
    double max_i_val = 0.0;

    for (int i = 0; i < v_mat.rows(); i++) {
      const double val = std::fabs(v_mat(i, j));
      if (val > max_i_val) {
        max_i     = i;
        max_i_val = val;
      }
    }

    eigen_values[j]  = lambda[max_i];
    eigen_vectors[j] = v_mat.row(max_i);
  }

  return {eigen_values, eigen_vectors};
}
/*//}*/

/*//{ getEigenValuesOrdered() */
template <class T_pt>
std::vector<double> AloamMapping<T_pt>::getEigenValuesOrdered(const Eigen::MatrixXd &jacobian) {

  const Eigen::Matrix<double, 6, 6> inf_mat = jacobian.transpose() * jacobian;

  std::vector<double> eigen_values;
  eigen_values.reserve(6);

  // Eigendecomposition of the upper left 3x3 sumatrix (translation part)
  {
    const Eigen::Matrix<double, 3, 3> submat = inf_mat.block(0, 0, 3, 3);
    const auto &[vals, _]                    = eigenDecomposition(submat);
    for (int i = 0; i < 3; i++) {
      eigen_values.push_back(vals[i]);
    }
  }

  // Eigendecomposition of the bottom right 3x3 sumatrix (rotation part)
  {
    const Eigen::Matrix<double, 3, 3> submat = inf_mat.block(3, 3, 3, 3);
    const auto &[vals, _]                    = eigenDecomposition(submat);
    for (int i = 0; i < 3; i++) {
      eigen_values.push_back(vals[i]);
    }
  }

  return eigen_values;
}
/*//}*/

/*//{ setSignatureVecMsg() */
template <class T_pt>
void AloamMapping<T_pt>::setSignatureVecMsg(const std::shared_ptr<feature_selection::DebugData_t> dbg_data, const IdxEigenvaluesPairs &idx_eigvals_sel,
                                            const IdxEigenvaluesPairs &idx_eigvals_rem, std::vector<aloam_slam::Signature> &msg) {

  const auto &data_sel = dbg_data->data_sel;
  const auto &data_rem = dbg_data->data_rem;

  msg.reserve(data_rem.size() + data_sel.size());

  // | -------------- Fill selected features first -------------- |
  for (const auto &dato : dbg_data->data_sel) {

    aloam_slam::Signature sig;

    sig.cloud_idx   = dato.cloud_idx;
    sig.segment_idx = dato.segment_idx;

    sig.selected   = true;
    sig.standalone = dato.standalone;
    sig.value      = dato.value;

    // Find if the index has been associated to the map (and hence used in optimization)
    const auto it = std::find_if(idx_eigvals_sel.cbegin(), idx_eigvals_sel.cend(),
                                 [&](const std::pair<unsigned int, std::vector<double>> &x) { return x.first == dato.cloud_idx; });

    if (it != idx_eigvals_sel.cend()) {
      sig.associated = true;

      // Convert eigen values to float
      sig.eigenvalues.reserve(it->second.size());
      for (const double v : it->second) {
        sig.eigenvalues.push_back(float(v));
      }

    } else {
      sig.associated = false;
    }

    msg.push_back(sig);
  }

  // | -------------- Fill removed features second -------------- |
  for (const auto &dato : dbg_data->data_rem) {

    aloam_slam::Signature sig;

    sig.cloud_idx   = dato.cloud_idx;
    sig.segment_idx = dato.segment_idx;

    sig.selected   = false;
    sig.standalone = dato.standalone;
    sig.value      = dato.value;

    // Find if the index has been associated to the map (and hence used in optimization)
    const auto it = std::find_if(idx_eigvals_rem.cbegin(), idx_eigvals_rem.cend(),
                                 [&](const std::pair<unsigned int, std::vector<double>> &x) { return x.first == dato.cloud_idx; });

    if (it != idx_eigvals_rem.cend()) {
      sig.associated = true;

      // Convert eigen values to float
      sig.eigenvalues.reserve(it->second.size());
      for (const double v : it->second) {
        sig.eigenvalues.push_back(float(v));
      }

    } else {
      sig.associated = false;
    }

    msg.push_back(sig);
  }
}
/*//}*/

/*//{ parseJacobianFromCeresProblem() */
template <class T_pt>
Eigen::MatrixXd AloamMapping<T_pt>::parseJacobianFromCeresProblem(ceres::Problem &problem) {

  Eigen::MatrixXd J;

  ceres::CRSMatrix J_ceres;
  problem.Evaluate(ceres::Problem::EvaluateOptions(), nullptr, nullptr, nullptr, &J_ceres);

  // Convert it to eigen matrix
  J.resize(J_ceres.num_rows, J_ceres.num_cols);
  J.setZero();

  for (int row = 0; row < J_ceres.num_rows; ++row) {
    const int start = J_ceres.rows.at(row);
    const int end   = J_ceres.rows.at(row + 1) - 1;

    for (int i = start; i <= end; i++) {
      const int    col   = J_ceres.cols.at(i);
      const double value = J_ceres.values.at(i);

      J(row, col) = value;
    }
  }

  return J;
}
/*//}*/

// | -------------------- VoxelFilter class ------------------- |

/*//{ VoxelFilter() */

// Method taken from: https://github.com/PointCloudLibrary/pcl/issues/2211
template <class T_pt>
VoxelFilter<T_pt>::VoxelFilter(const feature_selection::FSCloudManagerPtr<T_pt> manager, const float resolution) {

  const auto cloud = manager->getCloudPtr();

  if (cloud->empty()) {
    _empty = true;
    return;
  }

  _inverse_leaf_size = Eigen::Array3f::Ones() / Eigen::Vector3f(resolution, resolution, resolution).array();

  // Get the minimum and maximum dimensions
  Eigen::Vector4f min_p, max_p;
  pcl::getMinMax3D(*cloud, min_p, max_p);

  // Check that the leaf size is not too small, given the size of the data
  const uint64_t dx = static_cast<uint64_t>((max_p[0] - min_p[0]) * _inverse_leaf_size[0]) + 1;
  const uint64_t dy = static_cast<uint64_t>((max_p[1] - min_p[1]) * _inverse_leaf_size[1]) + 1;
  const uint64_t dz = static_cast<uint64_t>((max_p[2] - min_p[2]) * _inverse_leaf_size[2]) + 1;

  if ((dx * dy * dz) > static_cast<uint64_t>(std::numeric_limits<int>::max())) {
    ROS_WARN(
        "[AloamMapping] Leaf size in VoxelFilter is too small for the input dataset (min: [%.1f, %.1f, %.1f], max: [%.1f, %.1f, %.1f], dxyz: [%ld, %ld, %ld], "
        "points in cloud: %ld). "
        "Integer indices would overflow. Doing nothing.",
        min_p.x(), min_p.y(), min_p.z(), max_p.x(), max_p.y(), max_p.z(), dx, dy, dz, cloud->size());
    _enabled = false;
    return;
  }

  _manager = manager;

  // Compute the minimum and maximum bounding box values
  _min_b_flt[0] = min_p[0] * _inverse_leaf_size[0];
  _min_b_flt[1] = min_p[1] * _inverse_leaf_size[1];
  _min_b_flt[2] = min_p[2] * _inverse_leaf_size[2];

  const Eigen::Vector3i min_b = Eigen::Vector3i(std::floor(_min_b_flt[0]), std::floor(_min_b_flt[1]), std::floor(_min_b_flt[2]));
  const Eigen::Vector3i max_b =
      Eigen::Vector3i(std::floor(max_p[0] * _inverse_leaf_size[0]), std::floor(max_p[1] * _inverse_leaf_size[1]), std::floor(max_p[2] * _inverse_leaf_size[2]));

  const Eigen::Vector3i div_b = max_b - min_b + Eigen::Vector3i::Ones();
  _div_b_mul                  = Eigen::Vector3i(1, div_b[0], div_b[0] * div_b[1]);
}

/*//}*/

/*//{ filter() */
template <class T_pt>
feature_selection::FSCloudManagerPtr<T_pt> VoxelFilter<T_pt>::filter() {

  if (_empty || !_enabled) {
    return std::make_shared<feature_selection::FSCloudManager<T_pt>>(boost::make_shared<pcl::PointCloud<T_pt>>(),
                                                                     std::make_shared<feature_extraction::indices_t>());
  }

  // Voxelization map
  std::unordered_map<int, std::pair<size_t, T_pt>> voxel_map;

  // The voxelized indices
  const feature_extraction::indices_ptr_t indices_voxelized = std::make_shared<feature_extraction::indices_t>();

  // Iterate over the entire input cloud
  const auto cloud = _manager->getCloudPtr();
  for (const auto &idx : *_manager->getIndicesPtr()) {

    const auto &p = cloud->at(idx.val);

    const Eigen::Vector3i ijk =
        Eigen::Vector3i(std::floor(p.x * _inverse_leaf_size[0]) - _min_b_flt[0], std::floor(p.y * _inverse_leaf_size[1]) - _min_b_flt[1],
                        std::floor(p.z * _inverse_leaf_size[2]) - _min_b_flt[2]);

    // Compute the centroid leaf index
    const int voxel_idx = ijk.dot(_div_b_mul);

    // Insert to map if there is no point already stored at the leaf index
    auto it = voxel_map.find(voxel_idx);
    if (it == voxel_map.end()) {

      voxel_map[voxel_idx] = {1, p};
      indices_voxelized->push_back(idx);

    } else if (_return_centroid) {

      // Update centroid
      it->second.first++;
      auto &mean = it->second.second;
      mean.x += (p.x - mean.x) / it->second.first;
      mean.y += (p.y - mean.y) / it->second.first;
      mean.z += (p.z - mean.z) / it->second.first;
    }
  }

  return std::make_shared<feature_selection::FSCloudManager<T_pt>>(cloud, indices_voxelized);
}
/*//}*/

// Explicit template instantiation
template class VoxelFilter<ouster_ros::Point>;
template class VoxelFilter<pandar_pointcloud::PointXYZIT>;
/* template class VoxelFilter<pcl::PointXYZI>; */

template struct MappingCloudManagersOut_t<ouster_ros::Point>;
template struct MappingCloudManagersOut_t<pandar_pointcloud::PointXYZIT>;
/* template struct MappingCloudManagersOut_t<pcl::PointXYZI>; */

template struct MappingData<ouster_ros::Point>;
template struct MappingData<pandar_pointcloud::PointXYZIT>;
/* template struct MappingData<pcl::PointXYZI>; */

template class AloamMapping<ouster_ros::Point>;
template class AloamMapping<pandar_pointcloud::PointXYZIT>;
/* template class AloamMapping<pcl::PointXYZI>; */

}  // namespace aloam_slam
