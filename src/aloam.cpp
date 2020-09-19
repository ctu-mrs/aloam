/* includes //{ */

#include "aloam_slam/feature_extractor.h"
#include "aloam_slam/odometry.h"
#include "aloam_slam/mapping.h"

//}

namespace aloam_slam
{

/* //{ class AloamSlam */

class AloamSlam : public nodelet::Nodelet {

public:
  virtual void onInit();

  tf::Transform getStaticTf(std::string frame_from, std::string frame_to);

private:
  std::shared_ptr<AloamMapping>      aloam_mapping;
  std::shared_ptr<AloamOdometry>     aloam_odometry;
  std::shared_ptr<FeatureExtractor>  feature_extractor;
  std::shared_ptr<mrs_lib::Profiler> profiler;

  // dynamic reconfigure
  typedef dynamic_reconfigure::Server<aloam_slam::aloam_dynparamConfig> ReconfigureServer;
  void                                 callbackReconfigure(aloam_slam::aloam_dynparamConfig &config, [[maybe_unused]] uint32_t level);
  boost::recursive_mutex               _mutex_reconfigure_config;
  boost::shared_ptr<ReconfigureServer> _reconfigure_server;
  aloam_slam::aloam_dynparamConfig     _config_last_reconfigure;
};

//}

/* //{ onInit() */

void AloamSlam::onInit() {
  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[Aloam]: initializing");

  // | --------------------- parameters ------------------------- |

  // Shared parameters
  mrs_lib::ParamLoader param_loader(nh_, "Aloam");

  std::string   uav_name;
  std::string   frame_lidar;
  std::string   frame_fcu;
  std::string   frame_odom;
  std::string   frame_map;
  float         frequency;
  tf::Transform tf_lidar_in_fcu_frame;
  bool          verbose;
  bool          enable_profiler;

  param_loader.loadParam("uav_name", uav_name);
  param_loader.loadParam("lidar_frame", frame_lidar);
  param_loader.loadParam("fcu_frame", frame_fcu);
  param_loader.loadParam("odom_frame", frame_odom);
  param_loader.loadParam("map_frame", frame_map);
  param_loader.loadParam("sensor_frequency", frequency);
  param_loader.loadParam("verbose", verbose, false);
  param_loader.loadParam("enable_profiler", enable_profiler, false);

  if (verbose && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // | --------------------- tf transformer --------------------- |
  tf_lidar_in_fcu_frame = getStaticTf(frame_fcu, frame_lidar);

  // | ------------------------ profiler ------------------------ |
  profiler = std::make_shared<mrs_lib::Profiler>(nh_, "Aloam", enable_profiler);

  // | ----------------------- SLAM handlers  ------------------- |

  aloam_mapping = std::make_shared<AloamMapping>(nh_, param_loader, profiler, frame_fcu, frame_map, frequency, tf_lidar_in_fcu_frame);
  aloam_odometry =
      std::make_shared<AloamOdometry>(nh_, uav_name, profiler, aloam_mapping, frame_fcu, frame_lidar, frame_odom, 1.0f / frequency, tf_lidar_in_fcu_frame);
  feature_extractor = std::make_shared<FeatureExtractor>(nh_, param_loader, profiler, aloam_odometry, frame_map, 1.0f / frequency);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Aloam]: Could not load all parameters!");
    ros::shutdown();
  }

  feature_extractor->is_initialized = true;
  aloam_odometry->is_initialized    = true;
  aloam_mapping->is_initialized     = true;

  _config_last_reconfigure.R_acc_lin = aloam_mapping->lkf_imu_R_lin_acc;
  _config_last_reconfigure.R_ang_vel = aloam_mapping->lkf_imu_R_ang_vel;
  _config_last_reconfigure.Q_pos     = aloam_mapping->lkf_imu_Q_pos;
  _config_last_reconfigure.Q_vel     = aloam_mapping->lkf_imu_Q_vel;
  _config_last_reconfigure.Q_att     = aloam_mapping->lkf_imu_Q_att;

  _reconfigure_server.reset(new ReconfigureServer(_mutex_reconfigure_config, nh_));
  _reconfigure_server->updateConfig(_config_last_reconfigure);
  ReconfigureServer::CallbackType f = boost::bind(&AloamSlam::callbackReconfigure, this, _1, _2);
  _reconfigure_server->setCallback(f);

  ROS_INFO("[Aloam]: initialized");
}

//}

/*//{ getStaticTf() */
tf::Transform AloamSlam::getStaticTf(std::string frame_from, std::string frame_to) {
  tf::Transform                         tf_ret;
  std::shared_ptr<mrs_lib::Transformer> transformer_ = std::make_shared<mrs_lib::Transformer>("Aloam");

  ROS_INFO_ONCE("[Aloam]: Looking for transform from %s to %s", frame_from.c_str(), frame_to.c_str());
  auto tf_lidar_fcu = transformer_->getTransform(frame_from, frame_to, ros::Time(0));
  while (!tf_lidar_fcu) {
    tf_lidar_fcu = transformer_->getTransform(frame_from, frame_to, ros::Time(0));
  }
  ROS_INFO("[Aloam]: Successfully found transformation from %s to %s.", frame_from.c_str(), frame_to.c_str());

  tf::transformMsgToTF(tf_lidar_fcu->getTransform().transform, tf_ret);
  return tf_ret;
}
/*//}*/

/* //{ callbackReconfigure() */
void AloamSlam::callbackReconfigure([[maybe_unused]] aloam_slam::aloam_dynparamConfig &config, [[maybe_unused]] uint32_t level) {
  if (feature_extractor->is_initialized) {
  }
  if (aloam_odometry->is_initialized) {
  }
  if (aloam_mapping->is_initialized) {
    ROS_INFO(
        "[FeatureExtractor] Reconfigure Request:\n"
        "Measurement noise (cov. matrix R scale):\n"
        "- linear acceleration (cov. scale): %0.10f\n"
        "- angular velocity (cov. scale):    %0.10f\n"
        "Process noise (cov. matrix Q scale):\n"
        "- position: %0.10f\n"
        "- velocity: %0.10f\n"
        "- attitude: %0.10f\n",
        config.R_acc_lin, config.R_ang_vel, config.Q_pos, config.Q_vel, config.Q_att);

    aloam_mapping->lkf_imu_R.block<3, 3>(0, 0) = config.R_acc_lin * Eigen::Matrix3d::Identity();
    aloam_mapping->lkf_imu_R.block<3, 3>(3, 3) = config.R_ang_vel * Eigen::Matrix3d::Identity();
    aloam_mapping->lkf_imu_Q.block<3, 3>(0, 0) = config.Q_pos * Eigen::Matrix3d::Identity();
    aloam_mapping->lkf_imu_Q.block<3, 3>(3, 3) = config.Q_vel * Eigen::Matrix3d::Identity();
    aloam_mapping->lkf_imu_Q.block<3, 3>(6, 6) = config.Q_att * Eigen::Matrix3d::Identity();
  }
}
//}

}  // namespace aloam_slam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aloam_slam::AloamSlam, nodelet::Nodelet)
