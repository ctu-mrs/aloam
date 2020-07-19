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

  aloam_mapping     = std::make_shared<AloamMapping>(nh_, param_loader, profiler, frame_fcu, frame_map, frequency, tf_lidar_in_fcu_frame);
  aloam_odometry    = std::make_shared<AloamOdometry>(nh_, uav_name, profiler, aloam_mapping, frame_fcu, frame_lidar, frame_odom, 1.0f / frequency, tf_lidar_in_fcu_frame);
  feature_extractor = std::make_shared<FeatureExtractor>(nh_, param_loader, profiler, aloam_odometry, frame_map, 1.0f / frequency);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Aloam]: Could not load all parameters!");
    ros::shutdown();
  }

  feature_extractor->is_initialized = true;
  aloam_odometry->is_initialized    = true;
  aloam_mapping->is_initialized     = true;

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

}  // namespace aloam_slam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aloam_slam::AloamSlam, nodelet::Nodelet)
