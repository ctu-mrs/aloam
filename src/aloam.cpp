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

  void initOdom();

private:
  std::shared_ptr<AloamMapping>      aloam_mapping;
  std::shared_ptr<AloamOdometry>     aloam_odometry;
  std::shared_ptr<FeatureExtractor>  feature_extractor;
  std::shared_ptr<mrs_lib::Profiler> profiler;

  std::string frame_fcu;
  std::string frame_lidar;
  std::string frame_init;
  std::thread t_odom_init;
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
  param_loader.loadParam("init_frame", frame_init, {});
  param_loader.loadParam("sensor_frequency", frequency);
  param_loader.loadParam("verbose", verbose, false);
  param_loader.loadParam("enable_profiler", enable_profiler, false);
  const auto initialize_from_odom = param_loader.loadParam2<bool>("initialize_from_odom", false);

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

  if (initialize_from_odom)
  {
    t_odom_init = std::thread(&AloamSlam::initOdom, this);
    t_odom_init.detach();
    ROS_WARN("[Aloam] Waiting for pose initialization.");
  }
  else
  {
    feature_extractor->is_initialized = true;
    aloam_odometry->is_initialized    = true;
    aloam_mapping->is_initialized     = true;
    ROS_INFO("[Aloam]: \033[1;32minitialized\033[0m");
  }
}

//}

/*//{ getStaticTf() */
tf::Transform AloamSlam::getStaticTf(std::string frame_from, std::string frame_to) {
  tf::Transform                         tf_ret;
  mrs_lib::Transformer transformer("Aloam");
  ros::Duration(1.0).sleep(); // Wait for TF buffer to fill

  ROS_INFO_ONCE("[Aloam]: Looking for transform from %s to %s", frame_from.c_str(), frame_to.c_str());
  auto tf_lidar_fcu = transformer.getTransform(frame_from, frame_to, ros::Time(0));
  while (!tf_lidar_fcu) {
    tf_lidar_fcu = transformer.getTransform(frame_from, frame_to, ros::Time(0));
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("[Aloam]: Successfully found transformation from %s to %s.", frame_from.c_str(), frame_to.c_str());

  tf::transformMsgToTF(tf_lidar_fcu->getTransform().transform, tf_ret);
  return tf_ret;
}
/*//}*/

/* initOdom() //{ */

void AloamSlam::initOdom()
{
  mrs_lib::Transformer transformer("Aloam");

  ROS_WARN_STREAM("[Aloam] Waiting for transformation between " << frame_lidar << " and " << frame_init << ".");
  bool got_tf = false;
  while (!got_tf && ros::ok())
  {
    const auto tf_opt = transformer.getTransform(frame_lidar, frame_init, ros::Time(0));
    if (tf_opt.has_value())
    {
      const auto mrs_tf = tf_opt.value();
      const auto tf = mrs_tf.getTransformEigen();

      /* Eigen::Isometry3d init_T = tf2::transformToEigen(tf.transform); */
      Eigen::Vector3d t(tf.translation());
      Eigen::Quaterniond q(tf.rotation());

      aloam_odometry->setTransform(t, q, mrs_tf.stamp());
      aloam_mapping->setTransform(t, q, mrs_tf.stamp());

      feature_extractor->is_initialized = true;
      aloam_odometry->is_initialized    = true;
      aloam_mapping->is_initialized     = true;
      got_tf = true;
      ROS_INFO("[Aloam]: \033[1;32minitialized\033[0m");
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "[AloamSlam]: Did not get odometry initialization transform between " << frame_lidar << " and " << frame_init << ".");
      ros::Duration(0.1).sleep();
    }
  }
}

//}

}  // namespace aloam_slam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aloam_slam::AloamSlam, nodelet::Nodelet)
