/* includes //{ */

#include "aloam_slam/feature_extractor.h"

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>

//}

namespace aloam_slam
{

/* //{ class AloamSlam */

class AloamSlam : public nodelet::Nodelet {

public:
  virtual void onInit();

  tf::Transform getStaticTf(const std::string &frame_from, const std::string &frame_to);

  void initOdom();

private:
  std::shared_ptr<CommonHandlers_t> handlers;

  std::shared_ptr<AloamMapping>     aloam_mapping;
  std::shared_ptr<AloamOdometry>    aloam_odometry;
  std::shared_ptr<FeatureExtractor> feature_extractor;

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
  handlers               = std::make_shared<CommonHandlers_t>();
  handlers->nh           = nh_;
  handlers->param_loader = std::make_shared<mrs_lib::ParamLoader>(nh_, "Aloam");

  std::string time_logger_filepath;
  bool        verbose;
  bool        enable_profiler;
  bool        enable_scope_timer;

  handlers->param_loader->loadParam("uav_name", handlers->uav_name);
  handlers->param_loader->loadParam("lidar_frame", handlers->frame_lidar);
  handlers->param_loader->loadParam("fcu_frame", handlers->frame_fcu);
  handlers->param_loader->loadParam("odom_frame", handlers->frame_odom);
  handlers->param_loader->loadParam("map_frame", handlers->frame_map);
  handlers->param_loader->loadParam("init_frame", frame_init, {});
  handlers->param_loader->loadParam("sensor/lines", handlers->scan_lines);
  handlers->param_loader->loadParam("sensor/frequency", handlers->frequency, -1.0f);
  handlers->param_loader->loadParam("sensor/samples", handlers->samples_per_row);
  handlers->param_loader->loadParam("sensor/vertical_fov", handlers->vfov);
  handlers->param_loader->loadParam("verbose", verbose, false);
  handlers->param_loader->loadParam("enable_profiler", enable_profiler, false);
  handlers->param_loader->loadParam("scope_timer/enable", handlers->enable_scope_timer, false);
  handlers->param_loader->loadParam("scope_timer/log_filename", time_logger_filepath, std::string(""));
  const auto initialize_from_odom = handlers->param_loader->loadParam2<bool>("initialize_from_odom", false);

  handlers->offline_run = false;

  if (verbose && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // | ---------------------- TF lidar->fcu --------------------- |
  handlers->tf_lidar_in_fcu_frame = getStaticTf(handlers->frame_fcu, handlers->frame_lidar);

  // | ------------------------ profiler ------------------------ |
  handlers->profiler = std::make_shared<mrs_lib::Profiler>(nh_, "Aloam", enable_profiler);

  // | ------------------- scope timer logger ------------------- |
  handlers->scope_timer_logger = std::make_shared<mrs_lib::ScopeTimerLogger>(time_logger_filepath, enable_scope_timer);


  // | ----------------------- SLAM handlers  ------------------- |

  aloam_mapping     = std::make_shared<AloamMapping>(handlers);
  aloam_odometry    = std::make_shared<AloamOdometry>(handlers, aloam_mapping);
  feature_extractor = std::make_shared<FeatureExtractor>(handlers, aloam_odometry);

  if (!handlers->param_loader->loadedSuccessfully()) {
    ROS_ERROR("[Aloam]: Could not load all parameters!");
    ros::shutdown();
  }

  if (initialize_from_odom) {
    t_odom_init = std::thread(&AloamSlam::initOdom, this);
    t_odom_init.detach();
    ROS_WARN("[Aloam] Waiting for pose initialization.");
  } else {
    feature_extractor->is_initialized = true;
    aloam_odometry->is_initialized    = true;
    aloam_mapping->is_initialized     = true;
    ROS_INFO("[Aloam]: \033[1;32minitialized\033[0m");
  }
}

//}

/*//{ getStaticTf() */
tf::Transform AloamSlam::getStaticTf(const std::string &frame_from, const std::string &frame_to) {

  ROS_INFO_ONCE("[Aloam]: Looking for transform from %s to %s", frame_from.c_str(), frame_to.c_str());
  geometry_msgs::TransformStamped tf_lidar_fcu;
  bool                            found = false;

  mrs_lib::Transformer transformer("Aloam");
  transformer.setLookupTimeout(ros::Duration(0.1));

  while (ros::ok() && !found) {
    const auto ret = transformer.getTransform(frame_from, frame_to, ros::Time(0));
    if (ret.has_value()) {
      found        = true;
      tf_lidar_fcu = ret.value();
    }
  }

  if (found) {
    ROS_INFO("[Aloam]: Successfully found transformation from %s to %s.", frame_from.c_str(), frame_to.c_str());
  }

  tf::Transform tf_ret;
  tf::transformMsgToTF(tf_lidar_fcu.transform, tf_ret);
  return tf_ret;
}
/*//}*/

/* initOdom() //{ */

void AloamSlam::initOdom() {
  mrs_lib::Transformer transformer("Aloam");

  ROS_WARN_STREAM("[Aloam] Waiting for transformation between " << handlers->frame_lidar << " and " << frame_init << ".");
  bool got_tf = false;
  while (!got_tf && ros::ok()) {
    const auto tf_opt = transformer.getTransform(handlers->frame_lidar, frame_init, ros::Time(0));
    if (tf_opt.has_value()) {

      const auto tf = tf2::transformToEigen(tf_opt->transform);

      /* Eigen::Isometry3d init_T = tf2::transformToEigen(tf.transform); */
      Eigen::Vector3d    t(tf.translation());
      Eigen::Quaterniond q(tf.rotation());

      aloam_odometry->setTransform(t, q, tf_opt->header.stamp);
      aloam_mapping->setTransform(t, q, tf_opt->header.stamp);

      feature_extractor->is_initialized = true;
      aloam_odometry->is_initialized    = true;
      aloam_mapping->is_initialized     = true;
      got_tf                            = true;
      ROS_INFO("[Aloam]: \033[1;32minitialized\033[0m");
    } else {
      ROS_WARN_STREAM_THROTTLE(1.0, "[Aloam]: Did not get odometry initialization transform between " << handlers->frame_lidar << " and " << frame_init << ".");
      ros::Duration(0.1).sleep();
    }
  }
}

//}

}  // namespace aloam_slam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aloam_slam::AloamSlam, nodelet::Nodelet)
