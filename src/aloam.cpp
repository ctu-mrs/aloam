/* includes //{ */

#include "aloam_slam/feature_extractor.h"
#include "aloam_slam/odometry.h"
#include "aloam_slam/mapping.h"

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

//}

namespace aloam_slam
{

/* //{ class AloamSlam */

class AloamSlam : public nodelet::Nodelet {

public:
  virtual void onInit();

  tf::Transform getStaticTf(const std::string &frame_from, const std::string &frame_to, const bool custom_buffer);

  void initOdom();

private:
  std::shared_ptr<AloamMapping>              aloam_mapping;
  std::shared_ptr<AloamOdometry>             aloam_odometry;
  std::shared_ptr<FeatureExtractor>          feature_extractor;
  std::shared_ptr<mrs_lib::Profiler>         profiler;
  std::shared_ptr<mrs_lib::ScopeTimerLogger> scope_timer_logger = nullptr;

  std::string frame_fcu;
  std::string frame_lidar;
  std::string frame_init;
  std::thread t_odom_init;

  /* Offline processing */
  ros::Timer      _timer_offline_proc;
  ros::Publisher  _pub_offline_points;
  ros::Subscriber _sub_global_odom;

  rosbag::Bag _bag;

  std::unique_ptr<tf2_ros::Buffer>            tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;

  std::unique_ptr<rosbag::View> _offline_points_view;
  rosbag::View::iterator        _offline_points_view_it;

  long   _offline_frame_count         = 0;
  long   _offline_frame_invalid_count = 0;
  double _offline_processing_timeout;

  void       callbackGlobalOdom(const nav_msgs::Odometry::ConstPtr msg);
  std::mutex _mutex_global_odom;
  bool       _has_global_odom = false;
  ros::Time  _global_odom_stamp;

  void callbackOfflineProcessing(const ros::TimerEvent &event);

  ros::Time _offline_expected_stamp;
  ros::Time _offline_expected_stamp_rostime;
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
  std::string   time_logger_filepath;
  std::string   offline_rosbag;
  std::string   offline_points_topic;
  float         frequency;
  tf::Transform tf_lidar_in_fcu_frame;
  bool          verbose;
  bool          offline_run;
  bool          enable_profiler;
  bool          enable_scope_timer;

  param_loader.loadParam("uav_name", uav_name);
  param_loader.loadParam("lidar_frame", frame_lidar);
  param_loader.loadParam("fcu_frame", frame_fcu);
  param_loader.loadParam("odom_frame", frame_odom);
  param_loader.loadParam("map_frame", frame_map);
  param_loader.loadParam("init_frame", frame_init, {});
  param_loader.loadParam("sensor_frequency", frequency, -1.0f);
  param_loader.loadParam("verbose", verbose, false);
  param_loader.loadParam("enable_profiler", enable_profiler, false);
  param_loader.loadParam("scope_timer/enable", enable_scope_timer, false);
  param_loader.loadParam("scope_timer/log_filename", time_logger_filepath, std::string(""));
  param_loader.loadParam("offline/run", offline_run, false);
  param_loader.loadParam("offline/rosbag", offline_rosbag);
  param_loader.loadParam("offline/points_topic", offline_points_topic);
  param_loader.loadParam("offline/timeout", _offline_processing_timeout, 0.2);
  const auto initialize_from_odom = param_loader.loadParam2<bool>("initialize_from_odom", false);

  if (verbose && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  /*//{ Open rosbag */
  if (offline_run) {
    try {
      ROS_INFO("[AloamSlam] Opening rosbag: %s", offline_rosbag.c_str());
      _bag.open(offline_rosbag, rosbag::bagmode::Read);
    }
    catch (...) {
      ROS_ERROR("[AloamSlam] Couldn't open rosbag: %s", offline_rosbag.c_str());
      ros::shutdown();
      return;
    }

    tf_buffer   = std::make_unique<tf2_ros::Buffer>();
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

    // Fill TF buffer with static TFs
    rosbag::View tf_static = rosbag::View(_bag, rosbag::TopicQuery("/tf_static"));
    for (const rosbag::MessageInstance &msg : tf_static) {

      if (!ros::ok()) {
        ros::shutdown();
        _bag.close();
        return;
      }

      const tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
      if (tf_msg) {
        for (const auto &transform : tf_msg->transforms) {
          tf_buffer->setTransform(transform, "default_authority", true);

          static tf2_ros::StaticTransformBroadcaster static_broadcaster;
          static_broadcaster.sendTransform(transform);
        }
      }
    }

    // Prepare ROS objects for points publishing and subscription of global odometry (from ALOAM mapping)
    _pub_offline_points = nh_.advertise<sensor_msgs::PointCloud2>("laser_cloud_in", 1);
    _sub_global_odom    = nh_.subscribe("odom_global_out", 1, &AloamSlam::callbackGlobalOdom, this, ros::TransportHints().tcpNoDelay());
  }
  /*//}*/

  // | --------------------- tf transformer --------------------- |
  tf_lidar_in_fcu_frame = getStaticTf(frame_fcu, frame_lidar, offline_run);

  // | ------------------------ profiler ------------------------ |
  profiler = std::make_shared<mrs_lib::Profiler>(nh_, "Aloam", enable_profiler);

  // | ------------------- scope timer logger ------------------- |
  scope_timer_logger = std::make_shared<mrs_lib::ScopeTimerLogger>(time_logger_filepath, enable_scope_timer);

  // | ----------------------- SLAM handlers  ------------------- |

  aloam_mapping =
      std::make_shared<AloamMapping>(nh_, param_loader, profiler, frame_fcu, frame_map, tf_lidar_in_fcu_frame, enable_scope_timer, scope_timer_logger);
  aloam_odometry = std::make_shared<AloamOdometry>(nh_, uav_name, profiler, aloam_mapping, frame_fcu, frame_lidar, frame_odom, 1.0f / frequency,
                                                   tf_lidar_in_fcu_frame, enable_scope_timer, scope_timer_logger);
  feature_extractor =
      std::make_shared<FeatureExtractor>(nh_, param_loader, profiler, aloam_odometry, frame_map, 1.0f / frequency, enable_scope_timer, scope_timer_logger);

  if (!param_loader.loadedSuccessfully()) {
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

  if (offline_run) {
    _timer_offline_proc = nh_.createTimer(ros::Rate(100), &AloamSlam::callbackOfflineProcessing, this, false, true);

    _offline_points_view    = std::make_unique<rosbag::View>(_bag, rosbag::TopicQuery(offline_points_topic));
    _offline_points_view_it = _offline_points_view->begin();
  }
}

//}

/*//{ getStaticTf() */
tf::Transform AloamSlam::getStaticTf(const std::string &frame_from, const std::string &frame_to, const bool custom_buffer) {

  ROS_INFO_ONCE("[Aloam]: Looking for transform from %s to %s", frame_from.c_str(), frame_to.c_str());
  geometry_msgs::TransformStamped tf_lidar_fcu;
  bool found = false;

  if (custom_buffer) {

    while (ros::ok() && !found)
    {
      try
      {
        tf_lidar_fcu = tf_buffer->lookupTransform(frame_to, frame_from, ros::Time(0));
        found = true;
      }
      catch (...)
      {
        ros::Duration(0.1).sleep();
      }
    }
  }
  else
  {
    mrs_lib::Transformer transformer("Aloam");
    transformer.setLookupTimeout(ros::Duration(0.1));

    while (ros::ok() && !found)
    {
      const auto ret = transformer.getTransform(frame_from, frame_to, ros::Time(0));
      if (ret.has_value())
      {
        found = true;
        tf_lidar_fcu = ret.value();
      }
    }
  }

  if (found)
    ROS_INFO("[Aloam]: Successfully found transformation from %s to %s.", frame_from.c_str(), frame_to.c_str());

  tf::Transform tf_ret;
  tf::transformMsgToTF(tf_lidar_fcu.transform, tf_ret);
  return tf_ret;
}
/*//}*/

/* initOdom() //{ */

void AloamSlam::initOdom() {
  mrs_lib::Transformer transformer("Aloam");

  ROS_WARN_STREAM("[Aloam] Waiting for transformation between " << frame_lidar << " and " << frame_init << ".");
  bool got_tf = false;
  while (!got_tf && ros::ok()) {
    const auto tf_opt = transformer.getTransform(frame_lidar, frame_init, ros::Time(0));
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
      ROS_WARN_STREAM_THROTTLE(1.0, "[AloamSlam]: Did not get odometry initialization transform between " << frame_lidar << " and " << frame_init << ".");
      ros::Duration(0.1).sleep();
    }
  }
}

//}

/* callbackOfflineProcessing() //{ */
void AloamSlam::callbackOfflineProcessing([[maybe_unused]] const ros::TimerEvent &event) {

  const bool initialized = feature_extractor->is_initialized && aloam_odometry->is_initialized && aloam_mapping->is_initialized;
  if (!initialized) {
    return;
  }

  // End of the msg queue
  if (_offline_points_view_it == _offline_points_view->end()) {
    ROS_INFO("[Aloam] Offline processing finished. Ending.");

    _timer_offline_proc.stop();
    _bag.close();
    ros::shutdown();

    return;
  }

  const ros::Time now = ros::Time::now();

  // The previous frame was processed and the output pose estimate from global mapping has been published
  bool received;
  {
    std::scoped_lock lock(_mutex_global_odom);
    received = _offline_frame_count > 0 && _has_global_odom && std::fabs((_offline_expected_stamp - _global_odom_stamp).toSec()) < 0.001;
  }

  // Processing of the previous frame took more than timeout threshold
  const bool timeout = _offline_frame_count > 0 && (now - _offline_expected_stamp_rostime).toSec() > _offline_processing_timeout;

  if (_offline_frame_count == 0 || received || timeout) {

    if ((_offline_frame_count + _offline_frame_invalid_count) % 10 == 0) {

      if (timeout) {
        ROS_WARN("[Aloam] Frame timeouted.");
      }

      ROS_INFO("[Aloam] Offline processing of frame: %ld/%d", _offline_frame_count + _offline_frame_invalid_count + 1, _offline_points_view->size());
    }

    // Deserialize to ROS msg
    const sensor_msgs::PointCloud2::Ptr cloud_msg = _offline_points_view_it->instantiate<sensor_msgs::PointCloud2>();
    if (!cloud_msg) {
      ROS_WARN("[Aloam] Failed to instantiate frame: %ld/%d", _offline_frame_count + _offline_frame_invalid_count + 1, _offline_points_view->size());

      _offline_frame_invalid_count++;
      _offline_points_view_it++;

      return;
    }

    try {

      // Publish cloud msg to feature extractor
      _pub_offline_points.publish(cloud_msg);

      _offline_frame_count++;
      _offline_points_view_it++;

      _offline_expected_stamp         = cloud_msg->header.stamp;
      _offline_expected_stamp_rostime = now;
    }
    catch (...) {
      ROS_ERROR("[Aloam] Failed to publish msg on topic (%s) during offline processing.", _pub_offline_points.getTopic().c_str());
    }
  }
}
//}

/* callbackGlobalOdom() //{ */
void AloamSlam::callbackGlobalOdom(const nav_msgs::Odometry::ConstPtr msg) {
  std::scoped_lock lock(_mutex_global_odom);
  _has_global_odom   = true;
  _global_odom_stamp = msg->header.stamp;
}
//}

}  // namespace aloam_slam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aloam_slam::AloamSlam, nodelet::Nodelet)
