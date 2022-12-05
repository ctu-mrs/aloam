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

/* //{ class AloamSlamRosbag */

class AloamSlamRosbag : public nodelet::Nodelet {

public:
  virtual void onInit();

  tf::Transform getStaticTf(const std::string &frame_from, const std::string &frame_to);

private:
  std::shared_ptr<CommonHandlers_t> handlers;

  std::shared_ptr<AloamMapping>     aloam_mapping;
  std::shared_ptr<AloamOdometry>    aloam_odometry;
  std::shared_ptr<FeatureExtractor> feature_extractor;

  /* Rosbag processing */

  rosbag::Bag _bag_in_;
  rosbag::Bag _bag_out_;

  std::unique_ptr<tf2_ros::Buffer>            tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;

  void processRosbag(const std::string &offline_points_topic, const bool write_bag_out);
};

//}

/* //{ onInit() */

void AloamSlamRosbag::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  ROS_INFO("[AloamRosbag]: initializing");

  // | --------------------- parameters ------------------------- |

  // Shared parameters
  handlers               = std::make_shared<CommonHandlers_t>();
  handlers->nh           = nh_;
  handlers->param_loader = std::make_shared<mrs_lib::ParamLoader>(nh_, "AloamRosbag");

  handlers->param_loader->loadParam("uav_name", handlers->uav_name);
  handlers->param_loader->loadParam("lidar_frame", handlers->frame_lidar);
  handlers->param_loader->loadParam("fcu_frame", handlers->frame_fcu);
  handlers->param_loader->loadParam("odom_frame", handlers->frame_odom);
  handlers->param_loader->loadParam("map_frame", handlers->frame_map);
  handlers->param_loader->loadParam("sensor/lines", handlers->scan_lines);
  handlers->param_loader->loadParam("sensor/frequency", handlers->frequency, -1.0f);
  handlers->param_loader->loadParam("sensor/samples", handlers->samples_per_row);
  handlers->param_loader->loadParam("sensor/vertical_fov", handlers->vfov);
  const bool verbose                     = handlers->param_loader->loadParamReusable<bool>("verbose", false);
  const bool enable_profiler             = handlers->param_loader->loadParamReusable<bool>("enable_profiler", false);
  handlers->enable_scope_timer           = handlers->param_loader->loadParamReusable<bool>("scope_timer/enable", false);
  const std::string time_logger_filepath = handlers->param_loader->loadParamReusable<std::string>("scope_timer/log_filename", std::string(""));

  if (verbose && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Setup offline processing
  handlers->offline_run    = true;
  const bool write_bag_out = handlers->param_loader->loadParamReusable<bool>("rosbag/write_out");

  const std::string bag_dir              = handlers->param_loader->loadParamReusable<std::string>("rosbag/file/directory");
  const std::string bag_fn_in            = bag_dir + "/" + handlers->param_loader->loadParamReusable<std::string>("rosbag/file/filename_in");
  const std::string bag_fn_out           = bag_dir + "/" + handlers->param_loader->loadParamReusable<std::string>("rosbag/file/filename_out");
  const std::string offline_points_topic = handlers->param_loader->loadParamReusable<std::string>("rosbag/points_topic");

  /*//{ Open rosbags */
  try {
    ROS_INFO("[AloamRosbag] Opening rosbag (%s) for reading.", bag_fn_in.c_str());
    _bag_in_.open(bag_fn_in, rosbag::bagmode::Read);
  }
  catch (...) {
    ROS_ERROR("[AloamRosbag] Couldn't open rosbag (%s) for reading.", bag_fn_in.c_str());
    ros::shutdown();
    return;
  }

  if (write_bag_out) {
    try {
      ROS_INFO("[AloamRosbag] Opening rosbag (%s) for writing.", bag_fn_out.c_str());
      _bag_out_.open(bag_fn_out, rosbag::bagmode::Write);
    }
    catch (...) {
      ROS_ERROR("[AloamRosbag] Couldn't open rosbag (%s) for writing.", bag_fn_out.c_str());
      ros::shutdown();
      return;
    }
  }
  /*//}*/

  /*//{ Fill TF buffer with static TFs */
  tf_buffer   = std::make_unique<tf2_ros::Buffer>();
  tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  rosbag::View tf_static = rosbag::View(_bag_in_, rosbag::TopicQuery("/tf_static"));
  for (const rosbag::MessageInstance &msg : tf_static) {

    if (!ros::ok()) {
      ros::shutdown();
      _bag_in_.close();
      if (write_bag_out) {
        _bag_out_.close();
      }
      return;
    }

    const tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
    if (tf_msg) {
      for (const auto &transform : tf_msg->transforms) {
        tf_buffer->setTransform(transform, "default_authority", true);

        static tf2_ros::StaticTransformBroadcaster static_broadcaster;
        static_broadcaster.sendTransform(transform);
      }

      if (write_bag_out) {
        _bag_out_.write(msg.getTopic(), msg.getTime(), tf_msg);
      }
    }
  }
  /*//}*/

  // Setup handlers
  handlers->tf_lidar_in_fcu_frame = getStaticTf(handlers->frame_fcu, handlers->frame_lidar);
  handlers->profiler              = std::make_shared<mrs_lib::Profiler>(nh_, "AloamRosbag", enable_profiler);
  handlers->scope_timer_logger    = std::make_shared<mrs_lib::ScopeTimerLogger>(time_logger_filepath, handlers->enable_scope_timer);

  // | ----------------------- SLAM handlers  ------------------- |
  aloam_mapping     = std::make_shared<AloamMapping>(handlers);
  aloam_odometry    = std::make_shared<AloamOdometry>(handlers, aloam_mapping);
  feature_extractor = std::make_shared<FeatureExtractor>(handlers, aloam_odometry);

  if (!handlers->param_loader->loadedSuccessfully()) {
    ROS_ERROR("[AloamRosbag]: Could not load all parameters!");
    ros::shutdown();
  }

  feature_extractor->is_initialized = true;
  aloam_odometry->is_initialized    = true;
  aloam_mapping->is_initialized     = true;
  ROS_INFO("[AloamRosbag]: \033[1;32minitialized\033[0m");

  processRosbag(offline_points_topic, write_bag_out);
}

//}

/*//{ getStaticTf() */
tf::Transform AloamSlamRosbag::getStaticTf(const std::string &frame_from, const std::string &frame_to) {

  ROS_INFO_ONCE("[AloamRosbag]: Looking for transform from %s to %s", frame_from.c_str(), frame_to.c_str());
  geometry_msgs::TransformStamped tf_lidar_fcu;
  bool                            found = false;

  while (ros::ok() && !found) {
    try {
      tf_lidar_fcu = tf_buffer->lookupTransform(frame_to, frame_from, ros::Time(0));
      found        = true;
    }
    catch (...) {
      ros::Duration(0.1).sleep();
    }
  }

  if (found) {
    ROS_INFO("[AloamRosbag]: Successfully found transformation from %s to %s.", frame_from.c_str(), frame_to.c_str());
  }

  tf::Transform tf_ret;
  tf::transformMsgToTF(tf_lidar_fcu.transform, tf_ret);
  return tf_ret;
}
/*//}*/

/* processRosbag() //{ */
void AloamSlamRosbag::processRosbag(const std::string &offline_points_topic, const bool write_bag_out) {

  // Wait for initialization
  bool initialized = feature_extractor->is_initialized && aloam_odometry->is_initialized && aloam_mapping->is_initialized;
  while (!initialized) {
    ROS_INFO_THROTTLE(1.0, "[AloamRosbag] Rosbag processing is waiting for initialization.");

    ros::Duration(0.1).sleep();
    initialized = feature_extractor->is_initialized && aloam_odometry->is_initialized && aloam_mapping->is_initialized;
  }

  const std::unique_ptr<rosbag::View> bag_view = std::make_unique<rosbag::View>(_bag_in_, rosbag::TopicQuery(offline_points_topic));

  const size_t frame_total_count   = bag_view->size();
  int          frame_valid_count   = 0;
  int          frame_invalid_count = 0;

  for (auto it = bag_view->begin(); it != bag_view->end(); it++) {

    if (!ros::ok()) {
      break;
    }

    const int frame = frame_valid_count + frame_invalid_count + 1;

    // | ---------------- Read message from rosbag ---------------- |
    const sensor_msgs::PointCloud2::Ptr cloud_msg = it->instantiate<sensor_msgs::PointCloud2>();
    if (!cloud_msg) {
      ROS_WARN("[AloamRosbag] Failed to instantiate frame: %d/%ld", frame, frame_total_count);

      frame_invalid_count++;
      continue;
    }

    // | -------------------- Extract features -------------------- |
    const bool fe_succ = feature_extractor->extractFeatures(cloud_msg);

    if (!fe_succ) {
      ROS_WARN("[AloamRosbag] Feature extraction failed for frame: %d/%ld.", frame, frame_total_count);

      frame_invalid_count++;
      continue;
    }

    // | ---------------------- Call odometry --------------------- |
    const bool odom_succ = aloam_odometry->computeOdometry();

    if (!odom_succ) {
      ROS_WARN("[AloamRosbag] Odometry failed for frame: %d/%ld.", frame, frame_total_count);

      frame_invalid_count++;
      continue;
    }

    // | ---------------------- Call mapping ---------------------- |
    const bool mapping_succ = aloam_mapping->computeMapping();

    if (!mapping_succ) {
      ROS_WARN("[AloamRosbag] Mapping failed for frame: %d/%ld.", frame, frame_total_count);

      frame_invalid_count++;
      continue;
    }

    frame_valid_count++;
  }

  _bag_in_.close();
}
//}

}  // namespace aloam_slam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aloam_slam::AloamSlamRosbag, nodelet::Nodelet)
