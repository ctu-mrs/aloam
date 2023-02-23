/* includes //{ */

#include "aloam_slam/feature_extractor.h"

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

  const sensor_msgs::PointCloud2::Ptr pclToRos(const feature_selection::FSCloudManagerPtr manager);
  const tf2_msgs::TFMessage           geomToTf2(const geometry_msgs::TransformStamped &geom_msg);

  /* Rosbag processing */

  rosbag::Bag _bag_in_;
  rosbag::Bag _bag_out_;

  std::unique_ptr<rosbag::View> bag_points_view;

  std::unique_ptr<tf2_ros::Buffer>            tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;

  void readWriteRosbag(const std::string &rosbag_points_topic, const bool write_bag_out, const bool write_input_points, const bool write_pointclouds);
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
  handlers->offline_run = true;

  const bool        shutdown_ros  = handlers->param_loader->loadParamReusable<bool>("rosbag/shutdown_ros", false);
  const bool        write_bag_out = handlers->param_loader->loadParamReusable<bool>("rosbag/write_out/enable");
  const std::string bag_dir       = handlers->param_loader->loadParamReusable<std::string>("rosbag/directory");
  const std::string bag_fn_in     = bag_dir + "/" + handlers->param_loader->loadParamReusable<std::string>("rosbag/filename");

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

  std::string rosbag_points_topic;
  bool        write_input_points;
  bool        write_pointclouds;

  if (write_bag_out) {

    const std::string bag_fn_out = bag_dir + "/" + handlers->param_loader->loadParamReusable<std::string>("rosbag/write_out/filename");
    rosbag_points_topic          = handlers->param_loader->loadParamReusable<std::string>("rosbag/points_topic");
    write_input_points           = handlers->param_loader->loadParamReusable<bool>("rosbag/write_out/input_points");
    write_pointclouds            = handlers->param_loader->loadParamReusable<bool>("rosbag/write_out/pointclouds");

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

  /*//{ Read rosbag duration */
  bag_points_view                = std::make_unique<rosbag::View>(_bag_in_, rosbag::TopicQuery(rosbag_points_topic));
  const ros::Time bag_begin_time = bag_points_view->getBeginTime();
  const ros::Time bag_end_time   = bag_points_view->getEndTime();
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

        // Write static TFs every 5 seconds
        ros::Time t = msg.getTime();
        while (t < bag_end_time) {
          _bag_out_.write(msg.getTopic(), t, tf_msg);

          t += ros::Duration(5.0);
        }
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

  readWriteRosbag(rosbag_points_topic, write_bag_out, write_input_points, write_pointclouds);

  if (shutdown_ros) {
    ROS_INFO("[AloamRosbag]: Shutting down ROS.");
    nh_.shutdown();
    ros::shutdown();
  }
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

/* readWriteRosbag() //{ */
void AloamSlamRosbag::readWriteRosbag(const std::string &rosbag_points_topic, const bool write_bag_out, const bool write_input_points,
                                      const bool write_pointclouds) {

  const std::string topic_diagnostics      = "/" + handlers->uav_name + "/slam/diagnostics";
  const std::string topic_edges_extracted  = "/" + handlers->uav_name + "/slam/features/edges/extracted";
  const std::string topic_edges_salient    = "/" + handlers->uav_name + "/slam/features/edges/salient";
  const std::string topic_edges_selected   = "/" + handlers->uav_name + "/slam/features/edges/selected";
  const std::string topic_planes_extracted = "/" + handlers->uav_name + "/slam/features/planes/extracted";
  const std::string topic_planes_salient   = "/" + handlers->uav_name + "/slam/features/planes/salient";
  const std::string topic_planes_selected  = "/" + handlers->uav_name + "/slam/features/planes/selected";

  // Wait for initialization
  bool initialized = feature_extractor->is_initialized && aloam_odometry->is_initialized && aloam_mapping->is_initialized;
  while (!initialized) {
    ROS_INFO_THROTTLE(1.0, "[AloamRosbag] Rosbag processing is waiting for initialization.");

    ros::Duration(0.1).sleep();
    initialized = feature_extractor->is_initialized && aloam_odometry->is_initialized && aloam_mapping->is_initialized;
  }

  const size_t frame_total_count   = bag_points_view->size();
  int          frame_valid_count   = 0;
  int          frame_invalid_count = 0;

  for (auto it = bag_points_view->begin(); it != bag_points_view->end(); it++) {

    if (!ros::ok()) {
      break;
    }

    const int  frame = frame_valid_count + frame_invalid_count + 1;
    const auto stamp = it->getTime();

    if (frame % int(frame_total_count * 0.1) == 0) {
      ROS_INFO("[AloamRosbag] Processing frame: %d/%ld (%d %s done)", frame, frame_total_count, int(100.0 * frame / frame_total_count), "%");
    }

    // | ---------------- Read message from rosbag ---------------- |
    const sensor_msgs::PointCloud2::Ptr cloud_msg = it->instantiate<sensor_msgs::PointCloud2>();
    if (!cloud_msg) {
      ROS_WARN("[AloamRosbag] Failed to instantiate frame: %d/%ld", frame, frame_total_count);

      frame_invalid_count++;
      continue;
    }

    if (write_bag_out && write_input_points) {
      _bag_out_.write(it->getTopic(), stamp, cloud_msg);
    }

    // | -------------------- Extract features -------------------- |
    FECloudManagersOut_t fe_managers;
    const bool           write_clouds = write_bag_out && write_pointclouds;
    const bool           fe_succ      = feature_extractor->extractFeatures(cloud_msg, write_clouds, fe_managers);

    if (!fe_succ) {
      ROS_WARN("[AloamRosbag] Feature extraction failed for frame: %d/%ld.", frame, frame_total_count);

      frame_invalid_count++;
      continue;
    }

    if (write_clouds) {
      _bag_out_.write(topic_edges_salient, stamp, pclToRos(fe_managers.man_edges_salient));
      _bag_out_.write(topic_planes_salient, stamp, pclToRos(fe_managers.man_planes_salient));
      _bag_out_.write(topic_edges_extracted, stamp, pclToRos(fe_managers.man_edges_extracted));
      _bag_out_.write(topic_planes_extracted, stamp, pclToRos(fe_managers.man_planes_extracted));
      if (fe_managers.man_edges_selected) {
        _bag_out_.write(topic_edges_selected, stamp, pclToRos(fe_managers.man_edges_selected));
      }
      if (fe_managers.man_planes_selected) {
        _bag_out_.write(topic_planes_selected, stamp, pclToRos(fe_managers.man_planes_selected));
      }
    }

    // | ---------------------- Call odometry --------------------- |
    geometry_msgs::TransformStamped tf_msg_odom;

    const bool odom_succ = aloam_odometry->computeOdometry(tf_msg_odom);
    if (!odom_succ) {
      ROS_WARN("[AloamRosbag] Odometry failed for frame: %d/%ld.", frame, frame_total_count);

      frame_invalid_count++;
      continue;
    }

    if (write_bag_out && !tf_msg_odom.child_frame_id.empty() && !tf_msg_odom.header.frame_id.empty()) {
      _bag_out_.write("/tf", stamp, geomToTf2(tf_msg_odom));
    }

    // | ---------------------- Call mapping ---------------------- |
    geometry_msgs::TransformStamped   tf_msg_mapping;
    aloam_slam::AloamDiagnostics::Ptr diag_msg_mapping;

    const bool mapping_succ = aloam_mapping->computeMapping(tf_msg_mapping, diag_msg_mapping);

    if (!mapping_succ) {
      ROS_WARN("[AloamRosbag] Mapping failed for frame: %d/%ld.", frame, frame_total_count);

      frame_invalid_count++;
      continue;
    }

    if (write_bag_out) {
      _bag_out_.write("/tf", stamp, geomToTf2(tf_msg_mapping));
      _bag_out_.write(topic_diagnostics, stamp, diag_msg_mapping);
    }

    frame_valid_count++;
  }

  _bag_in_.close();
  if (write_bag_out) {
    _bag_out_.close();
  }
}
//}

/*//{ pclToRos() */
const sensor_msgs::PointCloud2::Ptr AloamSlamRosbag::pclToRos(const feature_selection::FSCloudManagerPtr manager) {
  const sensor_msgs::PointCloud2::Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*manager->getUnorderedCloudPtr(), *msg);
  return msg;
}
/*//}*/

/*//{ geomToTf2() */
const tf2_msgs::TFMessage AloamSlamRosbag::geomToTf2(const geometry_msgs::TransformStamped &geom_msg) {
  tf2_msgs::TFMessage tf2_msg;
  tf2_msg.transforms = {geom_msg};
  return tf2_msg;
}
/*//}*/

}  // namespace aloam_slam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aloam_slam::AloamSlamRosbag, nodelet::Nodelet)
