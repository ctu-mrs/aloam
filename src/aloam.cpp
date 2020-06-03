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
  std::string   frame_map;
  float         frequency;
  tf::Transform tf_lidar_in_fcu_frame;

  param_loader.loadParam("uav_name", uav_name);
  param_loader.loadParam("lidar_frame", frame_lidar);
  param_loader.loadParam("fcu_frame", frame_fcu);
  param_loader.loadParam("map_frame", frame_map);

  param_loader.loadParam("frequency", frequency);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Aloam]: Could not load all parameters!");
    ros::shutdown();
  }

  // | --------------------- tf transformer --------------------- |

  std::shared_ptr<mrs_lib::Transformer> transformer_ = std::make_shared<mrs_lib::Transformer>("Aloam", uav_name);
  ROS_INFO("[Aloam]: Waiting 0.5 second to fill transform buffer.");
  ros::Duration(0.5).sleep();
  // TODO: no need for mrs_lib transformer for this simple task
  ROS_INFO_ONCE("[Aloam]: Looking for transform from %s to %s", frame_lidar.c_str(), frame_fcu.c_str());
  auto tf_lidar_fcu = transformer_->getTransform(frame_lidar, frame_fcu, ros::Time(0));
  while (!tf_lidar_fcu) {
    tf_lidar_fcu = transformer_->getTransform(frame_lidar, frame_fcu, ros::Time(0));
  }
  tf::transformMsgToTF(tf_lidar_fcu->getTransform().transform, tf_lidar_in_fcu_frame);
  ROS_INFO("[Aloam]: Successfully found transformation from %s to %s.", frame_lidar.c_str(), frame_fcu.c_str());

  // | ----------------------- SLAM handlers  ------------------- |

  std::shared_ptr<AloamMapping> aloam_mapping = std::make_shared<AloamMapping>(nh_, param_loader, frame_fcu, frame_map, tf_lidar_in_fcu_frame.inverse());

  std::shared_ptr<AloamOdometry> aloam_odometry =
      std::make_shared<AloamOdometry>(nh_, param_loader, aloam_mapping, frame_lidar, frame_map, 1.0f / frequency, tf_lidar_in_fcu_frame);

  std::shared_ptr<FeatureExtractor> feature_extractor = std::make_shared<FeatureExtractor>(nh_, param_loader, aloam_odometry, frame_map, 1.0f / frequency);

  // | ------------------------ profiler ------------------------ |
  // TODO: Add Profiler to all methods
  /* profiler_ = mrs_lib::Profiler(nh_, "Aloam", _profiler_enabled_); */
  // TODO: Add AttitudeConverter

  feature_extractor->is_initialized = true;
  aloam_odometry->is_initialized    = true;
  aloam_mapping->is_initialized     = true;

  ROS_INFO("[Aloam]: initialized");
  ros::spin();
}

//}

}  // namespace aloam_slam

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aloam_slam::AloamSlam, nodelet::Nodelet)
