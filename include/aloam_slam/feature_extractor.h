#ifndef ALOAM_FEATURE_EXTRACTOR_H
#define ALOAM_FEATURE_EXTRACTOR_H

#include "aloam_slam/odometry.h"
#include "aloam_slam/mapping.h"

namespace aloam_slam
{
class FeatureExtractor {

public:
  FeatureExtractor(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::shared_ptr<mrs_lib::Profiler> profiler,
                   std::shared_ptr<AloamOdometry> odometry, std::string map_frame, float scan_period_sec);

  bool is_initialized = false;

private:
  // member objects
  std::shared_ptr<mrs_lib::Profiler> _profiler;
  ros::Subscriber _sub_laser_cloud;

  std::shared_ptr<AloamOdometry> _odometry;

  // member variables
  std::string _frame_map;

  bool _preprocess_data;

  float _vertical_fov_half;
  float _ray_vert_delta;
  float _min_range_sq;
  float _max_range_sq;
  float _scan_period_sec;

  int _initialization_frames_delay;

  long int _frame_count = 0;

  int _number_of_rings;

  // member methods
  void removeCloseAndFarPointCloud(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out);

  // callbacks
  void callbackLaserCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
};
}  // namespace aloam_slam
#endif