#ifndef ALOAM_FEATURE_EXTRACTOR_H
#define ALOAM_FEATURE_EXTRACTOR_H

#include "aloam_slam/odometry.h"
#include "aloam_slam/mapping.h"

#include <ouster_ros/point.h>
#include <mrs_lib/subscribe_handler.h>

namespace aloam_slam
{
class FeatureExtractor {

public:
  FeatureExtractor(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::shared_ptr<mrs_lib::Profiler> profiler,
                   std::shared_ptr<AloamOdometry> odometry, std::string map_frame, float scan_period_sec);

  std::atomic<bool> is_initialized = false;

private:
  // member objects
  std::shared_ptr<mrs_lib::Profiler> _profiler;
  mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2> _sub_laser_cloud;

  std::shared_ptr<AloamOdometry> _odometry;

  // member variables
  std::string _frame_map;

  float _vertical_fov_half;
  float _ray_vert_delta;
  float _scan_period_sec;

  int _initialization_frames_delay;

  long int _frame_count = 0;

  int _number_of_rings;

  bool _data_have_ring_field;

  void parseRowsFromCloudMsg(const sensor_msgs::PointCloud2::ConstPtr &cloud, pcl::PointCloud<PointType>::Ptr &cloud_processed,
                             std::vector<int> &rows_start_indices, std::vector<int> &rows_end_indices, float &processing_time);
  void parseRowsFromOS1CloudMsg(const sensor_msgs::PointCloud2::ConstPtr &cloud, pcl::PointCloud<PointType>::Ptr &cloud_processed,
                                std::vector<int> &rows_start_indices, std::vector<int> &rows_end_indices, float &processing_time);

  void removeNaNFromPointCloud(const pcl::PointCloud<ouster_ros::Point>::Ptr &cloud_in, pcl::PointCloud<ouster_ros::Point>::Ptr &cloud_out,
                               std::unordered_map<unsigned int, unsigned int> &indices);

  bool hasField(const std::string field, const sensor_msgs::PointCloud2::ConstPtr &msg);

  // callbacks
  void callbackLaserCloud(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2>& laserCloudMsg);
};
}  // namespace aloam_slam
#endif
