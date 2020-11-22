#pragma once

#include "aloam_slam/odometry.h"
/* #include "aloam_slam/mapping.h" */

#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <unordered_map>

namespace aloam_slam
{

class FeatureExtractor {

public:
  FeatureExtractor(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::shared_ptr<mrs_lib::Profiler> profiler,
                   std::shared_ptr<AloamOdometry> odometry, std::string lidar_frame, float scan_period_sec);

  bool is_initialized = false;

private:
  // member objects
  std::shared_ptr<mrs_lib::Profiler> _profiler;
  ros::Subscriber                    _sub_laser_cloud;

  std::shared_ptr<AloamOdometry> _odometry;
  std::shared_ptr<LUT>           _lut;

  // member variables
  std::string _frame_lidar;

  float _vertical_fov_half;
  float _ray_vert_delta;
  float _scan_period_sec;

  int _initialization_frames_delay;

  long int _frame_count = 0;

  int _number_of_rings;
  int _os1_rings_diff;

  bool _data_have_ring_field;


  void parseRowsFromCloudMsg(const sensor_msgs::PointCloud2::ConstPtr &cloud, pcl::PointCloud<PointType>::Ptr &cloud_processed,
                             std::vector<unsigned int> &rows_start_indices, std::vector<unsigned int> &rows_end_indices, float &processing_time);
  void parseRowsFromOS1CloudMsg(const sensor_msgs::PointCloud2::ConstPtr &cloud, pcl::PointCloud<PointType>::Ptr &cloud_processed,
                                std::vector<unsigned int> &rows_start_indices, std::vector<unsigned int> &rows_end_indices, float &processing_time);

  void removeNaNFromPointCloud(const pcl::PointCloud<ouster_ros::OS1::PointOS1>::Ptr cloud_in, pcl::PointCloud<ouster_ros::OS1::PointOS1>::Ptr &cloud_out,
                               std::vector<unsigned int> &indices);

  bool hasField(const std::string field, const sensor_msgs::PointCloud2::ConstPtr &msg);

  void applyVoxelGridFilter(const pcl::PointCloud<PointType>::Ptr &cloud_in, const std::vector<unsigned int> &cloud_indices_in, const float &res_x,
                            const float &res_y, const float &res_z, pcl::PointCloud<PointType> &cloud_out, std::vector<unsigned int> &cloud_indices_out);

  // callbacks
  void callbackLaserCloud(const sensor_msgs::PointCloud2::ConstPtr &laserCloudMsg);
};
}  // namespace aloam_slam
