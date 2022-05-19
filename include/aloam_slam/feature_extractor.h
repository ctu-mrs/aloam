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
  FeatureExtractor(const std::shared_ptr<CommonHandlers_t> handlers, const std::shared_ptr<AloamOdometry> aloam_odometry);

  std::atomic<bool> is_initialized = false;

private:
  std::shared_ptr<CommonHandlers_t> _handlers;

  bool _has_required_parameters = false;

  ros::Subscriber _sub_input_data_processing_diag;
  void            callbackInputDataProcDiag(const mrs_msgs::PclToolsDiagnosticsConstPtr &msg);

  // member objects
  mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2> _sub_laser_cloud;

  std::shared_ptr<AloamOdometry>             _aloam_odometry;

  // member variables
  float _vertical_fov_half;
  float _ray_vert_delta;
  float _scan_period_sec;

  int _initialization_frames_delay;

  long int _frame_count = 0;

  int _number_of_rings;

  bool parseRowsFromOusterMsg(const sensor_msgs::PointCloud2::ConstPtr cloud, const pcl::PointCloud<PointType>::Ptr cloud_raw,
                              const pcl::PointCloud<PointType>::Ptr cloud_processed, std::vector<int> &rows_start_index, std::vector<int> &rows_end_index,
                              std::unordered_map<int, feature_selection::IndexPair> &indices_map_proc_in_raw);

  void appendVoxelizedIndices(const IndicesPtr &indices_to_be_appended, const pcl::PointCloud<PointType>::Ptr &cloud_in,
                              const feature_selection::Indices_t &indices_in_cloud, const float resolution);

  void pseudoDownsampleIndices(const IndicesPtr &indices_to_be_appended, const pcl::PointCloud<PointType>::Ptr &cloud,
                               const feature_selection::Indices_t &indices_in_cloud, const double resolution_sq);

  bool hasField(const std::string field, const sensor_msgs::PointCloud2::ConstPtr &msg);

  // callbacks
  void callbackLaserCloud(mrs_lib::SubscribeHandler<sensor_msgs::PointCloud2> &laserCloudMsg);

  /* ros::Publisher _pub_dbg; */
};
}  // namespace aloam_slam
#endif
