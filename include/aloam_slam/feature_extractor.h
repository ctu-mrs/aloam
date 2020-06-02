#ifndef ALOAM_FEATURE_EXTRACTOR_H
#define ALOAM_FEATURE_EXTRACTOR_H

#include "aloam_slam/odometry.h"
#include "aloam_slam/mapping.h"

namespace aloam_slam
{
class FeatureExtractor {

public:
  FeatureExtractor(const ros::NodeHandle &parent_nh, mrs_lib::ParamLoader param_loader, std::shared_ptr<AloamOdometry> odometry, std::string map_frame,
                   float scan_period_sec);

  bool is_initialized = false;

private:
  // member objects
  ros::Subscriber _sub_laser_cloud;

  std::shared_ptr<AloamOdometry> _odometry;

  // member variables
  std::string _frame_map;

  bool _external_preprocessing;

  float _vertical_fov_half;
  float _ray_vert_delta;
  float _min_range_sq;
  float _max_range_sq;
  float _scan_period_sec;

  int _initialization_frames_delay;

  long int _frame_count = 0;

  int _number_of_rings;

  // TODO: make these arrays local or add mutexes
  float cloudCurvature[400000];
  int   cloudSortInd[400000];
  int   cloudNeighborPicked[400000];
  int   cloudLabel[400000];

  // member methods

  bool comp(int i, int j);
  void removeCloseAndFarPointCloud(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out);

  // callbacks
  void callbackLaserCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
};
}  // namespace aloam_slam
#endif
