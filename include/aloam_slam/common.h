// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <cmath>

#include <pcl/point_types.h>

#include "aloam_slam/AloamDiagnostics.h"
#include "aloam_slam/FeatureSelectionDiagnostics.h"
#include "aloam_slam/FeatureExtractionDiagnostics.h"
#include "aloam_slam/OdometryDiagnostics.h"
#include "aloam_slam/MappingDiagnostics.h"

typedef pcl::PointXYZI PointType;

inline double rad2deg(double radians) {
  return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees) {
  return degrees * M_PI / 180.0;
}

/*//{ struct ExtractedFeatures */
struct ExtractedFeatures
{
  bool                                                        is_new = true;
  pcl::PointCloud<PointType>::Ptr                             cloud_full_res;
  std::vector<unsigned int>                                   rows_start_idxs;
  std::vector<std::vector<unsigned int>>                      indices_corners_sharp;
  std::vector<std::vector<unsigned int>>                      indices_surfs_flat;
  std::vector<std::vector<unsigned int>>                      indices_corners_less_sharp;
  std::vector<std::vector<unsigned int>>                      indices_surfs_less_flat;
  std::vector<std::unordered_map<unsigned int, unsigned int>> unprocessed_cloud_indices_row_maps;
  aloam_slam::AloamDiagnostics::Ptr                           aloam_diag_msg;

  pcl::PointCloud<PointType>::Ptr getSharpCorners() {
    if (!features_corners_sharp) {
      features_corners_sharp = extractCloudFromIndices(indices_corners_sharp);
    }
    return features_corners_sharp;
  }

  pcl::PointCloud<PointType>::Ptr getFlatSurfs() {
    if (!features_surfs_flat) {
      features_surfs_flat = extractCloudFromIndices(indices_surfs_flat);
    }
    return features_surfs_flat;
  }

  pcl::PointCloud<PointType>::Ptr getLessSharpCorners() {
    if (!features_corners_less_sharp) {
      features_corners_less_sharp = extractCloudFromIndices(indices_corners_less_sharp);
    }
    return features_corners_less_sharp;
  }

  pcl::PointCloud<PointType>::Ptr getLessFlatSurfs() {
    if (!features_surfs_less_flat) {
      features_surfs_less_flat = extractCloudFromIndices(indices_surfs_less_flat);
    }
    return features_surfs_less_flat;
  }

private:
  pcl::PointCloud<PointType>::Ptr features_corners_sharp;
  pcl::PointCloud<PointType>::Ptr features_corners_less_sharp;
  pcl::PointCloud<PointType>::Ptr features_surfs_flat;
  pcl::PointCloud<PointType>::Ptr features_surfs_less_flat;

  pcl::PointCloud<PointType>::Ptr extractCloudFromIndices(const std::vector<std::vector<unsigned int>> indices) {
    pcl::PointCloud<PointType>::Ptr cloud = boost::make_shared<pcl::PointCloud<PointType>>();

    for (unsigned int i = 0; i < indices.size(); i++) {

      for (const unsigned int &ind : indices.at(i)) {
        cloud->push_back(cloud_full_res->at(ind));
      }
    }

    cloud->header   = cloud_full_res->header;
    cloud->height   = 1;
    cloud->width    = static_cast<uint32_t>(cloud->points.size());
    cloud->is_dense = true;

    return cloud;
  }
};
/*//}*/
