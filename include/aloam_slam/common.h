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

#include <ouster_ros/point.h>

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
  bool                                           is_new = true;
  pcl::PointCloud<ouster_ros::Point>::Ptr        cloud_raw;
  pcl::PointCloud<PointType>::Ptr                cloud_filt;
  std::vector<int>                               rows_start_idxs;
  std::vector<std::vector<unsigned int>>         indices_corners_sharp;
  std::vector<std::vector<unsigned int>>         indices_surfs_flat;
  std::vector<std::vector<unsigned int>>         indices_corners_less_sharp;
  std::vector<std::vector<unsigned int>>         indices_surfs_less_flat;
  std::unordered_map<unsigned int, unsigned int> point_indices_in_raw_cloud;
  aloam_slam::AloamDiagnostics::Ptr              aloam_diag_msg;

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

  std::vector<std::vector<int>> buildOrderedFeatureTable(const std::vector<std::vector<unsigned int>> &indices_in_filt) {

    std::vector<std::vector<int>> ret;
    ret.resize(cloud_raw->height, std::vector<int>(cloud_raw->width, -1));

    for (unsigned int i = 0; i < indices_in_filt.size(); i++) {
      for (unsigned int j = 0; j < indices_in_filt.at(i).size(); j++) {
        const unsigned int idx_in_filt = indices_in_filt.at(i).at(j);
        /* ROS_DEBUG("i|j|ind_filt|ind_raw|r|c ... %d|%d|%d|%d|%d|%d", i, j, indices_in_filt.at(i).at(j), ind_in_raw, r, c); */
        const auto [r, c] = getRowColInRawData(idx_in_filt);
        ret.at(r).at(c)   = (int)idx_in_filt;
      }
    }

    return ret;
  }

  std::tuple<unsigned int, unsigned int> getRowColInRawData(const unsigned int index_in_filt) {
    const unsigned int ind_in_raw = point_indices_in_raw_cloud[index_in_filt];
    const unsigned int r          = cloud_raw->at(ind_in_raw).ring;
    const unsigned int c          = ind_in_raw - r * cloud_raw->width;
    return std::make_tuple(r, c);
  }

  float getRange(const unsigned int index_in_filt) {
    const unsigned int ind_in_raw = point_indices_in_raw_cloud[index_in_filt];
    return cloud_raw->at(ind_in_raw).range / 1000.0f;
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
        cloud->push_back(cloud_filt->at(ind));
      }
    }

    cloud->header   = cloud_filt->header;
    cloud->height   = 1;
    cloud->width    = static_cast<uint32_t>(cloud->points.size());
    cloud->is_dense = true;

    return cloud;
  }
};
/*//}*/

/*//{ struct LUT */
struct LUT
{

  unsigned int width;
  unsigned int height;
  float        vfov;

  void build() {

    if (_is_filled) {
      return;
    }

    resolution_vertical   = vfov / float(height - 1);
    resolution_horizontal = 2.0f * M_PI / float(width);

    first_row_elevation = -vfov / 2.0f;

    table_el_az.resize(height, std::vector<std::pair<float, float>>(width));
    table_from_cloud.resize(width * height);

    unsigned int cloud_idx = 0;

    // Precompute possible elevations
    std::vector<float> elevations(height);
    for (unsigned int i = 0; i < height; i++) {
      elevations.at(i) = first_row_elevation + float(i) * resolution_vertical;
    }

    for (int j = width - 1; j >= 0; j--) {
      const float az = float(j) * resolution_horizontal;

      for (unsigned int i = 0; i < height; i++) {
        const float el = elevations.at(i);

        table_el_az.at(i).at(j) = std::make_pair(el, az);

        const std::pair<unsigned int, unsigned int> idx_pair = std::make_pair(i, j);
        table_from_cloud.at(cloud_idx)                       = idx_pair;
        table_to_cloud[idx_pair]                             = cloud_idx;

        cloud_idx++;
      }
    }

    _is_filled = true;

    // DEBUG
    /* unsigned int prev_r = -1; */
    /* unsigned int prev_c = -1; */
    /* for (int i = 0; i < cloud->size(); i++) { */

    /*   if (cloud->points.at(i).ring % 4 != 0) { */
    /*     continue; */
    /*   } */

    /*   const auto point = cloud->at(i); */
    /*   if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) { */
    /*     const float el = std::atan2(point.z, point.x); */
    /*     const float az = std::atan2(point.y, point.x); */

    /*     /1* const unsigned int r = getRow(el); *1/ */
    /*     const unsigned int r = cloud->points.at(i).ring / 4; */
    /*     const unsigned int c = getColumn(az); */

    /*     /1* ROS_DEBUG("el|az : %0.1f|%0.1f -- r|c : %d|%d", el, az, r, c); *1/ */
    /*     if (prev_r == r) { */
    /*       ROS_DEBUG("c++:      r|c : %d|%d -> %d|%d", prev_r, prev_c, r, c); */
    /*     } else if (prev_c == c) { */
    /*       ROS_DEBUG("r++:      r|c : %d|%d -> %d|%d", prev_r, prev_c, r, c); */
    /*     } else { */
    /*       ROS_DEBUG("r++, c++: r|c : %d|%d -> %d|%d", prev_r, prev_c, r, c); */
    /*     } */

    /*     prev_r = r; */
    /*     prev_c = c; */
    /*   } */
    /* } */
  }

  std::pair<unsigned int, unsigned int> getIdxs(const unsigned int cloud_idx) {
    return table_from_cloud.at(cloud_idx);
  }

  float getElevation(const unsigned int &r, const unsigned int &c) {
    return table_el_az.at(r).at(c).first;
  }

  float getAzimuth(const unsigned int &r, const unsigned int &c) {
    return table_el_az.at(r).at(c).second;
  }

  float getElevation(const unsigned int &cloud_idx) {
    const auto idxs = getIdxs(cloud_idx);
    return getElevation(idxs.first, idxs.second);
  }

  float getAzimuth(const unsigned int &cloud_idx) {
    const auto idxs = getIdxs(cloud_idx);
    return getAzimuth(idxs.first, idxs.second);
  }

  unsigned int getCloudIdx(const unsigned int &r, const unsigned int &c) {
    return table_to_cloud[std::make_pair(r, c)];
  }

  unsigned int getRow(const float &elevation) {
    return std::roundf(elevation / resolution_vertical);
  }

  unsigned int getColumn(const float &azimuth) {
    int c = std::roundf(azimuth / resolution_horizontal);
    if (c < 0) {
      c += width;
    }
    return (unsigned int)c;
  }

  void buildFeatureTable(std::unordered_map<unsigned int, unsigned int> &feature_map, const std::vector<std::vector<unsigned int>> &feature_indices,
                         std::vector<std::vector<int>> &table, std::unordered_map<unsigned int, std::pair<unsigned int, unsigned int>> &inv_table) {

    table.resize(height, std::vector<int>(width, -1));

    for (const auto &row : feature_indices) {
      for (const auto &ind : row) {
        ROS_ERROR("ind: %d", ind);
        const auto idx_pair = getIdxs(feature_map[ind]);
        ROS_ERROR("... map[ind]: %d, r: %d, c: %d", feature_map[ind], idx_pair.first, idx_pair.second);

        table.at(idx_pair.first).at(idx_pair.second) = ind;
        inv_table[ind]                               = idx_pair;
      }
    }
  }

private:
  bool _is_filled = false;

  float resolution_horizontal;
  float resolution_vertical;
  float first_row_elevation;

  /* 2d vector of elevation/azimuth pairs */
  std::vector<std::vector<std::pair<float, float>>> table_el_az;

  // TODO: table_to_cloud can be also vectorized with a known width*height conversion
  /* cloud index -> lut table position */
  std::vector<std::pair<unsigned int, unsigned int>> table_from_cloud;
  /* lut table position -> cloud index */
  std::map<std::pair<unsigned int, unsigned int>, unsigned int> table_to_cloud;
};
/*//}*/
