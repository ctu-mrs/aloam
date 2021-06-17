#pragma once

namespace aloam_slam
{

/* extractFeatures() //{ */
static std::tuple<PC_ptr, PC_ptr, PC_ptr, PC_ptr> extractFeatures(const PC_ptr &laser_cloud, const std::vector<int> &rows_start_idxs,
                                                                  const std::vector<int> &rows_end_idxs, float &time_pts, float &time_q_sort) {
  std::vector<float> cloudCurvature;
  std::vector<int>   cloudSortInd;
  std::vector<int>   cloudNeighborPicked;
  std::vector<int>   cloudLabel;

  const unsigned int cloud_size = laser_cloud->points.size();
  cloudCurvature.resize(cloud_size);
  cloudSortInd.resize(cloud_size);
  cloudNeighborPicked.resize(cloud_size, 0);
  cloudLabel.resize(cloud_size, 0);

  for (unsigned int i = 5; i < cloud_size - 5; i++) {
    const float diffX = laser_cloud->points.at(i - 5).x + laser_cloud->points.at(i - 4).x + laser_cloud->points.at(i - 3).x + laser_cloud->points.at(i - 2).x +
                        laser_cloud->points.at(i - 1).x - 10 * laser_cloud->points.at(i).x + laser_cloud->points.at(i + 1).x + laser_cloud->points.at(i + 2).x +
                        laser_cloud->points.at(i + 3).x + laser_cloud->points.at(i + 4).x + laser_cloud->points.at(i + 5).x;
    const float diffY = laser_cloud->points.at(i - 5).y + laser_cloud->points.at(i - 4).y + laser_cloud->points.at(i - 3).y + laser_cloud->points.at(i - 2).y +
                        laser_cloud->points.at(i - 1).y - 10 * laser_cloud->points.at(i).y + laser_cloud->points.at(i + 1).y + laser_cloud->points.at(i + 2).y +
                        laser_cloud->points.at(i + 3).y + laser_cloud->points.at(i + 4).y + laser_cloud->points.at(i + 5).y;
    const float diffZ = laser_cloud->points.at(i - 5).z + laser_cloud->points.at(i - 4).z + laser_cloud->points.at(i - 3).z + laser_cloud->points.at(i - 2).z +
                        laser_cloud->points.at(i - 1).z - 10 * laser_cloud->points.at(i).z + laser_cloud->points.at(i + 1).z + laser_cloud->points.at(i + 2).z +
                        laser_cloud->points.at(i + 3).z + laser_cloud->points.at(i + 4).z + laser_cloud->points.at(i + 5).z;

    cloudCurvature.at(i) = diffX * diffX + diffY * diffY + diffZ * diffZ;
    cloudSortInd.at(i)   = i;
  }

  TicToc t_pts;

  time_q_sort = 0.0f;

  const PC_ptr corner_points_sharp      = boost::make_shared<pcl::PointCloud<PointType>>();
  const PC_ptr corner_points_less_sharp = boost::make_shared<pcl::PointCloud<PointType>>();
  const PC_ptr surf_points_flat         = boost::make_shared<pcl::PointCloud<PointType>>();
  const PC_ptr surf_points_less_flat    = boost::make_shared<pcl::PointCloud<PointType>>();

  for (int i = 0; i < rows_start_idxs.size(); i++) {
    if (rows_end_idxs.at(i) - rows_start_idxs.at(i) < 6) {
      continue;
    }
    const PC_ptr surfPointsLessFlatScan = boost::make_shared<pcl::PointCloud<PointType>>();
    for (int j = 0; j < 6; j++) {
      const int sp = rows_start_idxs.at(i) + (rows_end_idxs.at(i) - rows_start_idxs.at(i)) * j / 6;
      const int ep = rows_start_idxs.at(i) + (rows_end_idxs.at(i) - rows_start_idxs.at(i)) * (j + 1) / 6 - 1;

      TicToc t_tmp;
      std::sort(cloudSortInd.begin() + sp, cloudSortInd.begin() + ep + 1,
                [&cloudCurvature](int i, int j) { return cloudCurvature.at(i) < cloudCurvature.at(j); });
      time_q_sort += t_tmp.toc();

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        const int ind = cloudSortInd.at(k);

        if (cloudNeighborPicked.at(ind) == 0 && cloudCurvature.at(ind) > 0.1) {

          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel.at(ind) = 2;
            corner_points_sharp->push_back(laser_cloud->points.at(ind));
            corner_points_less_sharp->push_back(laser_cloud->points.at(ind));
          } else if (largestPickedNum <= 20) {
            cloudLabel.at(ind) = 1;
            corner_points_less_sharp->push_back(laser_cloud->points.at(ind));
          } else {
            break;
          }

          cloudNeighborPicked.at(ind) = 1;

          for (int l = 1; l <= 5; l++) {
            const float diffX = laser_cloud->points.at(ind + l).x - laser_cloud->points.at(ind + l - 1).x;
            const float diffY = laser_cloud->points.at(ind + l).y - laser_cloud->points.at(ind + l - 1).y;
            const float diffZ = laser_cloud->points.at(ind + l).z - laser_cloud->points.at(ind + l - 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }
          for (int l = -1; l >= -5; l--) {
            const float diffX = laser_cloud->points.at(ind + l).x - laser_cloud->points.at(ind + l + 1).x;
            const float diffY = laser_cloud->points.at(ind + l).y - laser_cloud->points.at(ind + l + 1).y;
            const float diffZ = laser_cloud->points.at(ind + l).z - laser_cloud->points.at(ind + l + 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        const int ind = cloudSortInd.at(k);

        if (cloudNeighborPicked.at(ind) == 0 && cloudCurvature.at(ind) < 0.1) {

          cloudLabel.at(ind) = -1;
          surf_points_flat->push_back(laser_cloud->points.at(ind));

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked.at(ind) = 1;
          for (int l = 1; l <= 5; l++) {
            const float diffX = laser_cloud->points.at(ind + l).x - laser_cloud->points.at(ind + l - 1).x;
            const float diffY = laser_cloud->points.at(ind + l).y - laser_cloud->points.at(ind + l - 1).y;
            const float diffZ = laser_cloud->points.at(ind + l).z - laser_cloud->points.at(ind + l - 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }
          for (int l = -1; l >= -5; l--) {
            const float diffX = laser_cloud->points.at(ind + l).x - laser_cloud->points.at(ind + l + 1).x;
            const float diffY = laser_cloud->points.at(ind + l).y - laser_cloud->points.at(ind + l + 1).y;
            const float diffZ = laser_cloud->points.at(ind + l).z - laser_cloud->points.at(ind + l + 1).z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked.at(ind + l) = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel.at(k) <= 0) {
          surfPointsLessFlatScan->push_back(laser_cloud->points.at(k));
        }
      }
    }

    pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointType>  downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    *surf_points_less_flat += surfPointsLessFlatScanDS;
  }

  time_pts = t_pts.toc();

  return std::make_tuple(corner_points_sharp, corner_points_less_sharp, surf_points_flat, surf_points_less_flat);
}

//}

}  // namespace aloam_slam

