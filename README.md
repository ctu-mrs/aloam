# A-LOAM
## Advanced implementation of LOAM

A-LOAM is an Advanced implementation of LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), which uses Eigen and Ceres Solver to simplify code structure.
This code is modified from LOAM and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).
This code is clean and simple without complicated mathematical derivation and redundant operations.
It is a good learning material for SLAM beginners.

The MRS version of A-LOAM is parallelized (nodeleted) and refactored to be more readable.
It also depends on some MRS-specific packages -- see below.

## 1. Prerequisites
The same prerequisities as for the MRS system:

* Ubuntu 64-bit 16.04, 18.04, or 20.04,
* ROS Kinetic, Melodic or Noetic ([ROS Installation](http://wiki.ros.org/ROS/Installation)),
* [Ceres Solver](http://ceres-solver.org/installation.html), and
* [PCL](http://www.pointclouds.org/downloads/linux.html).

## 2. Dependencies
The MRS version of A-LOAM depends on these packages:

* [mrs_lib](https://github.com/ctu-mrs/mrs_lib),
* [ouster driver](https://github.com/ctu-mrs/ouster), and
* [mrs_pcl_tools](https://mrs.felk.cvut.cz/gitlab/uav/perception/mrs_pcl_tools) (optional, but recommended).

Theese packages have to be available and built in your ROS workspace.

## 3. Install & Build A-LOAM
Clone and install the package using the prepared script:

```
    cd ~/git
    git clone git@mrs.felk.cvut.cz:uav/perception/aloam.git
    cd aloam/install
    ./install.sh
```

Then link to your workspace and compile it using `catkin build aloam_slam`.

## 4. Running A-LOAM
Launch example [launch file for Ouster OS](https://mrs.felk.cvut.cz/gitlab/uav/perception/aloam/blob/master/launch/uav_os_standalone.launch).
For example:

```
    roslaunch mrs_simulation simulation.launch
    spawn_uav 1 --f450 --run --delete --enable-rangefinder --enable-ouster --use-gpu-ray 
    roslaunch aloam_slam uav_os_standalone.launch
```

## 6.Acknowledgements
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).
