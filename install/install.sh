#!/bin/bash

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"
CERES_PATH=$SCRIPT_PATH/../lib
CERES_VERSION=1.14.0

# install dependencies
sudo apt update
sudo apt-get -y install \
  cmake \
  libgoogle-glog-dev \
  libatlas-base-dev \
  libeigen3-dev \
  libsuitesparse-dev \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-pcl-conversions \
  ros-$ROS_DISTRO-pcl-msgs


# download ceres solver
[ ! -d $CERES_PATH ] && mkdir -p $CERES_PATH
cd $CERES_PATH

if [ ! -d $CERES_PATH/ceres-solver-$CERES_VERSION ]
then
  # unpack source files
  wget -O $CERES_PATH/ceres-solver-$CERES_VERSION.tar.gz http://ceres-solver.org/ceres-solver-$CERES_VERSION.tar.gz
  tar zxf ceres-solver-$CERES_VERSION.tar.gz
  rm -f ceres-solver-$CERES_VERSION.tar.gz
fi

# install ceres solver
cd $CERES_PATH/ceres-solver-$CERES_VERSION
[ ! -d "build" ] && mkdir build
cd build
cmake ..
make -j$[$(nproc) - 1]
sudo make install
