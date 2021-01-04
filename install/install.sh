#!/bin/bash

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"
distro=`lsb_release -r | awk '{ print $2 }'`

# CMake
sudo apt-get install --yes cmake

# google-glog + gflags
sudo apt-get install --yes libgoogle-glog-dev

# BLAS & LAPACK
sudo apt-get install --yes libatlas-base-dev

# Eigen3
sudo apt-get install --yes libeigen3-dev

# SuiteSparse and CXSparse (optional)
# - If you want to build Ceres as a *static* library (the default)
#   you can use the SuiteSparse package in the main Ubuntu package
#   repository:
sudo apt-get install --yes libsuitesparse-dev
 
# PCL
if [ "$distro" = "18.04" ]; then
  sudo apt-get install --yes ros-melodic-pcl-ros ros-melodic-pcl-conversions ros-melodic-pcl-msgs
elif [ "$distro" = "20.04" ]; then
  sudo apt-get install --yes ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-pcl-msgs
else
  echo -e "\e[31mUbuntu version not 18.04 or 20.04, installing Noetic packages.\e[0m"
  sudo apt-get install --yes ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-pcl-msgs
fi

# download ceres solver
cd $SCRIPT_PATH
mkdir tmp
cd tmp
wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz

# unpack source files
tar zxf ceres-solver-1.14.0.tar.gz

# install ceres solver
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-1.14.0
make -j3
sudo make install

# clean tmp files
cd $SCRIPT_PATH
rm -rf tmp
