#!/bin/bash

# get the path to this script
MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`
cd "$MY_PATH"

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

CERES_PATH=$SCRIPT_PATH/../lib
CERES_VERSION=1.14.0

# IMPORTANT: These variables should match the settings of your catkin workspace
PROFILE="RelWithDebInfo" # RelWithDebInfo, Release, Debug
BUILD_WITH_MARCH_NATIVE=false
if [ ! -z "$PCL_CROSS_COMPILATION" ]; then
  BUILD_WITH_MARCH_NATIVE=$PCL_CROSS_COMPILATION
fi
CMAKE_STANDARD=17

# install dependencies
sudo apt-get -y update
sudo apt-get -y install \
  cmake \
  libgoogle-glog-dev \
  libatlas-base-dev \
  libeigen3-dev \
  libsuitesparse-dev \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-pcl-conversions \
  ros-$ROS_DISTRO-pcl-msgs

# Build with march native?
if $BUILD_WITH_MARCH_NATIVE; then
  echo "Building ceres optimizer with -march=native"
  CMAKE_MARCH_NATIVE="-march=native"
else
  echo "Building ceres optimizer without -march=native"
  CMAKE_MARCH_NATIVE=""
fi

# Profile-dependent flags
if [[ "$PROFILE" == "RelWithDebInfo" ]]; then
  BUILD_FLAGS_PROFILE=(
                  -DCMAKE_CXX_FLAGS_${PROFILE^^}="-O2 -g"
                  -DCMAKE_C_FLAGS_${PROFILE^^}="-O2 -g")
elif [[ "$PROFILE" == "Release" ]]; then
  BUILD_FLAGS_PROFILE=(
                  -DCMAKE_CXX_FLAGS_${PROFILE^^}="-O3"
                  -DCMAKE_C_FLAGS_${PROFILE^^}="-O3")
else
  BUILD_FLAGS_PROFILE=(
                  -DCMAKE_CXX_FLAGS_${PROFILE^^}="-O0 -g"
                  -DCMAKE_C_FLAGS_${PROFILE^^}="-O0 -g")
fi

# Defaults taken from mrs_workspace building flags
BUILD_FLAGS_GENERAL=( 
              -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 
              -DCMAKE_CXX_STANDARD=$CMAKE_STANDARD 
              -DCMAKE_BUILD_TYPE=$PROFILE
              -DCMAKE_CXX_FLAGS="-std=c++$CMAKE_STANDARD $CMAKE_MARCH_NATIVE" 
              -DCMAKE_C_FLAGS="$CMAKE_MARCH_NATIVE"
            )

# download ceres solver
echo "Downloading cerese soler"
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
echo "Compiling ceres solver"
cd $CERES_PATH/ceres-solver-$CERES_VERSION
[ ! -d "build" ] && mkdir build
cd build
cmake "${BUILD_FLAGS_GENERAL[@]}" "${BUILD_FLAGS_PROFILE[@]}" ../
make -j$[$(nproc) - 1]
sudo make install

echo "Done installing prerequisities for A-LOAM"
