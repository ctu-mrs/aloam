#!/bin/bash

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

# CMake
sudo apt-get install cmake

# google-glog + gflags
sudo apt-get install libgoogle-glog-dev

# BLAS & LAPACK
sudo apt-get install libatlas-base-dev

# Eigen3
sudo apt-get install libeigen3-dev

# SuiteSparse and CXSparse (optional)
# - If you want to build Ceres as a *static* library (the default)
#   you can use the SuiteSparse package in the main Ubuntu package
#   repository:
sudo apt-get install libsuitesparse-dev

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
make test
make install

# clean tmp files
cd $SCRIPT_PATH
rm -rf tmp

