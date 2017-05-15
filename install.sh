#!/bin/sh

catkin config --cmake-args --DCMAKE_TOOLCHAIN_FILE=`pwd`/install/Toolchain-arm-linux-gnueabihf.cmake
