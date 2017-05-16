#!/bin/sh

loc=$(dirname $(readlink -f "$0"))

cd "$loc/../catkin_ws_private/"

if [ ! -d ".catkin_tools/profiles/indigo_arm" ]; then
	mkdir ".catkin_tools/profiles/indigo_arm"
fi

cp "../crosscompile/config.yaml" ".catkin_tools/profiles/indigo_arm/" 

catkin config --profile indigo_arm --cmake-args -DCMAKE_TOOLCHAIN_FILE="$loc/Toolchain-arm-linux-gnueabihf.cmake"

echo "\nRemember to 'touch src/Toolchain-arm-linux-gnueabihf.cmake' if catkin build encounters any errors.\n"
