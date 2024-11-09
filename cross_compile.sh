#!/bin/bash
# Author: Hao Cheng
# Read Parameters from the command line
if [ "$1" == "help" ]; then
    echo -e "\033[1;32mCross Compile ROS2 for ARM64"
    echo -e "\033[1;33mUsage: ./cross_compile.sh dest <port> <user>@<ip>:<path>"
    echo -e "Example: ./cross_compile.sh dest 8123 ubuntu@xx.xx.xx.xx:/home/ubuntu/ros_install\033[0m"
    exit 0
fi

# Set the target architecture
export TARGET_ARCH=aarch64
export SYSROOT=$(pwd)/ubuntu_arm64
export PYTHON_SOABI=cpython-36m-aarch64-linux-gnu
export ROS2_INSTALL_PATH=$SYSROOT/opt/ros/jazzy
export ROS_WS_INSTALL_PATH=ROS2_INSTALL_PATH

source $ROS2_INSTALL_PATH/setup.bash

echo -e "\033[1;32mBuilding ROS2 packages for $TARGET_ARCH\033[0m"
echo -e "\033[1;33mSYSROOT: $SYSROOT\033[0m"

if [ ! -d $SYSROOT ]; then
    echo -e "\033[1;31mSYSROOT does not exist. Please setup environment first\033[0m"
    exit 1
fi

colcon build --merge-install \
    --cmake-force-configure \
    --cmake-args \
        -DCMAKE_TOOLCHAIN_FILE=$(pwd)/toolchain.cmake

if [ $? -ne 0 ]; then
    echo -e "\033[1;31mFailed to build ROS2 packages\033[0m"
    exit 1
fi

# Modify the setup.bash to normal path for the target device
sed -i "s|$SYSROOT||g" $(pwd)/install/setup.bash
sed -i "s|$SYSROOT||g" $(pwd)/install/setup.zsh

echo -e "\033[1;32mROS2 packages built successfully.\033[0m"

# SCP the install folder to the target device using shell parameter indicating the target device IP and directory
# Example: ./cross_compile.sh -rP 8123 ubuntu@10.203.193.223:/home/ubuntu/ros_install
if [ "$1" == "dest" ]; then
    echo -e "\033[1;32mCopying the install folder to the target device\033[0m"
    scp -rP $2 $(pwd)/install $3
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to copy the install folder to the target device\033[0m"
        exit 1
    fi
else
    echo -e "\033[1;33mSkip copying the install folder to the target device\033[0m"
    exit 0
fi
