#!/bin/bash
# Author: Hao Cheng
# Read Parameters from the command line
if [ "$1" == "help" ]; then
    echo -e "\033[1;32mCross Compile ROS2 for ARM64"
    echo -e "\033[1;33mUsage: ./cross_compile.sh dest <port> <user>@<ip>:<path>"
    echo -e "Example: ./cross_compile.sh dest xxxx ubuntu@xx.xx.xx.xx:/home/ubuntu/ros_install\033[0m"
    exit 0
fi

# Set the target architecture
export TARGET_ARCH=aarch64
export SYSROOT=$(pwd)/ubuntu_arm64
export PYTHON_SOABI=cpython-36m-aarch64-linux-gnu
export ROS2_INSTALL_PATH=$SYSROOT/opt/ros/jazzy
export ROS_WS_INSTALL_PATH=/workspace/install

source $ROS2_INSTALL_PATH/setup.bash

echo -e "\033[1;32mBuilding ROS2 packages for $TARGET_ARCH\033[0m"
echo -e "\033[1;33mSYSROOT: $SYSROOT\033[0m"

if [ ! -d $SYSROOT ]; then
    echo -e "\033[1;31mSYSROOT does not exist. Please setup environment first\033[0m"
    exit 1
fi

# 将本地的工作空间install文件夹链接到SYSROOT的install文件夹
if [ -d $SYSROOT/$ROS_WS_INSTALL_PATH ]; then
    rm $SYSROOT/$ROS_WS_INSTALL_PATH -rf
fi
ln -s $(pwd)/install $SYSROOT/$ROS_WS_INSTALL_PATH

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

if [ "$1" == "dest" ]; then
    echo -e "\033[1;32mParsing target device information\033[0m"

    # 解析 $3 参数
    USER=$(echo $3 | cut -d'@' -f1)
    HOST=$(echo $3 | cut -d'@' -f2 | cut -d':' -f1)
    REMOTE_PATH=$(echo $3 | cut -d':' -f2)

    if [ -z "$USER" ] || [ -z "$HOST" ] || [ -z "$REMOTE_PATH" ]; then
        echo -e "\033[1;31mInvalid target device format: $3\033[0m"
        echo -e "\033[1;33mExpected format: username@host:/path/to/target\033[0m"
        exit 1
    fi

    echo -e "\033[1;32mCompressing the install folder\033[0m"
    tar -czf install.tar.gz -C $(pwd) install
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to compress the install folder\033[0m"
        exit 1
    fi

    echo -e "\033[1;32mCopying the compressed file to the target device\033[0m"
    scp -P $2 install.tar.gz $USER@$HOST:$REMOTE_PATH > /dev/null
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to copy the compressed file to the target device\033[0m"
        rm -f install.tar.gz
        exit 1
    fi

    echo -e "\033[1;32mExtracting the compressed file on the target device\033[0m"
    ssh -p $2 $USER@$HOST "cd $REMOTE_PATH && tar -xzf install.tar.gz && rm -f install.tar.gz"
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to extract the compressed file on the target device\033[0m"
        rm -f install.tar.gz
        exit 1
    fi

    echo -e "\033[1;32mCleaning up local temporary files\033[0m"
    rm -f install.tar.gz

    echo -e "\033[1;32mOperation completed successfully\033[0m"
else
    echo -e "\033[1;33mSkip copying the install folder to the target device\033[0m"
    exit 0
fi
