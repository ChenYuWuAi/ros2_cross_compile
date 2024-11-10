#!/bin/bash
# Author: Hao Cheng

# Set the target architecture
ARCH="arm64"
TARGET_ARCH="linux/$ARCH"
WORKSPACE_DIR=$(pwd)

TAG="cpp_pubsub:1.0-arm64"

# Gain root access
if [ "$EUID" -ne 0 ]; then
    sudo echo -e "\033[1;32mGaining root access..."
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to gain root access"
        echo -e "\033[1;31mFailed to gain root access"
        exit 1
    fi
    echo -e "\033[1;33mRoot access granted\033[0m"
fi

# Check installed
if ! docker >/dev/null 2>&1; then
    echo -e "\033[1;31mDocker is not installed. Please install it first.\033[0m"
    exit 1
fi

echo -e "\033[1;32mBuilding the docker image for $ARCH\033[0m"

# Build the docker image, if fails, stop the script
docker build . --platform=$TARGET_ARCH -t $TAG --load  # Limit the number of CPUs used by the container
if [ $? -ne 0 ]; then
    echo -e "\033[1;31mFailed to build the docker image\033[0m"
    exit 1
fi

docker run $TAG "echo 'Build successful'" >/dev/null 2>&1
if [ $? -ne 0 ]; then
    echo -e "\033[1;31mFailed to run the container, trying to reinitialize the qemu environment\033[0m"
    docker run --privileged --rm tonistiigi/binfmt --install all
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to reinitialize the qemu environment\033[0m"
        exit 1
    fi
    docker run $TAG "echo 'Build successful'" >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to run the container\033[0m"
        exit 1
    fi
fi

echo -e "\033[1;32mBuild complete. Copying the install folder to the workspace\033[0m"

# Need to run the container to copy the install folder
CONTAINER_ID=$(docker container ls -a | grep $TAG | awk 'NR==1 {print $1}')
if [ -z "$CONTAINER_ID" ]; then
    echo -e "\033[1;31mFailed to get the container ID\033[0m"
    exit 1
fi

# Copy the install folder to the workspace, remove first if it exists
if [ -d "$WORKSPACE_DIR/ubuntu_$ARCH" ]; then
    sudo rm -rf $WORKSPACE_DIR/ubuntu_$ARCH
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to remove the existing install folder\033[0m"
        exit 1
    fi
fi

echo -e "sudo docker cp $CONTAINER_ID:/ $WORKSPACE_DIR/ubuntu_$ARCH"
sudo docker cp $CONTAINER_ID:/ $WORKSPACE_DIR/ubuntu_$ARCH
if [ $? -ne 0 ]; then
    echo -e "\033[1;31mFailed to copy the install folder\033[0m"
    exit 1
fi
echo -e "\033[1;32mInstall folder copied to the workspace\033[0m"

# Add COLCON_IGNORE file to the install folder
echo -e "\033[1;32mAdding COLCON_IGNORE file to the install folder\033[0m"
sudo touch $WORKSPACE_DIR/ubuntu_$ARCH/COLCON_IGNORE
if [ $? -ne 0 ]; then
    echo -e "\033[1;31mFailed to add COLCON_IGNORE file\033[0m"
    exit 1
fi

# Mkdir for workspace
if [ ! -d "$WORKSPACE_DIR/ubuntu_$ARCH/workspace" ]; then
    sudo mkdir $WORKSPACE_DIR/ubuntu_$ARCH/workspace
    sudo chown $USER:$USER $WORKSPACE_DIR/ubuntu_$ARCH/workspace
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to create the workspace folder\033[0m"
        exit 1
    fi
fi

# Install the necessary packages
echo -e "\033[1;32mInstalling the necessary packages for cross-compilation\033[0m"
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu -y

echo -e "\033[1;32mSetup complete\033[0m"
