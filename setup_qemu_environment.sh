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
        echo -e "\031[1;31mFailed to gain root access"
        exit 1
    fi
    echo -e "\033[1;33mRoot access granted\033[0m"
fi

# Check installed
if ! docker >/dev/null 2>&1; then
    echo -e "\033[1;31mDocker is not installed. Please install it first.\033[0m"
    exit 1
fi

if ! docker buildx version >/dev/null 2>&1; then
    echo -e "\033[1;31mDocker buildx is not installed. Please install it first.\033[0m"
    exit 1
fi

# Check if builder exists
if ! docker buildx ls | grep -q "qemu_builder"; then
    echo -e "\033[1;33mBuilder does not exist. Creating a new one\033[0m"
    docker buildx create --name qemu_builder --driver-opt image=moby/buildkit:master --use >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to create the builder\033[0m"
        exit 1
    fi
fi

# Check if builder is active
if ! docker buildx inspect qemu_builder | grep -q "builder"; then
    echo -e "\033[1;31mBuilder is not active. Activating the builder\033[0m"
    docker buildx inspect qemu_builder --bootstrap >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to activate the builder\033[0m"
        exit 1
    fi
fi

echo -e "\033[1;32mBuilding the docker image for $ARCH\033[0m"

# Build the docker image, if fails, stop the script
docker buildx build . --platform=$TARGET_ARCH -t $TAG --load # Limit the number of CPUs used by the container
if [ $? -ne 0 ]; then
    echo -e "\033[1;31mFailed to build the docker image\033[0m"
    exit 1
fi

docker run $TAG "echo 'Build successful'" >/dev/null 2>&1
if [ $? -ne 0 ]; then
    echo -e "\033[1;31mFailed to run the container\033[0m"
    exit 1
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

echo -e "\033[1;32mSetup complete\033[0m"