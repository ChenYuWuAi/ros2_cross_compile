# ROS2 Cross Compile Shell Script
> ROS2交叉编译脚本

本脚本适用于在Linux环境下交叉编译ROS2源码。构建系统利用docker导出的目标架构系统环境和cmake工具链文件，将ROS2源码交叉编译到指定的目标架构上。

## Usage
### 1. 配置`Docker`交叉编译环境
首先，需要在`Docker`中配置好qemu和目标架构的系统环境。用户需自行安装`Docker`、保证`buildx`插件已安装，自行解决用户权限和代理问题。

执行以下命令安装`qemu`和目标架构的系统环境：
```shell
docker run --privileged --rm tonistiigi/binfmt --install all
```
执行这个命令后，`Docker`会自动下载`qemu`的环境。
```shell
docker run --rm arm64v8/alpine uname -a
```
执行这个命令可以验证`qemu`是否安装成功。

然后，克隆本仓库到本地，并构建编译镜像：
```shell
git clone https://github.com/ChenYuWuAi/ros2_cross_compile.git && \
    cd ros2_cross_compile && \
    ./setup_qemu_environment.sh
```
执行这个命令后，会在目录下导出一个系统根目录`ubuntu_arm64`，并构建一个编译镜像`cpp_pubsub:1.0-arm64`.实际上，`ubuntu_arm64`就是这个镜像的文件系统。
### 2. 编译源码
在`ros2_cross_compile`目录下，执行以下命令：
```shell
./cross_compile.sh
```
脚本会直接开始编译ROS2源码，编译完成后，会在`ros2_cross_compile`目录下生成一个`install`文件夹，里面包含了交叉编译后的ROS2源码。

在本案例中，我们编译了一个`cpp_pubsub`的例程，可以在`install`文件夹下找到这个例程的可执行文件。通过以下命令查看程序的架构信息：
```shell
file install/lib/cpp_pubsub/talker
```
应该出现如下信息：
```shell
install/lib/cpp_pubsub/talker: ELF 64-bit LSB pie executable, ARM aarch64, version 1 (GNU/Linux), dynamically linked, interpreter /lib/ld-linux-aarch64.so.1, BuildID[sha1]=5a318e5e42e7db5430ea6d3d464d958e7c08f07f, for GNU/Linux 3.7.0, not stripped
```


要验证交叉编译后的ROS2源码是否可以在目标架构上运行，可以将`install`文件夹拷贝到目标架构系统上，然后执行以下命令：
```shell
source install/setup.bash && \
    ros2 run cpp_pubsub talker
```
如果可以正常运行，说明交叉编译成功。