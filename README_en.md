# ROS2 Cross Compile Shell Script

This script is designed for cross-compiling ROS2 source code in a Linux environment. The build system uses a Docker-exported target architecture system environment and CMake toolchain files to cross-compile the ROS2 source code for the specified target architecture.

The script has been tested and is compatible with **Ubuntu 24.04.1**, using the **ROS** image version **jazzy**. It should work smoothly in a network environment with good connectivity.

![image](https://github.com/user-attachments/assets/e2094874-4f87-4b13-92a9-f4feb98a6e8a)
![image](https://github.com/user-attachments/assets/da18ed19-dc85-496c-bb42-31a95402d7cb)

The images above show the time overhead comparison between `rk3566` and `11th Gen Intel i7-11370H` when compiling a `ROS2` example program into an `aarch64` executable.

## Usage

### 1. Set up the `Docker` cross-compilation environment

First, configure the qemu and target architecture system environment inside `Docker`. Users need to install `Docker` and resolve any user permission or proxy issues.

Run the following command, and Docker will automatically download the `qemu` environment:
```shell
docker run --privileged --rm tonistiigi/binfmt --install all
```
> You can verify if `qemu` is installed successfully by running the following command:
```shell
docker run --rm arm64v8/alpine uname -a
```

Next, clone this repository to your local machine and build the compilation image:
```shell
git clone https://github.com/ChenYuWuAi/ros2_cross_compile.git && \
    cd ros2_cross_compile && \
    ./setup_qemu_environment.sh
```

After executing this command, a compilation image `cpp_pubsub:1.0-arm64` will be built in the directory, and a system root directory `ubuntu_arm64` will be exported.
> The `ubuntu_arm64` directory is the file system of this image.

> #### Tips
> If the image can't be downloaded, try changing the source or configuring a Docker proxy. If the proxy still doesn’t work, try manually pulling the `ros:jazzy` image using `docker pull ros:jazzy`. If that still doesn’t work, you can modify the `Dockerfile` by changing the `FROM` field in the first line to another source.

> #### How to add custom environments for cross-compiling your own package
> Modify the `Dockerfile` as needed and then rebuild the image by running `./setup_qemu_environment.sh`.

### 2. Compile the source code

In the `ros2_cross_compile` directory, execute the following command:
```shell
./cross_compile.sh
```

The script will begin compiling the ROS2 source code. Once the compilation is complete, an `install` folder will be generated in the `ros2_cross_compile` directory, containing the cross-compiled ROS2 source code.

In this example, we compiled the `cpp_pubsub` example. The compiled executable can be found in the `install` folder. To check the architecture of the executable, run the following command:
```shell
file install/lib/cpp_pubsub/talker
```
The output should look like this:
```shell
install/lib/cpp_pubsub/talker: ELF 64-bit LSB pie executable, ARM aarch64, version 1 (GNU/Linux), dynamically linked, interpreter /lib/ld-linux-aarch64.so.1, BuildID[sha1]=5a318e5e42e7db5430ea6d3d464d958e7c08f07f, for GNU/Linux 3.7.0, not stripped
```

To verify that the cross-compiled ROS2 source code can run on the target architecture, copy the `install` folder to the target system, then execute the following command:
```shell
source install/setup.bash && \
    ros2 run cpp_pubsub talker
```

If it runs successfully, the cross-compilation was successful.
