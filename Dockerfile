# 使用 ROS 2 Jazzy 官方基础镜像，强制使用本地构建
FROM dockerpull.org/ros:jazzy

# 设置工作目录
WORKDIR /ros_test_ws

# 换源
RUN sed -i 's/ports.ubuntu.com/mirrors.seu.edu.cn/g' /etc/apt/sources.list.d/ubuntu.sources

# # 更新并安装构建依赖项
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-controller-manager

COPY ./src/ros2_control_demos /home/ros2_ws/src/ros2_control_demos

ENV HTTP_PROXY=http://10.208.95.154:20172
ENV HTTPS_PROXY=http://10.208.95.154:20172

RUN cd /home/ros2_ws/src \
    && rosdep update --rosdistro ${ROS_DISTRO}  

RUN rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO} \
    --skip-keys ros-${ROS_DISTRO}-joint-state-publisher-gui --skip-keys ros-${ROS_DISTRO}-rviz2\
    && \
    : "remove cache" && \
    apt-get autoremove -y -qq && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/bin/bash", "-c"]