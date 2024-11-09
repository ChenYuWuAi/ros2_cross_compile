# ROS Launch File: cpp_pubsub.launch.py
# Description: Launches the C++ publisher and subscriber nodes
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pubsub',
            executable='talker',
            name='cpp_publisher',
            output='screen'
        )
    ])
