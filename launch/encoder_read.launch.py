import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='labjack_ros2_wrapper',
            executable='read_quad_encoder.py',
            name='encoder',
            output="screen",
        ),
        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name='rviz2',
        #     arguments=['-d', '/home/kwalker96/ros2_ws/src/imu_controller/rviz_config_array.rviz']
        # ),
    ])

