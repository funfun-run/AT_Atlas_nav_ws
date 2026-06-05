import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # ── 1. gazebo仿真 ──

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'gazebo_sim.launch.py')
        )
    )
    ld.add_action(gazebo_launch)

    # ── 2. at_nav2 (Cartographer + Nav2 bringup) ──
    # 延迟启动，等 /scan 和 /odom 就绪
    at_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('at_nav2'),
                         'launch', 'at_nav_gazebo.launch.py')
        )
    )
    ld.add_action(TimerAction(period=5.0, actions=[at_nav_launch]))

    return ld
