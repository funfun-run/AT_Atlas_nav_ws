import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("laser_driver"),
        "config",
    )

    params_file = os.path.join(config_dir, "lsn10p_params.yaml")

    laser_node = Node(
        package="laser_driver",
        executable="laser_driver_node",
        name="laser_driver",
        parameters=[params_file],
        output="screen",
    )

    return LaunchDescription([laser_node])
