import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    at_nav_dir = get_package_share_directory('at_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    at_params_file = os.path.join(at_nav_dir, 'config', 'at_nav2_params.yaml')
    at_map_file = os.path.join(at_nav_dir, 'maps', 'map.yaml')
    at_bt_xml = os.path.join(at_nav_dir, 'config', 'bt_navigator.xml')

    # pbstream 地图（需先用 cartographer 建图生成）
    pbstream_file = os.path.join(at_nav_dir, 'maps', 'map.pbstream')

    # Cartographer pure localization 节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', os.path.join(at_nav_dir, 'config'),
            '-configuration_basename', 'cartographer_localization.lua',
            '-load_state_filename', pbstream_file,
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
    )

    # Nav2 bringup — controller_server 默认发 /cmd_vel
    # FSM 仲裁后将最终指令发到 /motor_cmd_vel，底盘驱动订阅 /motor_cmd_vel
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': at_map_file,
            'bt_xml': at_bt_xml,
            'params_file': at_params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
            'use_composition': 'false',
            'use_lifecycle_mgr': 'true',
            'slam': 'false',
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(bringup_cmd)

    return ld
