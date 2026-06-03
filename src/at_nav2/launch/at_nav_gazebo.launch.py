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
    rviz_config = os.path.join(at_nav_dir, 'rviz2', 'nav2_gazebo.rviz')

    # ================================================================
    # Nav2 bringup 可传入参数（由 nav2_bringup/bringup_launch.py 接收）：
    #
    #   namespace          - 顶层命名空间                       (default: '')
    #   use_namespace      - 是否启用命名空间                    (default: 'false')
    #   slam               - 是否运行 SLAM（建图模式）            (default: 'False')
    #   map                - map.yaml 地图文件完整路径           (必填，本工程传入 at_map_file)
    #   use_sim_time       - 使用 Gazebo 仿真时钟               (default: 'false')
    #   params_file        - Nav2 参数 yaml 文件完整路径         (default: nav2_params.yaml)
    #   autostart          - 自动启动导航栈                     (default: 'true')
    #   use_composition    - 使用节点组合（component container）  (default: 'True')
    #   use_respawn        - 节点崩溃后自动重启                  (default: 'False')
    #   log_level          - 日志等级                          (default: 'info')
    #
    # 以下参数穿透 bringup_launch.py 直达 navigation_launch.py：
    #   use_lifecycle_mgr  - 是否使用生命周期管理                (default: 'True')
    # ================================================================
    # controller_server 默认发 /cmd_vel
    # FSM 仲裁后将最终指令发到 /motor_cmd_vel，底盘驱动订阅 /motor_cmd_vel
    
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'False',
            'map': at_map_file,
            'use_sim_time': 'True',
            'params_file': at_params_file,
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
    )


    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    ld = LaunchDescription()
    ld.add_action(bringup_cmd)
    ld.add_action(rviz2_node)

    return ld
