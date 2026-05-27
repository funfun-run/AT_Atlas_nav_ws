import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    #获取包的路径包括当前的包和nav2_bringup包
    at_nav_dir = get_package_share_directory('at_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    #指定机器人的参数文件和地图文件路径
    #注意：确保这地图文件和param文件存在于你的 config 和 maps 文件夹中
    at_params_file = os.path.join(at_nav_dir, 'config', 'at_nav2_params.yaml')
    at_map_file = os.path.join(at_nav_dir, 'maps', 'map.yaml')        # 修改地图文件
    at_bt_xml = os.path.join(at_nav_dir, 'config', 'bt_navigator.xml')

    #调用bringup_launch.py
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': at_map_file,
            'bt_xml': at_bt_xml,
            'params_file': at_params_file,
            'use_sim_time': 'false',             # 真实物理机器人设为false，仿真环境设为true
            'autostart': 'true',                 # 自动启动节点
            'use_composition': 'false',          # 使用组件化，传入true节省真实机器人的CPU，但先传入false调试，跑通流程后再改成true
            'use_lifecycle_mgr': 'true',         # 使用生命周期管理器，传入true，避免节点启动后卡在unconfigured 状态
            'slam': 'false'                      # 导航模式传入false，如果使用建图模式则传入true
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(bringup_cmd)
    
    return ld