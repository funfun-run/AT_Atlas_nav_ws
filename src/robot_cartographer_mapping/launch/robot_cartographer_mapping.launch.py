
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 定位到功能包的地址
    pkg_share = FindPackageShare(package='robot_cartographer_mapping').find('robot_cartographer_mapping')
    
    #=====================运行节点需要的配置=======================================================================
    # 是否使用仿真时间，我们用gazebo，这里设置成false
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    # 配置文件
    configuration_basename = LaunchConfiguration('configuration_basename', default='robot_2d_gazebo.lua')
    rviz_config_dir = os.path.join(pkg_share, 'config')+"/rviz2_cartographer.rviz"
    print(f"rviz config in {rviz_config_dir}")

    
    #=====================声明三个节点，cartographer/occupancy_grid_node/rviz_node=================================
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    robot_desc_dir = get_package_share_directory('robot_description')
    urdf_file = os.path.join(robot_desc_dir, 'urdf', 'robot_description.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    #===============================================定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(robot_state_pub)
    ld.add_action(rviz_node)

    return ld
