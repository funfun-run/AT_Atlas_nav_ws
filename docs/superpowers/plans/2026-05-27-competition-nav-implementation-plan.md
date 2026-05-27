# 竞赛导航架构实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现分层导航架构：at_nav2 配置修正 + Cartographer 集成，mission_manager 增强航点能力，competition_fsm 状态机 + cmd_vel 仲裁，robot_startup 总启动。

**Architecture:** 比赛任务逻辑（competition_fsm）与导航逻辑（at_nav2 Nav2 bringup）分离。FSM 通过 service 接收外部事件，通过 action 调用 mission_manager 发航点，作为 `/cmd_vel` 唯一出口仲裁遥控与 Nav2 的底盘指令。

**Tech Stack:** ROS2 Humble, Python 3, Nav2, Cartographer ROS, ament_python

---

## 文件结构

```
src/
├── at_nav2/                          # 修改
│   ├── config/
│   │   ├── at_nav2_params.yaml       # 修改: /scan, remap cmd_vel
│   │   ├── bt_navigator.xml          # 保持
│   │   └── cartographer_localization.lua  # 新增
│   ├── launch/
│   │   └── at_nav.launch.py          # 修改: Cartographer节点, remap
│   ├── maps/
│   │   ├── map.pgm                   # 保持
│   │   └── map.yaml                  # 修改: 修复image路径
│   ├── CMakeLists.txt                # 保持
│   └── package.xml                   # 修改: 加cartographer依赖
│
├── mission_manager/                  # 修改
│   ├── mission_manager/
│   │   ├── __init__.py               # 修改: 导出新函数
│   │   ├── mission_manager.py        # 修改: 加action server, waypoint加载
│   │   └── waypoint_loader.py        # 新增: map.yaml解析
│   ├── action/
│   │   └── NavigateToZone.action     # 新增
│   ├── CMakeLists.txt                # 新增: rosidl接口生成
│   ├── package.xml                   # 修改: 加rosidl依赖
│   ├── setup.py                      # 修改
│   └── setup.cfg                     # 保持
│
├── competition_fsm/                  # 新增
│   ├── competition_fsm/
│   │   ├── __init__.py
│   │   ├── fsm_node.py               # 主节点: 状态机 + cmd_vel仲裁 + service
│   │   └── fsm.py                     # 状态机核心: Enum + dict + handlers
│   ├── srv/
│   │   └── FsmEvent.srv
│   ├── CMakeLists.txt                # rosidl接口生成
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   └── resource/competition_fsm      # ament marker
│
└── robot_startup/                    # 修改
    ├── launch/
    │   └── robot_start.launch.py     # 重写: 总启动
    └── package.xml                   # 修改: 加依赖
```

---

### Task 1: at_nav2 — 修复 costmap 扫描源为 /scan

**说明:** 当前 local_costmap 的 obstacle_layer 订阅 PointCloud2 `/cloud_registered_body`。改为订阅 LaserScan `/scan`，匹配 2D 雷达。

**Files:**
- Modify: `src/at_nav2/config/at_nav2_params.yaml:178-211`

- [ ] **Step 1: 修改 local_costmap 的 obstacle_layer 配置**

打开 `src/at_nav2/config/at_nav2_params.yaml`，找到 `local_costmap.local_costmap.ros__parameters.plugins` 下的 `obstacle_layer`，替换为 LaserScan 配置：

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: true
  observation_sources: scan
  scan:
    topic: /scan
    data_type: "LaserScan"
    marking: true
    clearing: true
    min_obstacle_height: 0.0
    max_obstacle_height: 0.0
    obstacle_min_range: 0.0
    obstacle_max_range: 6.0
    raytrace_min_range: 0.0
    raytrace_max_range: 8.0
```

同时将 `width` 从 6 改为 4，`height` 从 6 改为 4（2D 雷达不需要那么大范围的局部代价地图）：

```yaml
width: 4
height: 4
```

- [ ] **Step 2: 验证配置文件语法**

```bash
python3 -c "import yaml; yaml.safe_load(open('/home/funfun/AT_Atlas_nav_ws/src/at_nav2/config/at_nav2_params.yaml'))" && echo "YAML OK"
```

Expected: `YAML OK`

- [ ] **Step 3: Commit**

```bash
git add src/at_nav2/config/at_nav2_params.yaml
git commit -m "fix(at_nav2): 将 costmap obstacle_layer 从 PointCloud2 改为 LaserScan /scan"
```

---

### Task 2: at_nav2 — 修复 behavior_server 和 controller_server 参数

**说明:** 当前 behavior_server 使用 `global_frame: odom`，应改为 `map`。controller_server 的 `failure_tolerance` 调整为更适合 Regulated Pure Pursuit 的值。

**Files:**
- Modify: `src/at_nav2/config/at_nav2_params.yaml:46-54,106-130`

- [ ] **Step 1: 修改 behavior_server 的 global_frame**

```yaml
# behavior_server.ros__parameters 下:
global_frame: map
```

- [ ] **Step 2: 修改 controller_server 的 failure_tolerance**

`failure_tolerance` 从 0.3 改为 0.5（RegulatedPurePursuit 有时需要更大的容差）：

```yaml
failure_tolerance: 0.5
```

- [ ] **Step 3: Commit**

```bash
git add src/at_nav2/config/at_nav2_params.yaml
git commit -m "fix(at_nav2): 修正 behavior_server global_frame 和 controller 容差"
```

---

### Task 3: at_nav2 — 创建 Cartographer pure localization 配置

**说明:** 创建 Cartographer `.lua` 配置文件，纯定位模式加载预建地图。

**Files:**
- Create: `src/at_nav2/config/cartographer_localization.lua`

- [ ] **Step 1: 创建配置文件**

```lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65

-- 纯定位模式：不建图，只定位
TRAJECTORY_BUILDER.pure_localization = true

-- 加载预建的 .pbstream 地图
-- 注意：首次运行时需要先建图生成此文件，路径将在 launch 中指定
return options
```

- [ ] **Step 2: Commit**

```bash
git add src/at_nav2/config/cartographer_localization.lua
git commit -m "feat(at_nav2): 添加 Cartographer pure localization 配置文件"
```

---

### Task 4: at_nav2 — 更新 launch 文件和 package.xml

**说明:** 在 `at_nav.launch.py` 中加入 Cartographer 节点，remap controller_server 的 cmd_vel，并处理 `.pbstream` 地图路径。同时更新 `at_nav2/package.xml` 加 Cartographer 依赖。

**Files:**
- Modify: `src/at_nav2/launch/at_nav.launch.py`
- Modify: `src/at_nav2/package.xml`

- [ ] **Step 1: 更新 package.xml 添加 Cartographer 依赖**

在 `src/at_nav2/package.xml` 的 `<exec_depend>` 列表中添加：

```xml
<exec_depend>cartographer_ros</exec_depend>
```

- [ ] **Step 2: 重写 at_nav.launch.py**

```python
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

    # pbstream 地图（需要用 cartographer 先建图生成）
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

    # Nav2 bringup — 注意 remap controller_server 的 cmd_vel
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
```

- [ ] **Step 3: 在 Nav2 参数中添加 controller_server cmd_vel remap**

Nav2 的 `bringup_launch.py` 支持通过 `remappings` 参数传递 remap，但更简单的方式是在 `at_nav2_params.yaml` 的 `controller_server` 节点配置中将话题名直接指定。实际上 Nav2 controller_server 默认发 `/cmd_vel`，remap 必须在 launch 层面做。

由于 Nav2 节点由 `bringup_launch.py` 内部启动，无法直接加 remap。替代方案：在 `at_nav2_params.yaml` 中确认 controller_server 发布的话题名，然后在 `competition_fsm` 侧直接订阅默认的 `/cmd_vel`，改由 FSM remap。

**改用更简单的方案**：controller_server 保持默认发布 `/cmd_vel`。FSM 订阅 `/cmd_vel` 作为 Nav2 指令源，遥控器发 `/teleop_cmd_vel`。FSM 内部仲裁后发布到 `/motor_cmd_vel`（实际控制底盘的话题）。

更新 launch 文件的注释反映这个设计变更：

```python
# Nav2 controller_server 默认发 /cmd_vel
# FSM 仲裁后将最终指令发到 /motor_cmd_vel
# 底盘驱动节点需订阅 /motor_cmd_vel
```

- [ ] **Step 4: 修复 map.yaml 的 image 路径**

`src/at_nav2/maps/map.yaml` 中第 10 行的 image 路径是 Windows 路径，改为相对路径：

```yaml
image: map.pgm
```

- [ ] **Step 5: Commit**

```bash
git add src/at_nav2/launch/at_nav.launch.py src/at_nav2/package.xml src/at_nav2/maps/map.yaml
git commit -m "feat(at_nav2): 添加 Cartographer 节点到 launch，修复 map.yaml 路径"
```

---

### Task 5: mission_manager — 创建 NavigateToZone action 定义

**说明:** mission_manager 提供 `NavigateToZone` action，FSM 通过 action client 调用。action 定义放在 mission_manager 包内。

**Files:**
- Create: `src/mission_manager/action/NavigateToZone.action`
- Create: `src/mission_manager/CMakeLists.txt`
- Modify: `src/mission_manager/package.xml`
- Modify: `src/mission_manager/setup.py`

- [ ] **Step 1: 创建 action 定义文件**

```yaml
# NavigateToZone.action
string zone_name
---
bool success
string message
---
float32 distance_remaining
```

`src/mission_manager/action/NavigateToZone.action`

- [ ] **Step 2: 创建 CMakeLists.txt 生成接口**

```cmake
cmake_minimum_required(VERSION 3.16)
project(mission_manager)

find_package(rosidl_default_generators REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToZone.action"
  DEPENDENCIES nav2_msgs action_msgs
)

ament_package()
```

- [ ] **Step 3: 更新 package.xml 添加接口依赖**

在 `src/mission_manager/package.xml` 中添加：

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

- [ ] **Step 4: 更新 setup.py**

```python
from setuptools import setup
from glob import glob

package_name = 'mission_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='funfun',
    maintainer_email='1219921425@qq.com',
    description='Mission Manager with NavigateToZone action',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = mission_manager.mission_manager:main',
        ],
    },
)
```

- [ ] **Step 5: 构建验证**

```bash
cd /home/funfun/AT_Atlas_nav_ws
colcon build --symlink-install --packages-select mission_manager
```

Expected: Build succeeds, `NavigateToZone.action` 生成 Python 消息类。

- [ ] **Step 6: Commit**

```bash
git add src/mission_manager/action/ src/mission_manager/CMakeLists.txt src/mission_manager/package.xml src/mission_manager/setup.py
git commit -m "feat(mission_manager): 添加 NavigateToZone action 定义"
```

---

### Task 6: mission_manager — 实现 waypoint_loader 模块

**说明:** 从 `map.yaml` 解析 zones，计算每个 zone 的几何中心作为导航目标点。

**Files:**
- Create: `src/mission_manager/mission_manager/waypoint_loader.py`

- [ ] **Step 1: 实现 waypoint_loader.py**

```python
"""从 map.yaml 加载区域并计算导航航点。"""
import yaml
import os
from typing import Dict, List, Tuple


def load_waypoints(map_yaml_path: str) -> Dict[str, Tuple[float, float]]:
    """解析 map.yaml，返回 {zone_name: (center_x, center_y)} 字典。

    只提取 type == 'task_area' 或 'start_area' 的区域作为可导航目标。
    """
    with open(map_yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    waypoints: Dict[str, Tuple[float, float]] = {}

    for zone in data.get('zones', []):
        zone_type = zone.get('type', '')
        if zone_type not in ('task_area', 'start_area'):
            continue

        name = zone.get('name', '')
        polygon = zone.get('polygon', [])
        if not name or not polygon:
            continue

        # 计算多边形几何中心
        xs = [p[0] for p in polygon]
        ys = [p[1] for p in polygon]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)

        waypoints[name] = (cx, cy)

    return waypoints


def get_zone_names(map_yaml_path: str) -> List[str]:
    """返回所有可导航区域名称列表。"""
    waypoints = load_waypoints(map_yaml_path)
    return sorted(waypoints.keys())
```

- [ ] **Step 2: 单元测试**

```bash
python3 -c "
from mission_manager.waypoint_loader import load_waypoints
wps = load_waypoints('/home/funfun/AT_Atlas_nav_ws/src/at_nav2/maps/map.yaml')
print('Loaded waypoints:', len(wps))
for k, v in wps.items():
    print(f'  {k}: ({v[0]:.2f}, {v[1]:.2f})')
"
```

Expected: 打印出 15 个左右航点，包括 "货物码垛区B"、"货物分拣区B"、"启动区B" 等。

- [ ] **Step 3: Commit**

```bash
git add src/mission_manager/mission_manager/waypoint_loader.py
git commit -m "feat(mission_manager): 添加 waypoint_loader 模块解析 map.yaml"
```

---

### Task 7: mission_manager — 实现 NavigateToZone action server

**说明:** 在 mission_manager 节点上添加 NavigateToZone action server，内部使用 NavigateToPose action client。

**Files:**
- Modify: `src/mission_manager/mission_manager/mission_manager.py`
- Modify: `src/mission_manager/mission_manager/__init__.py`

- [ ] **Step 1: 重写 mission_manager.py**

```python
"""
Mission Manager Node

提供 NavigateToZone action server，封装 Nav2 NavigateToPose 航点发送。
"""
import os
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from mission_manager.action import NavigateToZone
from geometry_msgs.msg import PoseStamped, Quaternion
from ament_index_python.packages import get_package_share_directory

from mission_manager.waypoint_loader import load_waypoints


class MissionManager(Node):
    """提供 NavigateToZone action，内部封装 NavigateToPose。"""

    def __init__(self) -> None:
        super().__init__("mission_manager")

        # 加载航点
        map_dir = get_package_share_directory('at_nav2')
        map_path = os.path.join(map_dir, 'maps', 'map.yaml')
        self.waypoints = load_waypoints(map_path)
        self.get_logger().info(f'加载了 {len(self.waypoints)} 个航点')

        # Nav2 NavigateToPose action client
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # NavigateToZone action server
        self._action_server = ActionServer(
            self, NavigateToZone, "navigate_to_zone",
            execute_callback=self._execute_nav_cb
        )

        self.get_logger().info("Mission Manager 已启动")

    def _make_pose(self, x: float, y: float) -> PoseStamped:
        """将 (x, y) 坐标转为 PoseStamped。"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = Quaternion(w=1.0)
        return pose

    async def _execute_nav_cb(self, goal_handle):
        """NavigateToZone action 执行回调。"""
        zone_name = goal_handle.request.zone_name
        self.get_logger().info(f'收到导航请求: {zone_name}')

        if zone_name not in self.waypoints:
            goal_handle.abort()
            result = NavigateToZone.Result()
            result.success = False
            result.message = f'未知区域: {zone_name}'
            return result

        cx, cy = self.waypoints[zone_name]
        target_pose = self._make_pose(cx, cy)

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            result = NavigateToZone.Result()
            result.success = False
            result.message = 'NavigateToPose action server 不可用'
            return result

        self.get_logger().info(f'发送导航目标: {zone_name} -> ({cx:.2f}, {cy:.2f})')

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = target_pose

        future = self._nav_client.send_goal_async(nav_goal)
        goal_handle_nav = await future

        if not goal_handle_nav.accepted:
            goal_handle.abort()
            result = NavigateToZone.Result()
            result.success = False
            result.message = '导航目标被拒绝'
            return result

        # 等待导航结果，期间反馈距离
        result_future = goal_handle_nav.get_result_async()
        while not result_future.done():
            feedback_msg = NavigateToZone.Feedback()
            # 从 nav2 feedback 获取距离（在 done 前不可用，发 -1 表示导航中）
            feedback_msg.distance_remaining = -1.0
            goal_handle.publish_feedback(feedback_msg)
            await self._sleep(0.5)

        nav_result = result_future.result()
        goal_handle.succeed()

        result = NavigateToZone.Result()
        result.success = nav_result.result == 0  # nav2 返回 0 表示成功
        result.message = '导航完成' if result.success else f'导航失败, code={nav_result.result}'
        return result

    async def _sleep(self, seconds: float):
        await self._countdown(seconds)

    async def _countdown(self, seconds: float):
        from rclpy.task import Future
        fut = Future()
        self.create_timer(seconds, lambda: fut.set_result(None))
        await fut


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 更新 __init__.py**

```python
from mission_manager.mission_manager import MissionManager, main
from mission_manager.waypoint_loader import load_waypoints, get_zone_names

__all__ = ['MissionManager', 'main', 'load_waypoints', 'get_zone_names']
```

- [ ] **Step 3: 构建验证**

```bash
cd /home/funfun/AT_Atlas_nav_ws
colcon build --symlink-install --packages-select mission_manager
```

Expected: Build succeeds.

- [ ] **Step 4: Commit**

```bash
git add src/mission_manager/mission_manager/mission_manager.py src/mission_manager/mission_manager/__init__.py
git commit -m "feat(mission_manager): 实现 NavigateToZone action server"
```

---

### Task 8: competition_fsm — 创建包结构

**说明:** 创建 `competition_fsm` Python 包骨架，包含 FsmEvent.srv 定义。

**Files:**
- Create: `src/competition_fsm/package.xml`
- Create: `src/competition_fsm/CMakeLists.txt`
- Create: `src/competition_fsm/setup.py`
- Create: `src/competition_fsm/setup.cfg`
- Create: `src/competition_fsm/resource/competition_fsm`
- Create: `src/competition_fsm/srv/FsmEvent.srv`
- Create: `src/competition_fsm/competition_fsm/__init__.py`

- [ ] **Step 1: 创建 package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>competition_fsm</name>
  <version>0.0.1</version>
  <description>Competition finite state machine for manual/auto switching and task orchestration</description>
  <maintainer email="1219921425@qq.com">funfun</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>mission_manager</exec_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: 创建 CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.16)
project(competition_fsm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FsmEvent.srv"
)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}"
)

install(PROGRAMS
  scripts/competition_fsm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Step 3: 创建 FsmEvent.srv**

```yaml
# FsmEvent.srv
# 外部团队调用此 service 向 FSM 汇报事件
string event_type  # task_identified | pickup_complete | delivery_complete
string payload     # JSON 格式，task_identified 时包含 {"target_zones":["园区1B","园区2B"],"dispatch_zone":"待派送区B","object_type":"box_A"}
---
bool   accepted    # FSM 是否接受此事件
string message     # 拒绝原因（当前状态不允许此事件）
```

- [ ] **Step 4: 创建 setup.py**

```python
from setuptools import setup
from glob import glob

package_name = 'competition_fsm'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='funfun',
    maintainer_email='1219921425@qq.com',
    description='Competition FSM node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'competition_fsm = competition_fsm.fsm_node:main',
        ],
    },
)
```

- [ ] **Step 5: 创建 setup.cfg**

```
[develop]
script_dir=$base/lib/competition_fsm
[install]
install_scripts=$base/lib/competition_fsm
```

- [ ] **Step 6: 创建 resource/competition_fsm（ament 标记文件）**

空文件即可。

- [ ] **Step 7: 创建 competition_fsm/__init__.py**

空文件即可。

- [ ] **Step 8: 构建验证**

```bash
cd /home/funfun/AT_Atlas_nav_ws
colcon build --symlink-install --packages-select competition_fsm
```

Expected: Build succeeds, `FsmEvent.srv` 生成 Python 类。

- [ ] **Step 9: Commit**

```bash
git add src/competition_fsm/
git commit -m "feat(competition_fsm): 创建包骨架和 FsmEvent.srv 定义"
```

---

### Task 9: competition_fsm — 实现 FSM 状态机核心

**说明:** 实现状态枚举 + 字典驱动 handler + 状态转移逻辑。

**Files:**
- Create: `src/competition_fsm/competition_fsm/fsm.py`

- [ ] **Step 1: 实现 fsm.py**

```python
"""竞赛状态机核心：Enum 状态定义 + 字典驱动 handler。"""
from enum import Enum
import logging


class FsmState(Enum):
    MANUAL = "manual"
    GO_TRANSIT = "go_transit"
    AT_TRANSIT = "at_transit"
    GO_DISPATCH = "go_dispatch"
    GO_ZONE_1 = "go_zone_1"
    GO_ZONE_2 = "go_zone_2"
    MISSION_DONE = "mission_done"


class CompetitionFsm:
    """比赛任务状态机。

    状态转移由两件事驱动：
    1. 导航到达 (on_arrived)
    2. 外部 /fsm_event service 调用 (on_fsm_event)
    """

    def __init__(self, logger: logging.Logger) -> None:
        self.logger = logger
        self.state = FsmState.MANUAL
        self.task_payload: str = ""

        # 每个状态的 handler
        self._handlers = {
            FsmState.MANUAL:        self._handle_manual,
            FsmState.GO_TRANSIT:    self._handle_go_transit,
            FsmState.AT_TRANSIT:    self._handle_at_transit,
            FsmState.GO_DISPATCH:   self._handle_go_dispatch,
            FsmState.GO_ZONE_1:     self._handle_go_zone,
            FsmState.GO_ZONE_2:     self._handle_go_zone,
            FsmState.MISSION_DONE:  self._handle_mission_done,
        }

        # (event_type, current_state) -> next_state 转移表
        self._event_transitions = {
            ("task_identified",  FsmState.AT_TRANSIT):  FsmState.GO_DISPATCH,
            ("pickup_complete",  FsmState.GO_DISPATCH): FsmState.GO_ZONE_1,
            ("delivery_complete", FsmState.GO_ZONE_1):  FsmState.GO_ZONE_2,
            ("delivery_complete", FsmState.GO_ZONE_2):  FsmState.MISSION_DONE,
        }

        # 外部回调（由 fsm_node 注入）
        self._on_enter_state = None   # callable(state): 进入状态时调用（发导航目标等）
        self._on_leave_state = None   # callable(state): 离开状态时调用

    def switch_to(self, new_state: FsmState) -> None:
        """切换到新状态，执行 enter handler。"""
        old = self.state
        if old == new_state:
            return
        self.logger.info(f'[{old.value}] -> [{new_state.value}]')
        self.state = new_state
        handler = self._handlers.get(new_state)
        if handler:
            handler()

    def on_arrived(self) -> None:
        """导航到达的回调（由 fsm_node 在 action result 中调用）。"""
        self.logger.info(f'导航到达 (当前状态: {self.state.value})')

        # 只有 GO_TRANSIT 到达后自动转移
        if self.state == FsmState.GO_TRANSIT:
            self.switch_to(FsmState.AT_TRANSIT)
        # GO_DISPATCH, GO_ZONE_1, GO_ZONE_2 到达后等待外部 signal

    def on_fsm_event(self, event_type: str, payload: str = "") -> tuple[bool, str]:
        """处理外部 /fsm_event service 调用。

        Returns:
            (accepted, message) — 是否接受事件，拒绝原因
        """
        key = (event_type, self.state)
        next_state = self._event_transitions.get(key)

        if next_state is None:
            return False, f'状态 [{self.state.value}] 不接受事件 [{event_type}]'

        if event_type == "task_identified":
            self.task_payload = payload

        self.switch_to(next_state)
        return True, f'事件 [{event_type}] 已接受，转移到 [{next_state.value}]'

    # ── 各状态 handler ──

    def _handle_manual(self) -> None:
        self.logger.info('手动模式 — 等待切换指令')

    def _handle_go_transit(self) -> None:
        self.logger.info('导航到中转区')
        if self._on_enter_state:
            self._on_enter_state(FsmState.GO_TRANSIT)

    def _handle_at_transit(self) -> None:
        self.logger.info('等待任务识别...')

    def _handle_go_dispatch(self) -> None:
        self.logger.info('导航到待派送区')
        if self._on_enter_state:
            self._on_enter_state(FsmState.GO_DISPATCH)

    def _handle_go_zone(self) -> None:
        self.logger.info(f'导航到 {self.state.value}')
        if self._on_enter_state:
            self._on_enter_state(self.state)

    def _handle_mission_done(self) -> None:
        self.logger.info('全部任务完成!')
```

- [ ] **Step 2: 单元测试 — 验证状态转移逻辑**

```bash
python3 -c "
from competition_fsm.fsm import CompetitionFsm, FsmState
import logging
log = logging.getLogger('test')
f = CompetitionFsm(log)

# 测试 1: 初始状态
assert f.state == FsmState.MANUAL, f'Expected MANUAL, got {f.state}'

# 测试 2: 手动切换到 GO_TRANSIT
f.switch_to(FsmState.GO_TRANSIT)
assert f.state == FsmState.GO_TRANSIT

# 测试 3: 导航到达自动转 AT_TRANSIT
f.on_arrived()
assert f.state == FsmState.AT_TRANSIT

# 测试 4: 在 AT_TRANSIT 收到 task_identified
ok, _ = f.on_fsm_event('task_identified', '{\"target_zones\":[\"园区1B\"]}')
assert ok and f.state == FsmState.GO_DISPATCH

# 测试 5: 在错误状态收到拒绝
ok, msg = f.on_fsm_event('delivery_complete')
assert not ok
print('所有断言通过!')
"
```

Expected: `所有断言通过!`

- [ ] **Step 3: Commit**

```bash
git add src/competition_fsm/competition_fsm/fsm.py
git commit -m "feat(competition_fsm): 实现 FSM 状态机核心"
```

---

### Task 10: competition_fsm — 实现 fsm_node 主节点

**说明:** 实现 ROS2 节点，整合 FSM + cmd_vel 仲裁 + /fsm_event service + NavigateToZone action client。

**Files:**
- Create: `src/competition_fsm/competition_fsm/fsm_node.py`

- [ ] **Step 1: 实现 fsm_node.py**

```python
"""
Competition FSM Node

整合:
- FSM 状态机核心
- cmd_vel 仲裁（手动 vs Nav2）
- /fsm_event service（外部信号入口）
- NavigateToZone action client（调用 mission_manager）
"""
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from competition_fsm.srv import FsmEvent
from mission_manager.action import NavigateToZone

from competition_fsm.fsm import CompetitionFsm, FsmState


# zone_name 到 FsmState 的映射
ZONE_TO_STATE = {
    FsmState.GO_TRANSIT:  "中转区",
    FsmState.GO_DISPATCH: "待派送区",
    FsmState.GO_ZONE_1:   "园区1",
    FsmState.GO_ZONE_2:   "园区2",
}

# 需要等待外部 signal 的状态（到达后不自动转移）
WAIT_EXTERNAL_STATES = {FsmState.GO_DISPATCH, FsmState.GO_ZONE_1, FsmState.GO_ZONE_2}


class CompetitionFsmNode(Node):
    """竞赛 FSM ROS2 节点。"""

    def __init__(self) -> None:
        super().__init__("competition_fsm")

        self.fsm = CompetitionFsm(self.get_logger())
        self.fsm._on_enter_state = self._on_enter_state
        self.arrived = False  # 当前导航是否已完成

        # ── cmd_vel 仲裁 ──
        self._nav2_cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self._nav2_cmd_cb, 10)
        self._teleop_cmd_sub = self.create_subscription(
            Twist, "/teleop_cmd_vel", self._teleop_cmd_cb, 10)
        self._cmd_vel_pub = self.create_publisher(Twist, "/motor_cmd_vel", 10)
        self._last_teleop_time = self.get_clock().now()

        # 遥控超时定时器（1s 无消息 → 零速）
        self._teleop_watchdog = self.create_timer(0.2, self._teleop_watchdog_cb)

        # ── /fsm_event service ──
        self._fsm_srv = self.create_service(
            FsmEvent, "/fsm_event", self._on_fsm_event_cb)

        # ── /switch_mode topic ──
        self._switch_sub = self.create_subscription(
            String, "/switch_mode", self._on_switch_cmd, 10)

        # ── NavigateToZone action client ──
        self._nav_action_client = ActionClient(self, NavigateToZone, "navigate_to_zone")

        # ── 导航状态监控定时器 ──
        self._nav_timer = self.create_timer(0.5, self._nav_timer_cb)
        self._active_nav_goal = None

        self.get_logger().info("Competition FSM 已启动，当前: MANUAL")

    # ── 状态进入回调 ──

    def _on_enter_state(self, state: FsmState) -> None:
        """FSM 转移时触发：发送导航目标。"""
        self.arrived = False
        zone_name = ZONE_TO_STATE.get(state)
        if zone_name is None:
            return

        if not self._nav_action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(f'NavigateToZone action server 不可用')
            return

        goal = NavigateToZone.Goal()
        goal.zone_name = zone_name
        self.get_logger().info(f'发送导航目标: {zone_name}')
        future = self._nav_action_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('导航目标被 mission_manager 拒绝')
            return
        self._active_nav_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        result = future.result()
        self._active_nav_goal = None
        if result.result.success:
            self.get_logger().info('导航到达目标')
            self.arrived = True
            self.fsm.on_arrived()
        else:
            self.get_logger().warn(f'导航失败: {result.result.message}')

    def _nav_timer_cb(self):
        """周期性检查：在等待状态检查 arrived + 是否有外部 signal。"""
        pass  # external signal 由 service 回调触发，这里做超时等兜底逻辑

    # ── cmd_vel 仲裁 ──

    def _nav2_cmd_cb(self, msg: Twist) -> None:
        """Nav2 controller 的 cmd_vel。只在 GO_* 状态转发。"""
        if self.fsm.state.value.startswith("go_"):
            self._cmd_vel_pub.publish(msg)

    def _teleop_cmd_cb(self, msg: Twist) -> None:
        """遥控器 cmd_vel。只在 MANUAL 状态转发。"""
        self._last_teleop_time = self.get_clock().now()
        if self.fsm.state == FsmState.MANUAL:
            self._cmd_vel_pub.publish(msg)

    def _teleop_watchdog_cb(self) -> None:
        """遥控超时保护。"""
        if self.fsm.state != FsmState.MANUAL:
            return
        elapsed = self.get_clock().now() - self._last_teleop_time
        if elapsed.nanoseconds * 1e-9 > 1.0:
            self._cmd_vel_pub.publish(Twist())  # 零速

    # ── /switch_mode ──

    def _on_switch_cmd(self, msg: String) -> None:
        """接收模式切换指令。

        "auto" → 手动切换到自动（如果当前 MANUAL）
        "manual" → 紧急切回 MANUAL
        """
        cmd = msg.data.strip().lower()
        if cmd == "auto" and self.fsm.state == FsmState.MANUAL:
            # 先刹车 0.5s
            self._cmd_vel_pub.publish(Twist())
            self.create_timer(0.5, lambda: self.fsm.switch_to(FsmState.GO_TRANSIT))
            self.get_logger().info('切换: MANUAL -> AUTO')

        elif cmd == "manual" and self.fsm.state != FsmState.MANUAL:
            self._cmd_vel_pub.publish(Twist())
            self.fsm.switch_to(FsmState.MANUAL)
            self.get_logger().info('切换: AUTO -> MANUAL')

    # ── /fsm_event service ──

    def _on_fsm_event_cb(self, request, response):
        """外部团队调用的 service 回调。"""
        accepted, message = self.fsm.on_fsm_event(
            request.event_type, request.payload)
        response.accepted = accepted
        response.message = message
        self.get_logger().info(f'/fsm_event: {request.event_type} -> accepted={accepted} ({message})')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompetitionFsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 构建验证**

```bash
cd /home/funfun/AT_Atlas_nav_ws
colcon build --symlink-install --packages-select competition_fsm
```

Expected: Build succeeds.

- [ ] **Step 3: Commit**

```bash
git add src/competition_fsm/competition_fsm/fsm_node.py
git commit -m "feat(competition_fsm): 实现 FSM 主节点（cmd_vel 仲裁 + /fsm_event + action client）"
```

---

### Task 11: competition_fsm — 完善节点启动入口

**说明:** 创建 `scripts/competition_fsm_node` 脚本作为可执行入口。

**Files:**
- Create: `src/competition_fsm/scripts/competition_fsm_node`

- [ ] **Step 1: 创建启动脚本**

```python
#!/usr/bin/env python3
"""竞赛 FSM 节点启动入口。"""
import sys
from competition_fsm.fsm_node import main

if __name__ == "__main__":
    main()
```

确保脚本可执行：

```bash
chmod +x /home/funfun/AT_Atlas_nav_ws/src/competition_fsm/scripts/competition_fsm_node
```

- [ ] **Step 2: 构建验证（确认脚本安装）**

```bash
cd /home/funfun/AT_Atlas_nav_ws
colcon build --symlink-install --packages-select competition_fsm
ls -la install/competition_fsm/lib/competition_fsm/
```

Expected: 包含 `competition_fsm_node` 可执行文件。

- [ ] **Step 3: Commit**

```bash
git add src/competition_fsm/scripts/
git commit -m "feat(competition_fsm): 添加节点启动脚本"
```

---

### Task 12: robot_startup — 实现总 launch 文件

**说明:** 重写 `robot_start.launch.py`，编排所有节点的启动顺序。

**Files:**
- Rewrite: `src/robot_startup/launch/robot_start.launch.py`
- Modify: `src/robot_startup/package.xml`

- [ ] **Step 1: 更新 package.xml 添加依赖**

```xml
<exec_depend>at_nav2</exec_depend>
<exec_depend>mission_manager</exec_depend>
<exec_depend>competition_fsm</exec_depend>
<exec_depend>lslidar_driver</exec_depend>
<exec_depend>robot_description</exec_depend>
```

- [ ] **Step 2: 实现 robot_start.launch.py**

```python
"""总启动文件：启动雷达、导航栈、任务管理节点。"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # ── 1. robot_description (robot_state_publisher) ──
    robot_desc_dir = get_package_share_directory('robot_description')
    urdf_file = os.path.join(robot_desc_dir, 'urdf', 'omnibot.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
    )
    ld.add_action(robot_state_pub)

    # ── 2. LiDAR 驱动 ──
    lslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lslidar_driver'),
                         'launch', 'lslidar_n10p_uart.launch.py')
        )
    )
    ld.add_action(lslidar_launch)

    # ── 3. mission_manager ──
    mission_mgr = Node(
        package='mission_manager',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
    )
    ld.add_action(mission_mgr)

    # ── 4. competition_fsm ──
    fsm_node = Node(
        package='competition_fsm',
        executable='competition_fsm',
        name='competition_fsm',
        output='screen',
    )
    ld.add_action(fsm_node)

    # ── 5. at_nav2 (Cartographer + Nav2 bringup) ──
    # 延迟启动，等 /scan 和 /odom 就绪
    at_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('at_nav2'),
                         'launch', 'at_nav.launch.py')
        )
    )
    ld.add_action(TimerAction(period=3.0, actions=[at_nav_launch]))

    return ld
```

- [ ] **Step 3: Commit**

```bash
git add src/robot_startup/launch/robot_start.launch.py src/robot_startup/package.xml
git commit -m "feat(robot_startup): 实现总 launch 文件，编排所有节点"
```

---

### Task 13: 全工作区构建 & 验证

**说明:** 清理构建并确认所有包编译通过。

- [ ] **Step 1: 清理并构建全部包**

```bash
cd /home/funfun/AT_Atlas_nav_ws
colcon build --symlink-install
```

Expected: 所有 7 个包构建成功（包括新增的 competition_fsm）。

- [ ] **Step 2: 验证包可发现**

```bash
source /home/funfun/AT_Atlas_nav_ws/install/setup.bash
ros2 pkg list | grep -E "at_nav2|mission_manager|competition_fsm|robot_startup"
```

Expected: 列出 4 个包。

- [ ] **Step 3: 验证 FsmEvent service 接口**

```bash
source /home/funfun/AT_Atlas_nav_ws/install/setup.bash
ros2 interface show competition_fsm/srv/FsmEvent
```

Expected: 显示 service 定义。

- [ ] **Step 4: 验证 NavigateToZone action 接口**

```bash
source /home/funfun/AT_Atlas_nav_ws/install/setup.bash
ros2 interface show mission_manager/action/NavigateToZone
```

Expected: 显示 action 定义。

- [ ] **Step 5: 验证 launch 文件语法（不需要运行节点）**

```bash
source /home/funfun/AT_Atlas_nav_ws/install/setup.bash
ros2 launch robot_startup robot_start.launch.py --show-args 2>&1 | head
```

Expected: 显示 launch 参数列表（如可用），无语法错误。

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "chore: 全工作区构建验证通过"
```

---

## 验证清单

构建完成后，可以用以下方式在没有真实机器人时做模拟测试：

```bash
# 终端 1: 模拟 /scan
ros2 topic pub /scan sensor_msgs/msg/LaserScan "..."

# 终端 2: 模拟 /odom
ros2 topic pub /odom nav_msgs/msg/Odometry "..."

# 终端 3: 模拟 /teleop_cmd_vel（手动遥控）
ros2 topic pub /teleop_cmd_vel geometry_msgs/msg/Twist "..."

# 终端 4: 启动总launch
ros2 launch robot_startup robot_start.launch.py

# 终端 5: 测试 /fsm_event service
ros2 service call /fsm_event competition_fsm/srv/FsmEvent \
  "{event_type: 'task_identified', payload: '{\"target_zones\":[\"园区1B\"]}'}"

# 终端 6: 测试模式切换
ros2 topic pub /switch_mode std_msgs/msg/String "{data: 'auto'}"
```
