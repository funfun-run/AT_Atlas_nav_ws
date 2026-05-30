# Gazebo 仿真环境设计

## 目标

为 `robot_description` 包中的 URDF 机器人模型搭建 Gazebo 仿真环境，用于导航（Nav2）的离线测试与开发。

## 范围

- 仿真范围：**仅移动底盘**（steering-wheel 驱动 + 2D LiDAR），机械臂固定不动
- 控制方式：`/cmd_vel` 抽象控制（planar_move_plugin），不进行关节级控制
- 传感器：Gazebo 模拟 2D LiDAR，发布 `/scan`
- 里程计：Gazebo 内置 odometry，发布 `/odom` + `odom→base_footprint` TF
- 环境：基于现有 `at_nav2/maps/map.yaml` 的比赛地图，手动放置墙体
- 碰撞：STL 网格替换为简化几何体（box/cylinder）

## 非目标

- 不修改 `robot_description` 包
- 不修改 `at_nav2` 及其他任何现有功能包
- 所有修改仅限新建的 `robot_gazebo` 包
- 不仿真机械臂运动和控制
- 不仿真 RGB-D 相机或其他传感器
- 不做硬件在环（HIL）仿真

## 架构

```
robot_description (不改动)
       │
       │  参考 link/joint 定义
       ▼
robot_gazebo (新包)
  ├── urdf/robot_sim.xacro        ← 模型定义 + Gazebo 插件
  ├── launch/gazebo_sim.launch.py ← 启动文件
  ├── worlds/competition.world    ← 比赛场地
  ├── CMakeLists.txt
  └── package.xml
```

## 包设计

### robot_gazebo

新包，不依赖 `robot_description` 进行构建（仅参考其内容），包含仿真所需的全部资源。

**package.xml 依赖：**
- `ament_cmake`（buildtool）
- `gazebo_ros`
- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`

**CMakeLists.txt：**
- `install(DIRECTORY urdf launch worlds DESTINATION share/${PROJECT_NAME})`

## robot_sim.xacro 设计

### 结构

`<robot name="robot_description">` 根元素，包含两大块：

1. **link/joint 定义** — 手动从 `robot_description.urdf` 复制 link/joint 内容到本文件（不引用，保持独立）。link 树保持不变：

   > **注意：** 如果 `robot_description.urdf` 后续有改动，需手动同步到 `robot_sim.xacro`。由于 URDF 已基本稳定（SolidWorks 导出），同步频率很低。
   ```
   base_footprint → base_link → laser (LiDAR)
                             → arm0_Link → arm1_Link → arm2_Link → arm3_Link → arm4_Link (固定)
                             → LFs_Link → LFd_Link (左前轮：steering → drive)
                             → LRs_Link → LRd_Link (左后轮：steering → drive)
                             → RFs_Link → RFd_Link (右前轮：steering → drive)
                             → RRs_Link → RRd_Link (右后轮：steering → drive)
   ```

2. **`<gazebo>` 插件标签** — 碰撞简化、planar_move、ray_sensor、odometry

### 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `base_joint` Z offset | 0.033 m | wheel radius，保持原 URDF 值 |
| `robot_radius` | 0.35 m | 用于碰撞体尺寸估算 |
| laser scan range | 0.15–30.0 m | 模拟 LSN10P 规格 |
| laser samples | 360 | 1° 角分辨率 |
| laser update_rate | 10 Hz | |
| laser frame_id | `laser` | 与原 lslidar_driver 对齐 |

## Gazebo 插件配置

### 1. Planar Move Plugin（底盘控制）

- **插件文件：** `libgazebo_ros_planar_move.so`
- **订阅话题：** `/cmd_vel`（`geometry_msgs/Twist`）
- **发布话题：** `/odom`（`nav_msgs/Odometry`）
- **TF 发布：** `odom → base_footprint`
- **功能：** 接收 cmd_vel(vx, vy, vz)，内部解算为 8 个 wheel joint 的 steering/drive 位置，发布里程计

```
/cmd_vel → [planar_move_plugin] → wheel joints + /odom + odom→base_footprint TF
```

### 2. Ray Sensor Plugin（2D LiDAR）

- **插件文件：** `libgazebo_ros_ray_sensor.so`
- **参考 link：** `laser`
- **发布话题：** `/scan`（`sensor_msgs/LaserScan`），通过 remap `/out:=/scan`
- **frame_id：** `laser`
- **扫描配置：** 360 样本，±π 弧度，30m 量程，10Hz

### 3. Joint State Publisher

- Gazebo 内置，发布 `/joint_states`
- `robot_state_publisher` 消费 `/joint_states` 发布 TF（base_footprint→base_link→laser 及各 wheel link）

## 碰撞简化

| Link | 原碰撞 | 简化后 | 尺寸估算 |
|------|--------|--------|----------|
| `base_link` | STL mesh | `<box>` | 0.6 × 0.5 × 0.25 m |
| `laser` | STL mesh | `<box>` | 0.07 × 0.07 × 0.07 m |
| `LFs_Link`, `LRs_Link`, `RFs_Link`, `RRs_Link` | STL mesh | `<cylinder>` | radius 0.02, length 0.04 m |
| `LFd_Link`, `LRd_Link`, `RFd_Link`, `RRd_Link` | STL mesh | `<cylinder>` | radius 0.033, length 0.02 m |
| `arm0_Link` ~ `arm4_Link` | STL mesh | 禁用碰撞 | 机械臂不参与仿真 |

方式：在 `<gazebo reference="...">` 标签内设置 `<collision>`，覆盖原始 URDF 的 collision 定义。

## World 文件设计

### competition.world

- **格式：** SDF 1.6
- **内容：**
  1. 地面平面（20m × 20m 灰色平面）
  2. 墙体模型（`<static>true`），手动对照 map.yaml 的 zone 边界放置 `<box>` 墙体（高度 1.0m）
  3. 定向光源
  4. 物理引擎：ODE（默认）

### 墙体放置策略

根据 `map.yaml` 中 `no_go_zone` 类型的 zone polygon，手动转换为 SDF box model。每个 zone 用一个或多个 box 近似其边界。

示例：待派送区B（no_go_zone，polygon: [3.44, 0.72] → [3.88, 1.12]）

```xml
<model name="wall_待派送区B">
  <static>true</static>
  <pose>3.66 0.92 0.5 0 0 0</pose>
  <link name="body">
    <collision>
      <geometry><box><size>0.44 0.40 1.0</size></box></geometry>
    </collision>
    <visual>
      <geometry><box><size>0.44 0.40 1.0</size></box></geometry>
      <material><ambient>0.6 0.6 0.6 1</ambient></material>
    </visual>
  </link>
</model>
```

地图尺寸：4m × 4m（200 px × 0.02 m/px 分辨率），world 中坐标与 map.yaml 坐标系对齐。

## Launch 文件设计

### gazebo_sim.launch.py

启动顺序：

1. 启动 Gazebo server，加载 `competition.world`
2. 通过 xacro 处理 `robot_sim.xacro`，得到 `robot_description` 参数字符串
3. 启动 `robot_state_publisher` 节点（从 `/joint_states` 计算 TF）
4. 调用 `spawn_entity.py` 将机器人 spawn 到 Gazebo（初始位姿 origin）

参数：
- `world_path`：world 文件路径（默认指向 `share/robot_gazebo/worlds/competition.world`）
- `x_pos`, `y_pos`, `z_pos`：spawn 位置（默认 0, 0, 0.1）
- `use_sim_time`：强制 `true`

### 使用方式

```bash
# 仅启动仿真
ros2 launch robot_gazebo gazebo_sim.launch.py

# 仿真 + 导航
# terminal 1
ros2 launch robot_gazebo gazebo_sim.launch.py
# terminal 2
ros2 launch at_nav2 at_nav.launch.py
```

## 话题对比：仿真 vs 真车

| 话题 | 仿真来源 | 真车来源 |
|------|----------|----------|
| `/scan` | Gazebo ray_plugin | `lslidar_driver` 节点 |
| `/odom` | planar_move 内置 odometry | 底盘 odom_driver |
| `odom→base_footprint` TF | planar_move 发布 | odom_driver 发布 |
| `/cmd_vel` 消费者 | planar_move 插件（Gazebo 内部） | 电控组控制器（硬件） |
| `/joint_states` | Gazebo 发布 | `robot_state_publisher`（静态 joint） |
| `base_footprint→…` TF | `robot_state_publisher` | `robot_state_publisher` |

## 验证标准

1. `ros2 launch robot_gazebo gazebo_sim.launch.py` 成功启动，Gazebo 窗口显示机器人和环境
2. `/scan` 话题有 LaserScan 数据发布，`ros2 topic echo /scan` 可见
3. `/odom` 话题有里程计数据，`odom→base_footprint` TF 存在
4. 发布 `/cmd_vel` 可以使机器人在 Gazebo 中移动
5. Nav2 可以基于仿真环境进行路径规划和导航
