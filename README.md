# Omnibot — ROS2 差分驱动机器人导航系统

基于 ROS2 Humble + Nav2，搭载 Leishen LSN10P 2D LiDAR 的自主导航机器人。

## 硬件

| 组件 | 型号 |
|------|------|
| 底盘 | 差分驱动（两轮） |
| LiDAR | Leishen LSN10P（180° FOV，0.25° 分辨率，10Hz） |
| 通信 | 串口（UART）/ 以太网 |

## 包结构

| 包 | 语言 | 说明 |
|----|------|------|
| `robot_description` | URDF | 机器人模型，`base_link` + `laser_frame` 固定关节 |
| `lslidar_driver` | C++ | Leishen 雷达驱动（X10/CH/CX/LS 系列），多线程，支持串口/网口 |
| `lslidar_msgs` | ROS2 msg/srv | 雷达自定义消息（2 msg + 12 srv） |
| `at_nav2` | YAML | Nav2 配置：NavfnPlanner + DWB + AMCL + costmap |
| `map_server` | — | `nav2_map_server` 包装，待添加地图文件 |
| `mission_manager` | Python 3 | `NavigateToPose` action client，带反馈回调 |
| `odom_driver` | C++ | 底盘里程计（骨架，待实现） |
| `robot_startup` | Python | 顶层 bringup launch（开发中） |

### 依赖关系

```
robot_description ──► robot_startup ──┬── lslidar_driver ──► lslidar_msgs
                                      ├── at_nav2 ──► mission_manager
                                      └── odom_driver
```

## 构建

```bash
# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 构建
colcon build --symlink-install

# 加载环境
source install/setup.bash
```

## 运行

```bash
# 启动完整导航（待完善）
ros2 launch robot_startup robot_start.launch.py

# 单独启动雷达
ros2 launch lslidar_driver lsn10p_launch.py

# 启动任务管理器
ros2 run mission_manager mission_manager
```

## TF 树

```
map ──► odom ──► base_link ──► laser_frame
(AMCL)   (driver)   (URDF)      (URDF fixed joint)
```

## 关键 Nav2 参数

| 参数 | 值 |
|------|-----|
| robot_radius | 0.22 m |
| max_vel_x / max_vel_y | 0.26 m/s |
| max_vel_theta | 1.0 rad/s |
| inflation_radius | 0.55 m |
| xy_goal_tolerance | 0.25 m |
| yaw_goal_tolerance | 0.25 rad |

## 开发状态

- [x] 雷达驱动（完整实现）
- [x] 自定义消息/服务
- [x] Nav2 参数配置
- [x] 任务管理器
- [ ] 里程计驱动（待实现）
- [ ] 顶层 bringup launch（开发中）
- [ ] 地图文件
- [ ] lidar frame_id 与 URDF 统一（当前 `laser` vs `laser_frame` 不一致）
