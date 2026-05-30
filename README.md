# AGT — 竞赛机器人导航系统

基于 ROS2 Humble + Nav2 + Cartographer，搭载 Leishen LSN10P 2D LiDAR 的自主导航竞赛机器人。

## 硬件

| 组件 | 型号 |
|------|------|
| 底盘 | 舵轮底盘（全向运动，vx/vy/wz 接口） |
| LiDAR | Leishen LSN10P（180° FOV，0.25° 分辨率，10Hz） |
| 定位 | Cartographer pure localization（预建地图） |

## 比赛流程

```
手动遥控 → 进入启动区 → 按钮/语音切换 → 全自动任务:
  中转区(读取任务) → 待派送区(取货) → 园区1(派送) → 园区2(派送) → 完成
```

## 包结构

| 包 | 语言 | 说明 |
|----|------|------|
| `robot_description` | URDF + RViz2 | 机器人模型 + RViz 可视化配置（`rviz2/rviz.rviz`），启动即加载 |
| `lslidar_driver` | C++ | Leishen 2D 雷达驱动 → `/scan` |
| `lslidar_msgs` | ROS2 msg/srv | 雷达自定义消息 |
| `at_nav2` | YAML/Lua/Python | Nav2 配置 + Cartographer 定位 + 比赛地图 |
| `mission_manager` | Python 3 | `NavigateToZone` action server + 航点加载 |
| `competition_fsm` | Python 3 | 比赛状态机 + cmd_vel 仲裁 + `/fsm_event` service |
| `robot_startup` | Python | 顶层 bringup launch |

### 架构

```
robot_description ──► robot_startup (总 launch)
                                │
        ┌───────────────────────┼────────────────────────┐
        ▼                       ▼                        ▼
 lslidar_driver             at_nav2            competition_fsm
      │                  (Carto + Nav2)         (状态机)
      ▼                       │                        │
 lslidar_msgs           mission_manager       摄像头/机械臂
                       (NavigateToZone)        (其他团队)
```

## TF 树

```
map ──► odom ──► base_footprint ──► base_link ──► laser_frame
(Carto)  (底盘)     (地面投影)      (轮轴高度)    (URDF 固定关节)
                        ↑ ~33 mm (轮半径)
```

## 构建

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 运行

```bash
# 仅启动机器人模型 + RViz2（调试 URDF / TF 树）
ros2 launch robot_description robot_description.launch.py

# 完整导航启动
ros2 launch robot_startup robot_start.launch.py

# 手动→自动模式切换
ros2 topic pub /switch_mode std_msgs/msg/String "{data: 'auto'}"

# 模拟外部事件（摄像头团队识别任务后）
ros2 service call /fsm_event competition_fsm/srv/FsmEvent \
  "{event_type: 'task_identified', payload: '{\"target_zones\":[\"园区1B\",\"园区2B\"]}'}"

# 模拟机械臂完成取货
ros2 service call /fsm_event competition_fsm/srv/FsmEvent \
  "{event_type: 'pickup_complete', payload: ''}"
```

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `robot_radius` | 0.35 m | [HW_CONFIG] 机器人外接圆半径 |
| `desired_linear_vel` | 0.3 m/s | [HW_CONFIG] 期望线速度 |
| `max_velocity` | [0.5, 0.5, 1.0] | [HW_CONFIG] vx/vy/vth 全向 |
| `xy_goal_tolerance` | 0.25 m | [HW_CONFIG] 到达容差 |
| `lookahead_dist` | 0.6 m | [HW_CONFIG] 前视距离 |
| 定位方式 | Cartographer | pure localization 模式 |
| 控制器 | RegulatedPurePursuit | |
| LiDAR topic | `/scan` | LaserScan |

> **[HW_CONFIG]** 标注的参数需根据实际机器人调整。搜索 `grep -rn "\[HW_CONFIG\]" src/` 定位。

## 开发进度

- [x] 雷达驱动
- [x] 自定义消息/服务
- [x] Nav2 参数配置 + 行为树
- [x] 比赛地图 + 区域定义
- [x] 项目架构设计文档
- [x] 实现计划（13 任务全部完成）
- [x] Cartographer 配置
- [x] mission_manager 增强（NavigateToZone action + waypoint_loader）
- [x] competition_fsm 包（状态机 + cmd_vel 仲裁 + /fsm_event service）
- [x] robot_startup 总 launch

## 部署前待办

详见 [`docs/TODO-before-deployment.md`](docs/TODO-before-deployment.md)。关键项：
- Zone 名称与 map.yaml 对齐
- LiDAR `frame_id` 与 URDF 统一
- 生成 `.pbstream` 地图
- 接入里程计 `/odom`
