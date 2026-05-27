# 竞赛机器人导航架构设计

日期: 2026-05-27

## 背景

参加物流搬运类比赛，机器人需要在手动遥控和全自动导航两种模式间切换，按固定顺序完成搬运任务。机器人使用 2D LiDAR (Leishen LSN10P)，已有预建地图，摄像头和机械臂由其他团队负责。

## 架构概览

采用分层架构（方案 B）：比赛任务逻辑与导航逻辑分离。

```
robot_startup (总 launch)
  ├── lslidar_driver    → /scan
  ├── odom_driver       → /odom + odom→base_link TF (其他团队实现)
  ├── cartographer      → map→odom TF (pure localization)
  ├── at_nav2 (Nav2)    → /nav2_cmd_vel
  ├── mission_manager   → NavigateToPose action 封装
  └── competition_fsm   → 状态机 + cmd_vel 仲裁 + /fsm_event service
```

### 包职责

| 包 | 状态 | 职责 |
|---|---|---|
| `robot_description` | 保持 | URDF 模型 |
| `lslidar_driver` | 保持 | 2D LiDAR 驱动 → `/scan` |
| `lslidar_msgs` | 保持 | 雷达自定义消息 |
| `odom_driver` | 其他团队 | 底盘串口 + 里程计 → `/odom` + `odom→base_link` TF |
| `at_nav2` | 重写 | Cartographer 配置 + Nav2 参数 + BT + 地图 + launch |
| `mission_manager` | 增强 | Nav2 action 封装，支持从 map.yaml 加载航点 |
| `competition_fsm` | **新增** | 比赛状态机 + cmd_vel 仲裁 + 外部信号接收 |
| `robot_startup` | 实现 | 总 launch 文件 |

## Cartographer

- **模式**: pure localization（纯定位，不建图）
- **输入**: `/scan` (LaserScan) + `/odom` (Odometry)
- **输出**: `map → odom` TF
- **配置**: `at_nav2/config/cartographer_localization.lua`，加载预建的 `.pbstream` 地图

## cmd_vel 仲裁

FSM 是 `/cmd_vel` 的唯一发布者：

| 状态 | 转发来源 |
|---|---|
| `MANUAL` | `/teleop_cmd_vel` → `/cmd_vel` |
| `GO_*` | `/nav2_cmd_vel` → `/cmd_vel` |
| `AT_*` / `MISSION_DONE` | 零速 |

Nav2 controller_server 的 cmd_vel 需 remap 到 `/nav2_cmd_vel`。

安全保护：
- MANUAL → AUTO 切换时先发零速刹车 0.5s
- 遥控器超时 1s 自动发零速
- AUTO → MANUAL 紧急切回立即生效

## 状态机设计

### 状态枚举

```
MANUAL → GO_TRANSIT → AT_TRANSIT → GO_DISPATCH → GO_ZONE_1 → GO_ZONE_2 → MISSION_DONE
```

### 转移条件

| 当前状态 | 触发条件 | 下一状态 |
|---|---|---|
| MANUAL | 收到 switch_cmd | GO_TRANSIT |
| GO_TRANSIT | 导航到达 | AT_TRANSIT |
| AT_TRANSIT | /fsm_event: task_identified | GO_DISPATCH |
| GO_DISPATCH | 导航到达 + /fsm_event: pickup_complete | GO_ZONE_1 |
| GO_ZONE_1 | 导航到达 + /fsm_event: delivery_complete | GO_ZONE_2 |
| GO_ZONE_2 | 导航到达 + /fsm_event: delivery_complete | MISSION_DONE |
| 任意 | 紧急停止指令 | MANUAL |

### GO_* 状态行为

1. 进入状态 → 调用 mission_manager 发送导航目标
2. 等待导航结果（成功=到达，失败=重试或告警）
3. 到达后 → 等待外部信号（除 GO_TRANSIT 直接自动转移）

## 外部接口

### /fsm_event Service

```yaml
# FsmEvent.srv
string event_type   # task_identified | pickup_complete | delivery_complete
string payload      # JSON, task_identified 时包含任务详情
---
bool   accepted     # FSM 是否接受
string message      # 拒绝原因
```

单一 service 替代多 topic 方案，同步调用避免信号丢失。

### 任务信息 payload 格式

```json
{
  "target_zones": ["园区1B", "园区2B"],
  "dispatch_zone": "待派送区B",
  "object_type": "box_A"
}
```

## 实现方式

### 状态机 (competition_fsm/fsm.py)

- Python Enum 定义状态 + dict 驱动 handler
- 约 150 行，无外部依赖
- 状态转移表：`(event_type, current_state) → next_state`

### Nav2 配置变更 (at_nav2)

- local_costmap obstacle_layer 改订阅 `/scan` (LaserScan)，删除 PointCloud2 配置
- controller_server remap: `cmd_vel` → `/nav2_cmd_vel`
- 新建 `cartographer_localization.lua` 配置文件
- launch 中加入 Cartographer 节点

### mission_manager 增强

- 从 `map.yaml` 加载 zones，解析各区域坐标中心点
- 封装 `send_waypoint(zone_name)` 方法供 FSM 调用

## TF 树

```
map → odom → base_link → laser_frame
      ↑        ↑            ↑
  Cartographer odom_driver  URDF static
```

## 不在此范围内

- odom_driver 实现（其他团队）
- 摄像头任务识别逻辑（其他团队）
- 机械臂取货/投递逻辑（其他团队）
