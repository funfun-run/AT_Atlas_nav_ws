# 部署前待办清单

> 生成日期：2026-05-27 | 基于当前项目状态

## 🔴 致命问题（不解决系统跑不起来）

### 1. Zone 名称不匹配

**问题：** FSM 状态与 zone 名的映射不匹配 map.yaml 中的实际区域名。

| FSM 当前使用 | map.yaml 中存在 |
|---|---|
| `"中转区"` | `"中转区B"` / `"中转区"` (无后缀的只有一个) |
| `"待派送区"` | `"待派送区B"` / `"待派送区A"` |
| `"园区1"` | `"园区1B"` / `"园区1A"` |
| `"园区2"` | `"园区2B"` / `"园区2A"` |

`waypoint_loader` 做精确名字匹配，查不到 zone 名导航直接失败。

**修改位置：** `src/competition_fsm/competition_fsm/fsm_node.py` — `ZONE_TO_STATE` 字典

**操作：** 确认比赛走 A 侧还是 B 侧，将 zone 名改为 map.yaml 中实际的名字。

---

### 2. LiDAR frame_id 与 URDF 不统一

**问题：** 雷达驱动配置 `frame_id: "laser"`，URDF 中 link 名为 `laser_frame`。TF 树里是 `laser_frame`，但 `/scan` 消息 header 写的是 `laser`。Cartographer 和 Nav2 costmap 做 TF lookup 时会失败。

**修改位置（二选一）：**
- 方案 A：`src/lslidar_driver/config/lslidar_n10p_uart.yaml` — 改 `frame_id: "laser"` → `frame_id: "laser_frame"`
- 方案 B：`src/robot_description/urdf/omnibot.urdf` — 改 `<link name="laser_frame">` → `<link name="laser">`，同步改 joint 的 `<child link="laser_frame"/>` → `<child link="laser"/>`

**建议：** 选方案 A，只动雷达配置，不影响 URDF 和其他依赖。

---

### 3. 缺少 .pbstream 地图文件

**问题：** Cartographer pure localization 模式需要预建的 `.pbstream` 地图。当前 `at_nav2/maps/` 下只有 `map.pgm` + `map.yaml`。

**操作：**
1. 在真实机器人上跑一轮 Cartographer SLAM 建图
2. 将生成的 `.pbstream` 保存到 `src/at_nav2/maps/map.pbstream`
3. 确认 `at_nav.launch.py` 中的 `pbstream_file` 路径指向正确

**参考命令：**
```bash
# 先以建图模式启动 Cartographer（修改 lua 配文件 pure_localization = false）
# 遥控走一遍全场
# 保存地图
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '/path/to/map.pbstream'}"
```

---

### 4. 里程计 `/odom` 缺失

**问题：** 设计上 `odom_driver` 由其他团队实现。在此之前，没有人发布 `/odom` 话题和 `odom → base_link` TF。Cartographer 和 Nav2 依赖这两个输入，TF 树会断链。

**操作：**
- 与底盘团队确认 `/odom` (nav_msgs/Odometry) 和 `odom → base_link` TF 的交付时间
- 在交付前，可用以下命令 mock 测试：

```bash
# 终端 1：模拟 odom → base_link TF
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

# 终端 2：模拟 /odom 话题
ros2 topic pub /odom nav_msgs/msg/Odometry "{header: {frame_id: 'odom'}, child_frame_id: 'base_link', pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, twist: {twist: {linear: {x: 0.0}, angular: {z: 0.0}}}}" -r 10
```

---

## 🟡 重要但可后补

### 5. /switch_mode 物理触发

**问题：** 当前模式切换通过 `ros2 topic pub /switch_mode std_msgs/msg/String` 模拟。需要实际的物理按钮或语音识别模块触发。

**接口规格（供硬件团队）：**
- Topic: `/switch_mode`
- 类型: `std_msgs/String`
- 取值: `"auto"` — 切换到自动 / `"manual"` — 紧急切回手动

---

### 6. /fsm_event service 对接

**问题：** 摄像头团队和机械臂团队需要知道如何调用 `/fsm_event` service。

**接口规格（供对方团队）：**
- Service: `/fsm_event`
- 类型: `competition_fsm/srv/FsmEvent`

```yaml
# 请求
string event_type  # "task_identified" | "pickup_complete" | "delivery_complete"
string payload     # JSON, 仅 task_identified 时需要
                   # 格式: {"target_zones":["园区1B","园区2B"],"dispatch_zone":"待派送区B","object_type":"box_A"}

# 响应
bool   accepted    # FSM 是否接受（false = 当前状态不允许此事件）
string message     # 拒绝原因
```

**联调流程：**
1. 双方独立用 `ros2 service call` 模拟对方先跑通
2. 确认 event_type 枚举和 payload JSON 格式
3. 联调

---

### 7. FSM zone 名需确认 A/B 侧

**问题：** map.yaml 有两套对称区域（A 侧 / B 侧）。比赛走哪一侧，`ZONE_TO_STATE` 就要对应改。当前 `waypoint_loader` 只匹配 `task_area` 和 `start_area`，过滤掉了 `no_go_zone`。

**操作：** 确认赛制后，检查 `ZONE_TO_STATE` 中的名字是否存在于 map.yaml 的 `task_area` 列表中。

```bash
# 快速检查当前哪些 zone 可被 Waypoint_loader 匹配
python3 -c "
from mission_manager.waypoint_loader import load_waypoints
wps = load_waypoints('src/at_nav2/maps/map.yaml')
print('可用 zone:', sorted(wps.keys()))
"
```

---

### 8. Cartographer 配置中的 IMU 设置

**问题：** 当前 `cartographer_localization.lua` 假设无 IMU (`use_imu_data = false`)。如果机器人有 IMU，需要改为 `true` 以提升定位精度。

**修改位置：** `src/at_nav2/config/cartographer_localization.lua`

---

## 🟢 优化项

### 9. Behavior Tree 无恢复行为

**问题：** 当前 BT 只有 `ComputePathToPose → FollowPath`。导航失败（碰撞、卡住）时不会自动恢复。

**改进方案：** 在 `bt_navigator.xml` 中加入 recovery subtree（backup、spin、clear costmap）。

---

### 10. URDF 仅含底盘圆柱体

**问题：** URDF 缺少车轮、IMU link 等。不影响功能，只影响 RViz 可视化效果。

**操作：** 可选，有需要时完善。

---

## 快速验证命令

```bash
# 检查 zone 名匹配情况
python3 -c "
from mission_manager.waypoint_loader import load_waypoints
wps = load_waypoints('src/at_nav2/maps/map.yaml')
targets = ['中转区', '待派送区', '园区1', '园区2']
for t in targets:
    print(f'{t}: {\"✅\" if t in wps else \"❌ 不存在\"} 可用: {[k for k in wps if t in k]}')
"

# 检查 frame_id 一致性
echo "URDF link:" && grep 'link name="laser' src/robot_description/urdf/omnibot.urdf
echo "LiDAR frame_id:" && grep 'frame_id' src/lslidar_driver/config/lslidar_n10p_uart.yaml

# 检查 .pbstream 是否存在
ls -la src/at_nav2/maps/map.pbstream 2>/dev/null && echo "✅ 存在" || echo "❌ 缺失"
```
