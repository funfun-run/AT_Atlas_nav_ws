# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Development Commands

```bash
# Build the workspace (from workspace root)
colcon build --symlink-install

# Build only one package
colcon build --symlink-install --packages-select <package_name>

# Source the workspace
source install/setup.bash

# Run linters on a package
colcon test --packages-select <package_name>
cat build/<package_name>/test_results/<package_name>/linter_results.txt

# Run a single Python node directly (after build)
ros2 run mission_manager mission_manager
```

## Architecture

This is a ROS2 (Humble) colcon workspace for the **Omnibot** — a differential-drive robot using Nav2 for autonomous navigation with a Leishen LSN10P 2D LiDAR.

### Package Dependency Graph

```
robot_description ──► robot_startup (top-level bringup)
                                 │
           ┌─────────────────────┼─────────────────────┐
           ▼                     ▼                     ▼
    lslidar_driver           at_nav2             odom_driver
   (LSN10 LiDAR)         (Nav2 bringup)         (chassis/odo)
           │                     │
           ▼                     ▼
    lslidar_msgs          mission_manager
  (custom msg/srv)       (Python, Nav2 action client)
```

### Package Details

- **`robot_description`** — URDF model. Single `base_link` cylinder (r=0.2m) with a fixed `laser_frame` joint (offset x=0.1m, z=0.15m). No wheels in URDF. Installs URDF to share.
- **`lslidar_driver`** — Fully implemented C++ Leishen LiDAR driver (X10/CH/CX/LS series). Multi-threaded executor, supports both UART and Ethernet. Config at `config/lslidar_n10p_uart.yaml` and `config/lslidar_n10p_net.yaml`. Publishes `sensor_msgs/PointCloud2` to `lslidar_point_cloud` and (optionally) `sensor_msgs/LaserScan` to `/scan`. Key deps: PCL, libpcap, Boost.
- **`lslidar_msgs`** — Custom ROS2 messages and services for lslidar_driver. Defines `LslidarPacket.msg`, `LslidarInformation.msg`, and 12 service definitions (motor control, power control, time mode, frame rate, etc.).
- **`odom_driver`** — C++ node for chassis serial communication and odometry publishing (skeleton — source commented out). Expected to publish `odom`→`base_link` transform and `nav_msgs/Odometry`.
- **`at_nav2`** — Nav2 bringup config package. The single `config/nav2_params.yaml` defines:
  - **Planner**: `nav2_navfn_planner/NavfnPlanner` (Dijkstra, not A*)
  - **Controller**: `dwb_core::DWBLocalPlanner` with 7 critics, max linear/angular = 0.26 m/s, 1.0 rad/s
  - **Localization**: AMCL with likelihood_field model, 500–2000 particles, differential drive model
  - **Costmaps**: global (map frame, 1Hz) and local (odom frame, rolling 3m×3m, 5Hz), both obstacle layer subscribed to `/scan`
  - **Behavior Tree**: uses default Nav2 BT plugins (no custom XML; `bt_xml_filename` is empty, Nav2 fallback behavior applies)
- **`map_server`** — Wrapper around `nav2_map_server`. `maps/` directory exists but currently empty (map file to be added).
- **`mission_manager`** (Python) — A `MissionManager` node that wraps `NavigateToPose` action client with feedback/goal-response/result callbacks. Entry point: `mission_manager` console script.
- **`robot_startup`** — Intended top-level bringup launch package. `launch/robot_start.launch.py` is a minimal stub (work in progress).

### TF Tree (expected)

```
map ──► odom ──► base_link ──► laser_frame
(AMCL)   (driver)   (URDF)      (URDF fixed joint)
```

**Note:** The LiDAR driver config sets `frame_id: "laser"` while the URDF defines the link as `laser_frame`. This mismatch needs to be resolved before Nav2 can consume scan data correctly.

### Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| `robot_radius` | 0.22 m | In both costmaps |
| `controller_frequency` | 20 Hz | DWB control loop |
| `max_vel_x` / `max_vel_y` | 0.26 m/s | Differential drive |
| `max_vel_theta` | 1.0 rad/s | |
| `inflation_radius` | 0.55 m | 2.5× robot radius |
| `xy_goal_tolerance` | 0.25 m | |
| `yaw_goal_tolerance` | 0.25 rad | |
