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

# Run a single Python node directly (after build)
ros2 run mission_manager mission_manager
ros2 run competition_fsm competition_fsm_node

# Test FSM state transitions
python3 -c "
from competition_fsm.fsm import CompetitionFsm, FsmState
import logging
f = CompetitionFsm(logging.getLogger('test'))
f.switch_to(FsmState.GO_TRANSIT)
f.on_arrived()
assert f.state == FsmState.AT_TRANSIT
print('FSM test passed')
"
```

## Architecture

This is a ROS2 (Humble) colcon workspace for **AGT** — a holonomic (steering-wheel) competition robot using Nav2 for autonomous navigation with a Leishen LSN10P 2D LiDAR and Cartographer for localization.

### Competition Flow

```
Manual remote control → Enter start zone → Button/voice switch → Autonomous task sequence:
  中转区 (read task) → 待派送区 (pickup) → 园区1 (deliver) → 园区2 (deliver) → Done
```

### Package Dependency Graph

```
robot_description ──► robot_startup (top-level bringup)
                                 │
         ┌───────────────────────┼────────────────────────┐
         ▼                       ▼                        ▼
  lslidar_driver              at_nav2            competition_fsm
 (LSN10 2D LiDAR)    (Cartographer + Nav2)    (state machine)
         │                       │                        │
         ▼                       ▼                        ▼
  lslidar_msgs            mission_manager          (camera/arm
 (custom msg/srv)    (NavigateToZone action)      external teams)
```

### Package Details

- **`robot_description`** — URDF model. Cylinder `base_link` + fixed `laser_frame` joint. [HW_CONFIG] annotations mark physical dimensions for adjustment.
- **`lslidar_driver`** — Fully implemented C++ Leishen LiDAR driver (X10/CH/CX/LS series). Publishes `sensor_msgs/LaserScan` to `/scan`. **Known issue:** frame_id is `"laser"`, URDF uses `"laser_frame"` — must align before Cartographer/Nav2 can consume scan data (see `docs/TODO-before-deployment.md`).
- **`lslidar_msgs`** — Custom ROS2 messages and services for lslidar_driver.
- **`at_nav2`** — Nav2 bringup config + Cartographer pure localization. Contains:
  - `config/at_nav2_params.yaml` — Nav2 parameters (planner, controller, costmaps, smoother, velocity_smoother)
  - `config/bt_navigator.xml` — Custom behavior tree (ComputePathToPose → FollowPath)
  - `config/cartographer_localization.lua` — Cartographer pure localization config
  - `launch/at_nav.launch.py` — Wraps `nav2_bringup/bringup_launch.py` with Cartographer node
  - `maps/map.yaml` + `map.pgm` — Competition arena map with zone definitions
- **`mission_manager`** (Python) — `NavigateToZone` action server. Loads waypoints from `map.yaml` zones, wraps `NavigateToPose` action client. Entry point: `mission_manager` console script.
- **`competition_fsm`** (Python) — Competition state machine. Manages MANUAL↔AUTONOMOUS switching, orchestrates task sequence, arbitrates `/cmd_vel` between teleop and Nav2 (publishes to `/motor_cmd_vel`), hosts `/fsm_event` service for external team communication. Built with `ament_cmake` + `ament_cmake_python` for rosidl service generation.
- **`robot_startup`** — Top-level bringup launch. Composes all nodes: LiDAR driver, Cartographer, Nav2, mission_manager, competition_fsm.

### TF Tree

```
map ──► odom ──► base_link ──► laser_frame
(Carto)  (chassis)  (URDF)      (URDF fixed joint)
```

### Key Parameters (see `at_nav2_params.yaml` for all [HW_CONFIG] values)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `robot_radius` | 0.35 m | Both costmaps |
| `controller` | RegulatedPurePursuit | Not DWB |
| `desired_linear_vel` | 0.3 m/s | |
| `max_velocity` | [0.5, 0.5, 1.0] | Holonomic: vy enabled |
| `xy_goal_tolerance` | 0.25 m | |
| `yaw_goal_tolerance` | 0.25 rad | |
| `inflation_radius` | 0.5–0.6 m | |
| `observation_source` | `/scan` (LaserScan) | 2D LiDAR |
| `odom_topic` | `/odom` | |

## Design Docs

- `docs/superpowers/specs/2026-05-27-competition-nav-architecture-design.md` — Architecture decision record
- `docs/superpowers/plans/2026-05-27-competition-nav-implementation-plan.md` — Implementation plan (13 tasks)

## Implementation Status

All 13 tasks complete. All packages build and launch successfully.

- [x] Tasks 1-4: at_nav2 config (costmap, Cartographer, launch)
- [x] Tasks 5-7: mission_manager (NavigateToZone action, waypoint loader, action server)
- [x] Tasks 8-11: competition_fsm (package, FSM core, fsm_node, entry point)
- [x] Task 12: robot_startup (total launch)
- [x] Task 13: full workspace build (7/7 packages)

## Known Issues (Pre-Deployment)

See `docs/TODO-before-deployment.md` for full checklist. Critical items:
- Zone names in `ZONE_TO_STATE` don't match map.yaml (待派送区/园区1/园区2 are no_go_zone, not task_area)
- LiDAR frame_id `"laser"` vs URDF `"laser_frame"` mismatch
- Missing `.pbstream` map for Cartographer
- Missing `/odom` + `odom→base_link` TF (odom_driver by other team)
