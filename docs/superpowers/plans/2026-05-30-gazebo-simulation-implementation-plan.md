# Gazebo 仿真环境 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 创建 `robot_gazebo` 包，为现有 URDF 机器人模型提供 Gazebo 仿真环境（planar_move + LiDAR + odom + 比赛地图）。

**Architecture:** 新建独立包 `robot_gazebo`，不修改任何现有 package。包含仿真专用 URDF(xacro)、world 文件(SDF)、Python launch 文件。通过 `planar_move_plugin` 接收 `/cmd_vel` 驱动机器人，`ray_plugin` 模拟 LSN10P LiDAR，内置 odometry publisher 发布 `/odom` + TF。

**Tech Stack:** ROS2 Humble, Gazebo Classic, xacro (URDF), SDF 1.6, Python 3 launch files

**Constraints:** 只能创建/修改 `robot_gazebo` 包内的文件。现有包（robot_description, at_nav2 等）只读不写。

---

## File Structure

```
src/robot_gazebo/
├── package.xml                          # 新建
├── CMakeLists.txt                       # 新建
├── urdf/
│   └── robot_sim.xacro                  # 新建：link/joint + gazebo 插件
├── launch/
│   └── gazebo_sim.launch.py             # 新建：启动 Gazebo + spawn
└── worlds/
    └── competition.world                # 新建：比赛场地
```

---

### Task 1: Create package skeleton

**Files:**
- Create: `src/robot_gazebo/package.xml`
- Create: `src/robot_gazebo/CMakeLists.txt`
- Create: `src/robot_gazebo/urdf/` (directory)
- Create: `src/robot_gazebo/launch/` (directory)
- Create: `src/robot_gazebo/worlds/` (directory)

- [ ] **Step 1: Create directory structure**

```bash
mkdir -p src/robot_gazebo/{urdf,launch,worlds}
```

- [ ] **Step 2: Write package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_gazebo</name>
  <version>0.0.1</version>
  <description>Gazebo simulation environment for AGT competition robot</description>
  <maintainer email="funfun@todo.email">funfun</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>gazebo_plugins</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
    <architecture_independent/>
  </export>
</package>
```

- [ ] **Step 3: Write CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
```

- [ ] **Step 4: Build the package to verify skeleton**

```bash
cd /home/funfun/AT_Atlas_nav_ws
colcon build --symlink-install --packages-select robot_gazebo
```

Expected: `Summary: 1 package finished [<time>]` — no errors.

- [ ] **Step 5: Commit**

```bash
git add src/robot_gazebo/
git commit -m "feat: add robot_gazebo package skeleton

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: Write robot_sim.xacro — link/joint definitions

**Files:**
- Create: `src/robot_gazebo/urdf/robot_sim.xacro`
- Reference (read-only): `src/robot_description/urdf/robot_description.urdf`

- [ ] **Step 1: Write the xacro preamble and copy all link/joint definitions**

The xacro file has two major sections:
1. **link/joint definitions** — manually copied from `robot_description.urdf`
2. **Gazebo plugin tags** — added in Task 3

Create `src/robot_gazebo/urdf/robot_sim.xacro` with the preamble and all 20 links + 19 joints from the original URDF. The content below is the COMPLETE file for this step (Task 3 will add gazebo tags):

```xml
<?xml version="1.0" encoding="utf-8"?>
<robot name="robot_description"
  xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ================================================================
       link/joint definitions — copied from robot_description.urdf
       If robot_description.urdf changes, manually sync to this file.
       ================================================================ -->

  <!-- Robot Footprint -->
  <link name="base_footprint"/>

  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0.0 0.0 0.033" rpy="0 0 0"/>
  </joint>

  <!-- base link -->
  <link name="base_link">
    <inertial>
      <origin xyz="-0.00905791507473457 -0.000488795930540497 0.0640826076248514" rpy="0 0 0"/>
      <mass value="3.02079030126272"/>
      <inertia ixx="0.0047459102901528" ixy="3.51756586633145E-06" ixz="9.25187553131626E-07" iyy="0.00495134636228368" iyz="-1.11927483956347E-07" izz="0.00929793533778467"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/base_link.STL"/>
      </geometry>
      <material name="">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/base_link.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- laser link -->
  <link name="laser">
    <inertial>
      <origin xyz="-3.28944182967072E-09 -2.79918769626876E-09 -0.034597418415799" rpy="0 0 0"/>
      <mass value="0.0197577859576288"/>
      <inertia ixx="4.82189534964696E-06" ixy="2.48436928325093E-14" ixz="2.38955057703625E-14" iyy="4.82189520756809E-06" iyz="2.28917153671191E-15" izz="9.36232732215207E-06"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/laser_frame.STL"/>
      </geometry>
      <material name="">
        <color rgba="0.376470588235294 0.376470588235294 0.376470588235294 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/laser_frame.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- laser joint -->
  <joint name="laser_joint" type="fixed">
    <origin xyz="-0.000482541160108889 0 0.111874126705049" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="laser"/>
    <axis xyz="0 0 0"/>
  </joint>

  <!-- arm0_link -->
  <link name="arm0_Link">
    <inertial>
      <origin xyz="5.27093519414809E-05 2.44954880894475E-05 0.0113815325455051" rpy="0 0 0"/>
      <mass value="0.469376261813121"/>
      <inertia ixx="0.000439903649212175" ixy="6.19012321653416E-08" ixz="-2.10717922407623E-06" iyy="0.000442080302381582" iyz="-3.85248785977126E-11" izz="0.000856561950617975"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/arm0_Link.STL"/>
      </geometry>
      <material name="">
        <color rgba="0.647058823529412 0.619607843137255 0.588235294117647 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/arm0_Link.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- arm0 joint -->
  <joint name="arm0" type="continuous">
    <origin xyz="0.0295174588398881 0 0.173124126705049" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="arm0_Link"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- arm1_link -->
  <link name="arm1_Link">
    <inertial>
      <origin xyz="-0.105007518584986 0.00141630130598419 0.0187999942153139" rpy="0 0 0"/>
      <mass value="0.1905874165244"/>
      <inertia ixx="4.18380122418888E-05" ixy="-8.77952273477526E-06" ixz="-1.03900484165303E-08" iyy="0.000406432980248108" iyz="1.47794609107449E-11" izz="0.000424002018340545"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/arm1_Link.STL"/>
      </geometry>
      <material name="">
        <color rgba="0.792156862745098 0.819607843137255 0.933333333333333 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/arm1_Link.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- arm1 joint -->
  <joint name="arm1" type="revolute">
    <origin xyz="0.02 0.0183 0.0595" rpy="1.5708 0 0"/>
    <parent link="arm0_Link"/>
    <child link="arm1_Link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-210" upper="0" effort="1" velocity="3"/>
  </joint>

  <!-- arm2_link -->
  <link name="arm2_Link">
    <inertial>
      <origin xyz="0.100452932197325 0.0447481861533194 -0.0181893769189027" rpy="0 0 0"/>
      <mass value="0.164248712945774"/>
      <inertia ixx="3.52887905356114E-05" ixy="-2.00181663560874E-05" ixz="5.21115865733209E-09" iyy="0.00029554091042407" iyz="-3.44733343118317E-10" izz="0.000308884693204529"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/arm2_Link.STL"/>
      </geometry>
      <material name="">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/arm2_Link.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- arm2 joint -->
  <joint name="arm2" type="revolute">
    <origin xyz="-0.214299994813131 0 0.0376000028997485" rpy="0 0 0"/>
    <parent link="arm1_Link"/>
    <child link="arm2_Link"/>
    <axis xyz="0 0 -1"/>
    <limit lower="0" upper="270" effort="1" velocity="3"/>
  </joint>

  <!-- arm3_link -->
  <link name="arm3_Link">
    <inertial>
      <origin xyz="0.0244536421910909 -0.0093611713162593 -0.0184330992425119" rpy="0 0 0"/>
      <mass value="0.0312238812156817"/>
      <inertia ixx="1.2997419944031E-05" ixy="-6.86664871709515E-07" ixz="-9.49084586365268E-09" iyy="1.14173396064683E-05" iyz="4.27514678311462E-09" izz="1.1219689644564E-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/arm3_Link.STL"/>
      </geometry>
      <material name="">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/arm3_Link.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- arm3 joint -->
  <joint name="arm3" type="revolute">
    <origin xyz="0.195198405547995 0.0479983915101106 0.000499985962073995" rpy="0.00872697851439958 -0.00872598171315928 -7.61524215215586E-05"/>
    <parent link="arm2_Link"/>
    <child link="arm3_Link"/>
    <axis xyz="-0.00872587097686232 -0.00872653549837341 -0.99992385047757"/>
    <limit lower="0" upper="100" effort="1" velocity="3"/>
  </joint>

  <!-- arm4_link -->
  <link name="arm4_Link">
    <inertial>
      <origin xyz="0.0347390678915086 -0.00413801569158295 -0.0209305905705711" rpy="0 0 0"/>
      <mass value="0.0522475609584348"/>
      <inertia ixx="1.36823938394385E-05" ixy="2.66574817985518E-06" ixz="8.39394854007867E-07" iyy="9.79309028256577E-06" iyz="-1.69593151918628E-06" izz="1.75606392336261E-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/arm4_Link.STL"/>
      </geometry>
      <material name="">
        <color rgba="0.650980392156863 0.619607843137255 0.588235294117647 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/arm4_Link.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- arm4 joint -->
  <joint name="arm4" type="revolute">
    <origin xyz="0.0398926340613602 -0.0346685199789408 -0.0183469477877459" rpy="1.56824861064815 0.00509594419856987 -0.366478515653153"/>
    <parent link="arm3_Link"/>
    <child link="arm4_Link"/>
    <axis xyz="-0.00872587097688493 -0.00872653523448956 -0.999923850479873"/>
    <limit lower="-90" upper="90" effort="1" velocity="3"/>
  </joint>

  <!-- LFs link (左前 steering) -->
  <link name="LFs_Link">
    <inertial>
      <origin xyz="-3.43181859924213E-06 0.00109461595355642 -0.0424747271155292" rpy="0 0 0"/>
      <mass value="0.114715980836134"/>
      <inertia ixx="1.43428504844199E-05" ixy="1.07530266321773E-11" ixz="3.39986697862229E-12" iyy="2.96038470108407E-05" iyz="2.80051423962253E-10" izz="2.00828947865518E-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/LFs_Link.STL"/>
      </geometry>
      <material name="">
        <color rgba="1 0.949019607843137 0.898039215686275 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/LFs_Link.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- LFs joint -->
  <joint name="LFs" type="continuous">
    <origin xyz="0.13238 0.13286 0.055124" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="LFs_Link"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- LFd link (左前 drive) -->
  <link name="LFd_Link">
    <inertial>
      <origin xyz="-1.03712429144753E-06 -4.22896943690254E-05 -0.0180040579132936" rpy="0 0 0"/>
      <mass value="0.0555886189628006"/>
      <inertia ixx="2.17656627943887E-05" ixy="1.87563768432862E-16" ixz="8.87301353584558E-17" iyy="2.17656627947915E-05" iyz="1.05868514442875E-16" izz="3.98658756411664E-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/LFd_Link.STL"/>
      </geometry>
      <material name="">
        <color rgba="0.298039215686275 0.298039215686275 0.298039215686275 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/LFd_Link.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- LFd joint -->
  <joint name="LFd" type="continuous">
    <origin xyz="0 -0.0192000070641395 -0.0549577103053597" rpy="1.5707963267949 0 0"/>
    <parent link="LFs_Link"/>
    <child link="LFd_Link"/>
    <axis xyz="0 0 -1"/>
  </joint>

  <!-- LRs link (左后 steering) -->
  <link name="LRs_Link">
    <inertial>
      <origin xyz="-3.45773536702954E-06 0.00109460811387849 -0.0424747271155197" rpy="0 0 0"/>
      <mass value="0.114715980835521"/>
      <inertia ixx="1.43428504843719E-05" ixy="1.07530264428732E-11" ixz="3.39986454052669E-12" iyy="2.96038470107929E-05" iyz="2.80051427479846E-10" izz="2.00828947865514E-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/LRs_Link.STL"/>
      </geometry>
      <material name="">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/LRs_Link.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- LRs joint -->
  <joint name="LRs" type="continuous">
    <origin xyz="-0.13335 0.13286 0.055124" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="LRs_Link"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- LRd link (左后 drive) -->
  <link name="LRd_Link">
    <inertial>
      <origin xyz="-1.05702918615225E-06 -4.22896946395625E-05 -0.0180040579107741" rpy="0 0 0"/>
      <mass value="0.0555886189628006"/>
      <inertia ixx="2.17656627948538E-05" ixy="-7.90717223889219E-17" ixz="9.89661937500372E-17" iyy="2.17656627943267E-05" iyz="-8.71922434634148E-18" izz="3.98658756411665E-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/LRd_Link.STL"/>
      </geometry>
      <material name="">
        <color rgba="0.796078431372549 0.823529411764706 0.937254901960784 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/LRd_Link.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- LRd joint -->
  <joint name="LRd" type="continuous">
    <origin xyz="0 -0.0192000149011577 -0.0549577103053588" rpy="1.5707963267949 0 0"/>
    <parent link="LRs_Link"/>
    <child link="LRd_Link"/>
    <axis xyz="0 0 -1"/>
  </joint>

  <!-- RFs link (右前 steering) -->
  <link name="RFs_Link">
    <inertial>
      <origin xyz="3.44283421241376E-06 -0.00109462301503799 -0.0424747271155221" rpy="0 0 0"/>
      <mass value="0.114715980835538"/>
      <inertia ixx="1.43428504843731E-05" ixy="1.07530264536241E-11" ixz="-3.39986462854254E-12" iyy="2.96038470107942E-05" iyz="-2.80051427386621E-10" izz="2.00828947865514E-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/RFs_Link.STL"/>
      </geometry>
      <material name="">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/RFs_Link.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- RFs joint -->
  <joint name="RFs" type="continuous">
    <origin xyz="0.13238 -0.13286 0.055124" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="RFs_Link"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- RFd link (右前 drive) -->
  <link name="RFd_Link">
    <inertial>
      <origin xyz="1.05702960626064E-06 -4.22896943098171E-05 0.0180040579107738" rpy="0 0 0"/>
      <mass value="0.0555886189628007"/>
      <inertia ixx="2.17656627945638E-05" ixy="2.74003517078324E-16" ixz="-7.01209695758619E-17" iyy="2.17656627946164E-05" iyz="-1.23646991662953E-16" izz="3.98658756411663E-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/RFd_Link.STL"/>
      </geometry>
      <material name="">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/RFd_Link.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- RFd joint -->
  <joint name="RFd" type="continuous">
    <origin xyz="0 0.0192000000000008 -0.0549577103053633" rpy="1.5707963267949 0 0"/>
    <parent link="RFs_Link"/>
    <child link="RFd_Link"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- RRs_Link (右后 steering) -->
  <link name="RRs_Link">
    <inertial>
      <origin xyz="3.56441174925726E-06 -0.00113327767047011 -0.0438441812611075" rpy="0 0 0"/>
      <mass value="0.110803165091198"/>
      <inertia ixx="1.37587466407648E-05" ixy="1.07530264549324E-11" ixz="-3.39986463165107E-12" iyy="2.89883861963043E-05" iyz="-2.80051427336613E-10" izz="1.89090821817843E-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/RRs_Link.STL"/>
      </geometry>
      <material name="">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/RRs_Link.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- RRs joint -->
  <joint name="RRs" type="continuous">
    <origin xyz="-0.133347475742118 -0.132864934582008 0.0551241267050491" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="RRs_Link"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- RRd_Link (右后 drive) -->
  <link name="RRd_Link">
    <inertial>
      <origin xyz="1.05702937763796E-06 -4.22896944295686E-05 0.0180040579107738" rpy="0 0 0"/>
      <mass value="0.0555886189628006"/>
      <inertia ixx="2.17656627943621E-05" ixy="-1.53737084532589E-16" ixz="2.70310359630393E-17" iyy="2.17656627948185E-05" iyz="-8.95623799269351E-17" izz="3.98658756411666E-05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/RRd_Link.STL"/>
      </geometry>
      <material name="">
        <color rgba="0.298039215686275 0.298039215686275 0.298039215686275 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://robot_description/meshes/RRd_Link.STL"/>
      </geometry>
    </collision>
  </link>

  <!-- RRd joint -->
  <joint name="RRd" type="continuous">
    <origin xyz="0 0.0192000000000007 -0.0549577103053587" rpy="1.5707963267949 0 0"/>
    <parent link="RRs_Link"/>
    <child link="RRd_Link"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- ================================================================
       Gazebo plugin tags — see Task 3
       ================================================================ -->
  <!-- PLACEHOLDER: Task 3 will insert gazebo plugins here -->

</robot>
```

> **Note:** The link/joint content is identical to `robot_description.urdf` lines 1–873. If the original URDF changes, these must be synced manually.

- [ ] **Step 2: Commit**

```bash
git add src/robot_gazebo/urdf/robot_sim.xacro
git commit -m "feat: add robot_sim.xacro with link/joint definitions

Copied from robot_description.urdf for standalone simulation use.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 3: Add Gazebo plugin tags to robot_sim.xacro

**Files:**
- Modify: `src/robot_gazebo/urdf/robot_sim.xacro`

- [ ] **Step 1: Replace PLACEHOLDER with collision simplification + planar_move + ray + joint_state_publisher**

Replace the `<!-- PLACEHOLDER ... -->` comment before `</robot>` with:

```xml
  <!-- ================================================================
       Gazebo Simulation Plugins
       ================================================================ -->

  <!-- ===== Collision Overrides (simplified geometry) ===== -->

  <gazebo reference="base_link">
    <collision>
      <geometry>
        <box>
          <size>0.6 0.5 0.25</size>
        </box>
      </geometry>
    </collision>
    <maxContacts>10</maxContacts>
  </gazebo>

  <gazebo reference="laser">
    <collision>
      <geometry>
        <box>
          <size>0.07 0.07 0.07</size>
        </box>
      </geometry>
    </collision>
  </gazebo>

  <!-- Steering links: LFs, LRs, RFs, RRs -->
  <gazebo reference="LFs_Link">
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.04"/>
      </geometry>
    </collision>
  </gazebo>

  <gazebo reference="LRs_Link">
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.04"/>
      </geometry>
    </collision>
  </gazebo>

  <gazebo reference="RFs_Link">
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.04"/>
      </geometry>
    </collision>
  </gazebo>

  <gazebo reference="RRs_Link">
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.04"/>
      </geometry>
    </collision>
  </gazebo>

  <!-- Drive links: LFd, LRd, RFd, RRd -->
  <gazebo reference="LFd_Link">
    <collision>
      <geometry>
        <cylinder radius="0.033" length="0.02"/>
      </geometry>
    </collision>
    <material>
      <uri>model://grey</uri>
    </material>
  </gazebo>

  <gazebo reference="LRd_Link">
    <collision>
      <geometry>
        <cylinder radius="0.033" length="0.02"/>
      </geometry>
    </collision>
  </gazebo>

  <gazebo reference="RFd_Link">
    <collision>
      <geometry>
        <cylinder radius="0.033" length="0.02"/>
      </geometry>
    </collision>
  </gazebo>

  <gazebo reference="RRd_Link">
    <collision>
      <geometry>
        <cylinder radius="0.033" length="0.02"/>
      </geometry>
    </collision>
  </gazebo>

  <!-- Arm links: disable collision (not simulated) -->
  <gazebo reference="arm0_Link">
    <collision>
      <geometry>
        <box>
          <size>0.01 0.01 0.01</size>
        </box>
      </geometry>
    </collision>
  </gazebo>

  <gazebo reference="arm1_Link">
    <collision>
      <geometry>
        <box>
          <size>0.01 0.01 0.01</size>
        </box>
      </geometry>
    </collision>
  </gazebo>

  <gazebo reference="arm2_Link">
    <collision>
      <geometry>
        <box>
          <size>0.01 0.01 0.01</size>
        </box>
      </geometry>
    </collision>
  </gazebo>

  <gazebo reference="arm3_Link">
    <collision>
      <geometry>
        <box>
          <size>0.01 0.01 0.01</size>
        </box>
      </geometry>
    </collision>
  </gazebo>

  <gazebo reference="arm4_Link">
    <collision>
      <geometry>
        <box>
          <size>0.01 0.01 0.01</size>
        </box>
      </geometry>
    </collision>
  </gazebo>

  <!-- ===== Planar Move Plugin (cmd_vel → base motion + odom) ===== -->
  <gazebo>
    <plugin name="planar_move" filename="libgazebo_ros_planar_move.so">
      <ros>
        <namespace>/</namespace>
      </ros>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <publish_odom_tf>true</publish_odom_tf>
      <odometry_publish_rate>30.0</odometry_publish_rate>
    </plugin>
  </gazebo>

  <!-- ===== Joint State Publisher (→ /joint_states) ===== -->
  <gazebo>
    <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
      <ros>
        <namespace>/</namespace>
        <remapping>~/out:=joint_states</remapping>
      </ros>
      <update_rate>30</update_rate>
    </plugin>
  </gazebo>

  <!-- ===== 2D LiDAR Ray Sensor (→ /scan) ===== -->
  <gazebo reference="laser">
    <sensor type="ray" name="laser_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.15</min>
          <max>30.0</max>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <frame_name>laser</frame_name>
      </plugin>
    </sensor>
  </gazebo>
```

- [ ] **Step 2: Verify xacro processing works**

```bash
xacro src/robot_gazebo/urdf/robot_sim.xacro > /tmp/robot_sim_processed.urdf 2>&1
echo "Exit code: $?"
head -5 /tmp/robot_sim_processed.urdf
```

Expected: exit code 0, output starts with `<robot name="robot_description">`

- [ ] **Step 3: Verify URDF parses correctly**

```bash
check_urdf /tmp/robot_sim_processed.urdf 2>&1
```

Expected: `robot name is: robot_description` and no errors.

- [ ] **Step 4: Commit**

```bash
git add src/robot_gazebo/urdf/robot_sim.xacro
git commit -m "feat: add Gazebo plugins to robot_sim.xacro

- Simplified collision geometry (box/cylinder)
- Planar move plugin for cmd_vel control
- Joint state publisher for /joint_states
- Ray sensor for 2D LiDAR /scan

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 4: Create competition.world

**Files:**
- Create: `src/robot_gazebo/worlds/competition.world`
- Reference (read-only): `src/at_nav2/maps/map.yaml`

- [ ] **Step 1: Write the world file with ground plane, walls, and physics**

Create `src/robot_gazebo/worlds/competition.world`:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="competition">

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <scene>
      <shadows>false</shadows>
    </scene>

    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>2.0 2.0 5.0 0 0.8 2.3</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    <!-- Physics: ODE solver tuned for mobile robot simulation -->
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- ================================================================
         Walls — manually placed based on no_go_zone polygons in map.yaml
         Map: 4m × 4m (200px × 0.02m resolution)
         Wall height: 1.0m
         ================================================================ -->

    <!-- 垛码区货架B: [1.04,0.16]→[1.8,0.48] -->
    <model name="wall_垛码区货架B">
      <static>true</static>
      <pose>1.42 0.32 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.76 0.32 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.76 0.32 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- 分拣区货架B: [0.18,0.84]→[0.5,1.78] -->
    <model name="wall_分拣区货架B">
      <static>true</static>
      <pose>0.34 1.31 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.32 0.94 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.32 0.94 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- 货物待装区货架B: [0.82,1.66]→[1.24,1.96] -->
    <model name="wall_货物待装区货架B">
      <static>true</static>
      <pose>1.03 1.81 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.42 0.30 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.42 0.30 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- 货物装箱区B: [1.82,1.44]→[2.32,1.94] -->
    <model name="wall_货物装箱区B">
      <static>true</static>
      <pose>2.07 1.69 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.50 0.50 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.50 0.50 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- 零件部件仓储区: [1.5,1.92]→[1.66,2.02] -->
    <model name="wall_零件部件仓储区">
      <static>true</static>
      <pose>1.58 1.97 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.16 0.10 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.16 0.10 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- 待派送区B: [3.44,0.72]→[3.88,1.12] -->
    <model name="wall_待派送区B">
      <static>true</static>
      <pose>3.66 0.92 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.44 0.40 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.44 0.40 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- 园区2B: [2.7,1.54]→[3.22,1.8] -->
    <model name="wall_园区2B">
      <static>true</static>
      <pose>2.96 1.67 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.52 0.26 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.52 0.26 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- 园区1B: [2.54,0.14]→[3.04,0.42] -->
    <model name="wall_园区1B">
      <static>true</static>
      <pose>2.79 0.28 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.50 0.28 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.50 0.28 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- 垛码区货架A: [1.08,3.48]→[1.78,3.8] -->
    <model name="wall_垛码区货架A">
      <static>true</static>
      <pose>1.43 3.64 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.70 0.32 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.70 0.32 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- 分拣区货架A: [0.16,2.2]→[0.5,3.1] -->
    <model name="wall_分拣区货架A">
      <static>true</static>
      <pose>0.33 2.65 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.34 0.90 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.34 0.90 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- 园区1A: [2.52,3.54]→[3.04,3.82] -->
    <model name="wall_园区1A">
      <static>true</static>
      <pose>2.78 3.68 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.52 0.28 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.52 0.28 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- 园区2A: [2.7,2.16]→[3.2,2.42] -->
    <model name="wall_园区2A">
      <static>true</static>
      <pose>2.95 2.29 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.50 0.26 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.50 0.26 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- 货物待装区货架A: [0.82,2.0]→[1.24,2.3] -->
    <model name="wall_货物待装区货架A">
      <static>true</static>
      <pose>1.03 2.15 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.42 0.30 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.42 0.30 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- 货物装箱区A: [1.82,2.02]→[2.34,2.52] -->
    <model name="wall_货物装箱区A">
      <static>true</static>
      <pose>2.08 2.27 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.52 0.50 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.52 0.50 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- 待派送区A: [3.5,2.7]→[3.9,3.08] -->
    <model name="wall_待派送区A">
      <static>true</static>
      <pose>3.70 2.89 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.40 0.38 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.40 0.38 1.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Arena boundary walls (4m × 4m map) -->
    <model name="wall_boundary_north">
      <static>true</static>
      <pose>2.0 4.05 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>4.0 0.1 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>4.0 0.1 1.0</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="wall_boundary_south">
      <static>true</static>
      <pose>2.0 -0.05 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>4.0 0.1 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>4.0 0.1 1.0</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="wall_boundary_east">
      <static>true</static>
      <pose>4.05 2.0 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.1 4.0 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.1 4.0 1.0</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="wall_boundary_west">
      <static>true</static>
      <pose>-0.05 2.0 0.5 0 0 0</pose>
      <link name="body">
        <collision>
          <geometry><box><size>0.1 4.0 1.0</size></box></geometry>
        </collision>
        <visual>
          <geometry><box><size>0.1 4.0 1.0</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

- [ ] **Step 2: Verify world file is valid**

```bash
gazebo --verbose -p src/robot_gazebo/worlds/competition.world &
GAZEBO_PID=$!
sleep 5
kill $GAZEBO_PID 2>/dev/null
```

Expected: Gazebo starts and renders within 5 seconds. No SDF parse errors in console.

- [ ] **Step 3: Commit**

```bash
git add src/robot_gazebo/worlds/competition.world
git commit -m "feat: add competition.world with 15 no-go-zone walls

Walls manually placed based on map.yaml no_go_zone polygons.
Includes arena boundary and ODE physics configuration.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 5: Create launch file

**Files:**
- Create: `src/robot_gazebo/launch/gazebo_sim.launch.py`
- Reference (read-only): `/opt/ros/humble/share/turtlebot3_gazebo/launch/turtlebot3_world.launch.py`

- [ ] **Step 1: Write the launch file**

Create `src/robot_gazebo/launch/gazebo_sim.launch.py`:

```python
#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Paths
    world_path = LaunchConfiguration(
        'world_path',
        default=os.path.join(pkg_share, 'worlds', 'competition.world')
    )
    xacro_path = os.path.join(pkg_share, 'urdf', 'robot_sim.xacro')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pos = LaunchConfiguration('x_pos', default='1.0')
    y_pos = LaunchConfiguration('y_pos', default='0.3')
    z_pos = LaunchConfiguration('z_pos', default='0.1')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    declare_x_pos = DeclareLaunchArgument(
        'x_pos', default_value='1.0',
        description='Robot spawn X position (m)'
    )
    declare_y_pos = DeclareLaunchArgument(
        'y_pos', default_value='0.3',
        description='Robot spawn Y position (m)'
    )
    declare_z_pos = DeclareLaunchArgument(
        'z_pos', default_value='0.1',
        description='Robot spawn Z position (m)'
    )

    # 1. Gazebo server (loads the world)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 2. Gazebo client (GUI)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 3. robot_state_publisher (TF from /joint_states)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_path]),
                value_type=str
            ),
        }],
    )

    # 4. Spawn robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot_description',
            '-topic', 'robot_description',
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos,
        ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x_pos)
    ld.add_action(declare_y_pos)
    ld.add_action(declare_z_pos)

    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    return ld
```

- [ ] **Step 2: Verify Python syntax**

```bash
python3 -c "import py_compile; py_compile.compile('src/robot_gazebo/launch/gazebo_sim.launch.py', doraise=True)"
echo "Syntax OK"
```

Expected: `Syntax OK`

- [ ] **Step 3: Commit**

```bash
git add src/robot_gazebo/launch/gazebo_sim.launch.py
git commit -m "feat: add gazebo_sim.launch.py

Launches Gazebo with competition.world, spawns robot via
robot_state_publisher + spawn_entity, with configurable spawn pose.

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 6: Build and smoke test

**Files:**
- No new files created

- [ ] **Step 1: Build the package**

```bash
cd /home/funfun/AT_Atlas_nav_ws
colcon build --symlink-install --packages-select robot_gazebo
source install/setup.bash
```

Expected: `Summary: 1 package finished` — no errors.

- [ ] **Step 2: Verify xacro resolution in launch context**

```bash
ros2 launch robot_gazebo gazebo_sim.launch.py --show-args
```

Expected: Shows `x_pos`, `y_pos`, `z_pos`, `use_sim_time` arguments.

- [ ] **Step 3: Launch Gazebo and verify basic startup**

```bash
timeout 30 ros2 launch robot_gazebo gazebo_sim.launch.py 2>&1 | head -50
```

Expected: Gazebo starts, robot spawns within 30 seconds. Look for:
- `[spawn_entity-*] [INFO] ... Spawn status: SpawnEntity: Successfully spawned entity`
- No SDF/URDF parse errors

- [ ] **Step 4: Verify topics (run in a separate terminal while Gazebo is running)**

```bash
# In a separate terminal, source the workspace and check:
source /home/funfun/AT_Atlas_nav_ws/install/setup.bash
sleep 5
ros2 topic list | grep -E "/scan|/odom|/cmd_vel|/joint_states"
```

Expected: Should list `/scan`, `/odom`, `/cmd_vel`, `/joint_states`

```bash
ros2 topic echo /scan --once 2>&1 | head -10
```

Expected: LaserScan message with valid ranges.

```bash
ros2 topic echo /odom --once 2>&1 | head -15
```

Expected: Odometry message with pose and twist.

- [ ] **Step 5: Verify cmd_vel moves the robot**

```bash
# Publish a forward velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

Expected: Robot moves forward in Gazebo. Check with:

```bash
ros2 topic echo /odom --once 2>&1 | grep -A5 "pose:"
```

Expected: pose.position.x should be non-zero (robot has moved).

- [ ] **Step 6: Commit if any fixes were made, or confirm all tests pass**

```bash
git status
```

---

## Summary

| Task | Files | Description |
|------|-------|-------------|
| 1 | `package.xml`, `CMakeLists.txt` | Package skeleton |
| 2 | `urdf/robot_sim.xacro` | Link/joint definitions (from robot_description) |
| 3 | `urdf/robot_sim.xacro` (edit) | Gazebo plugins: collision, planar_move, ray, joint_state |
| 4 | `worlds/competition.world` | Competition arena with walls |
| 5 | `launch/gazebo_sim.launch.py` | Launch file: Gazebo + spawn robot |
| 6 | (verification only) | Build, launch, topic check, cmd_vel test |
