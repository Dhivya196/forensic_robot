# Forensic Scene Mapping & Potential Evidence Localization Robot

An autonomous ROS 2 robotics system designed to inspect, map, and document indoor forensic crime scenes using 2D LiDAR SLAM, differential drive navigation (Nav2), TF-linked spatial evidence logging, and computer vision.

---

## Hardware Architecture

| Component | Specification | ROS 2 Interface / Topic |
|---|---|---|
| **Chassis** | 4-Wheel Differential Drive | Coordinate base `base_footprint` / `base_link` |
| **Motors** | 4x Geared DC Motors | `/cmd_vel` $\rightarrow$ L298N Driver |
| **Motor Driver** | L298N Dual H-Bridge | IN1 (GPIO24), IN2 (GPIO23), ENA (GPIO12), IN3 (GPIO22), IN4 (GPIO27), ENB (GPIO13) |
| **Main Computer** | Raspberry Pi 4 (ROS 2 Jazzy) | Core ROS 2 computation graph |
| **2D LiDAR** | 360° Laser Scanner | `/scan` (`sensor_msgs/msg/LaserScan`) |
| **Camera** | Raspberry Pi Camera / USB Cam | `/camera/image_raw` (`sensor_msgs/msg/Image`) |
| **Coordinate Frames** | `map` $\rightarrow$ `odom` $\rightarrow$ `base_footprint` $\rightarrow$ `base_link` $\rightarrow$ `laser_frame`, `camera_optical_link` |

---

## Package Directory Structure

```
src/forensic_robot/
├── config/
│   ├── mapper_params_online_async.yaml   # SLAM Toolbox config
│   ├── nav2_params.yaml                  # Nav2 planner, controller, and costmap parameters
│   └── rviz_config.rviz                  # RViz visualization configuration
├── forensic_robot/
│   ├── active_repositioner.py            # Active multi-view inspection planner
│   ├── evidence_capture.py               # Standalone interactive capture utility
│   ├── evidence_logger.py                # TF-linked spatial evidence logger + RViz markers
│   ├── l298n_driver.py                   # Physical L298N GPIO motor driver
│   ├── motor_controller.py               # Kinematics controller / emulator
│   └── vision_detector.py                # YOLO & heuristic forensic candidate detector
├── launch/
│   ├── evidence_system.launch.py         # Launch evidence logger + vision detector + active repositioner
│   ├── nav2.launch.py                    # Launch Nav2 stack
│   ├── physical_robot.launch.py          # Physical hardware bringup (Motors + LiDAR + Camera + TF)
│   ├── rsp.launch.py                     # Robot State Publisher (TF tree broadcast)
│   ├── sim.launch.py                     # Gazebo Harmonic crime scene simulation & RViz
│   └── slam.launch.py                    # SLAM Toolbox online async mapping
├── urdf/
│   ├── chassis.xacro                     # Dimensions, materials, and wheel links
│   ├── forensic_robot.urdf.xacro         # Master robot description
│   ├── gazebo_control.xacro              # Gazebo DiffDrive & JointState plugins
│   ├── inertial_macros.xacro             # Inertia calculation macros
│   ├── robot_core.xacro                  # 4-wheel differential chassis tree
│   └── sensors.xacro                     # LiDAR, Camera, and IMU sensor frames
├── worlds/
│   └── crime_scene.world                 # Simulated crime scene room with evidence objects
├── package.xml
└── setup.py
```

---

## Quick Start & Operating Modes

### 1. Build the Workspace
```bash
cd ~/forensic_robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

### 2. Gazebo Simulation Workflow (PC / Workstation)

#### Step 1: Launch Gazebo Crime Scene & Robot Model
```bash
ros2 launch forensic_robot sim.launch.py
```
*This spawns the 4-wheel robot in an indoor crime scene room with partitions and simulated evidence items, launches the ROS-GZ bridge, and opens RViz2.*

#### Step 2: Teleoperate the Robot
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Step 3: Start Real-Time 2D SLAM Mapping
```bash
ros2 launch forensic_robot slam.launch.py
```
*Drive through the scene to generate the 2D occupancy grid map.*

#### Step 4: Launch Nav2 Autonomous Navigation
```bash
ros2 launch forensic_robot nav2.launch.py
```
*Click **"Nav2 Goal"** in RViz2 to command autonomous navigation.*

#### Step 5: Start the Spatial Evidence & Vision Detection System
```bash
ros2 launch forensic_robot evidence_system.launch.py
```
*Automatically logs evidence with $(x, y, z)$ map coordinates, generates CSV audit trails, and drops 3D numbered pins on the RViz map.*

---

### 3. Physical Hardware Workflow (Raspberry Pi 4)

#### Step 1: Launch Physical Sensors & Motors
```bash
ros2 launch forensic_robot physical_robot.launch.py lidar_serial_port:=/dev/ttyUSB0
```

#### Step 2: Start SLAM or Nav2
```bash
ros2 launch forensic_robot slam.launch.py use_sim_time:=false
```

#### Step 3: Start Evidence Logger
```bash
ros2 launch forensic_robot evidence_system.launch.py use_sim_time:=false
```

---

## Evidence Output & Audit Trail
All captured observations are recorded under `~/forensic_evidence/`:
* **Images:** `~/forensic_evidence/images/E001_YYYYMMDD_HHMMSS.jpg`
* **Audit CSV Log:** `~/forensic_evidence/evidence_log.csv` with columns:
  `evidence_id, timestamp, image_path, x, y, z, yaw_deg, category, confidence, status`
* **RViz Spatial Markers:** Real-time 3D pins and text flags published on `/evidence_markers`.
