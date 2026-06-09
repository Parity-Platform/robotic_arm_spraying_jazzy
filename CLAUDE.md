# RAPSEB Robotic Arm Spraying - Jazzy

## Project

Autonomous epoxy spraying workcell for composite panel coating, developed by Parity Platform under the ARISE open call (Horizon Europe). The system uses a UR10e robot arm with a flat-fan spray end-effector, depth cameras, and LiDAR for surface scanning and path planning.

## Stack

- ROS 2 Jazzy on Vulcanexus (previously ROS 2 Humble, migration bugs present)
- Gz Harmonic (not Gazebo Classic - this matters for all simulation plugins and URDF sensors)
- MoveIt 2 for motion planning
- gz_ros2_control for joint control in simulation
- ros_gz_bridge for Gz <-> ROS 2 topic bridging

## Workspace Layout

```
ros2_ws/src/
  spraying_pathways/     # Main package: path planning, spraying nodes, URDF, worlds, launch files
  ur_simulation_gazebo/  # UR10e Gz Harmonic simulation config
  rapseb_hri_safety/     # Optional HRI safety guard (ros4hri, ISO 10218 zones)
Dockerfile/              # Vulcanexus Jazzy container setup
```

## Key Packages

### spraying_pathways
Main package. C++ nodes for cartesian path planning and spraying (flat_fan_spraying_v4.cpp is the latest). Python scripts for point cloud processing, obstacle detection, and trajectory logging. Launch files: bringup_v5.launch.py is the latest, includes ros_gz_bridge.

Important paths:
- `urdf/` - Xacro files for depth cameras and LiDAR (Gz Harmonic sensor types: depth_camera, gpu_lidar)
- `worlds/first_test_case.world` - Gz Harmonic world with UR10e, spray panels, and human_arm obstacle
- `config/gz_bridge.yaml` - ros_gz_bridge topic mappings
- `models/human_arm/model.sdf` - Moving obstacle using gz::sim::systems::VelocityControl

### rapseb_hri_safety
Optional add-on. Hard dependency on hri_msgs (ros4hri). Monitors /humans/bodies/tracked and enforces ISO 10218 safety zones:
- Zone 1 (< 1.0m): protective stop
- Zone 2 (1.0-1.5m): reduced speed via UR dashboard
- Zone 3 (> 1.5m): normal operation

Does not break the main spraying pipeline if not running.

### ur_simulation_gazebo
Gz Harmonic simulation configuration for the UR10e. Controller configs, launch files for spawning the robot in Gz.

## Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Common Pitfalls

- Never use Gazebo Classic plugins (libgazebo_ros_*.so) - this is Gz Harmonic. Use gz:: system plugins.
- URDF sensors use Gz-native types (depth_camera, gpu_lidar), not Classic types (depth, ray). No <plugin> blocks inside <sensor>.
- The ros_gz_bridge handles all Gz <-> ROS 2 topic bridging via config/gz_bridge.yaml.
- Controller manager uses scaled_joint_trajectory_controller for UR10e.
- The /rapseb/spray_status topic (std_msgs/String, JSON payload) is the integration point for FIWARE. Not yet published by spraying nodes - see rapseb-fiware repo for the bridge consumer.

## Style
- resort to the official github repos for all packages. Always use the jazzy branch.
- Minimal comments. No "AI-generated" style explanatory comments.
- Direct, technical language. No filler adjectives.
- Package maintainer: Parity Platform P.C. (info@parityplatform.com)
