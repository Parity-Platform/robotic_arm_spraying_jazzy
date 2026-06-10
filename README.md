# RAPSEB Autonomous Epoxy Spraying Workcell

Autonomous robotic epoxy spraying system for composite panel coating, developed by Parity Platform under the ARISE open call (Horizon Europe). The system uses a UR10e manipulator with a flat-fan spray end-effector, depth cameras for surface inspection, and 2D LiDAR for obstacle detection, all running on ROS 2 Jazzy with Gz Harmonic simulation.

## System Architecture

The workcell operates as a closed-loop spraying pipeline:

1. **Surface scanning** -- Depth cameras and LiDAR acquire the panel geometry and build a 3D point cloud of the workspace.
2. **Path planning** -- MoveIt 2 computes Cartesian spray trajectories over the panel surface, using flat-fan coverage grids with velocity-profiled motion.
3. **Spraying execution** -- The UR10e follows the planned trajectory while the flat-fan nozzle deposits epoxy. Pass coverage is tracked incrementally.
4. **Obstacle monitoring** -- Point cloud filtering separates known geometry (table, panel, robot) from unknown objects. DBSCAN clustering detects obstacle clusters and triggers protective stops. The spraying node includes a pause/resume gate that halts execution while unknown points are present and resumes from the nearest trajectory point once the hazard clears.
5. **Collision avoidance** -- Obstacle detection is complete. Detected clusters trigger a protective stop and resume automatically once the obstacle clears. Integration of detected clusters into the MoveIt planning scene as collision objects (route-around instead of stop) is not yet implemented.
6. **HRI safety (optional)** -- A separate node monitors human proximity via ros4hri and enforces ISO 10218 safety zones.

The pipeline runs inside a Vulcanexus Jazzy Docker container. Gz Harmonic provides physics simulation with depth camera and LiDAR sensor emulation. The ros_gz_bridge forwards sensor data between Gz transport and ROS 2 topics.

## Repository Structure

```
robotic_arm_spraying_jazzy/
  Dockerfile/
    Dockerfile                    # Vulcanexus Jazzy desktop container
    README.md                     # Build, run, and usage instructions
  ros2_ws/
    src/
      gz_ros2_control/            # gz_ros2_control built from source (jazzy branch)
      spraying_pathways/          # Main package: planning, spraying, sensing
        src/                      # C++ nodes (20 executables)
        scripts/                  # Python nodes and utilities
        include/spraying_pathways/# Header-only libraries
        urdf/                     # Robot + sensor URDF/xacro
        worlds/                   # Gz Harmonic world files
        models/                   # Simulation models (tables, obstacles)
        config/                   # Controller and bridge configs
        launch/                   # Launch files (v1-v5)
        materials/                # Visual textures
      ur_simulation_gazebo/       # UR10e Gz Harmonic simulation config
        config/                   # UR controller definitions
        launch/                   # Simulation + MoveIt launch files
        test/                     # Launch integration tests
      rapseb_hri_safety/          # Optional HRI safety guard
        rapseb_hri_safety/        # Python package source
        launch/                   # Safety guard launch file
```

## ROS 2 Packages

### spraying_pathways

Core package containing all planning, spraying, sensing, and simulation assets.

#### C++ Nodes

**Spraying nodes** (latest: flat_fan_spraying_v4):
- `flat_fan_spraying_v4_node` -- Main spraying controller. Generates flat-fan spray grids over the target surface using coverage-optimised waypoints, executes multi-line trajectories with cubic/quintic velocity profiles and Bezier curve direction changes. Tracks per-pass coverage. Publishes `/spray_plan` (grid layout, transient-local QoS) and `/spray_current_idx` (active spray centre, 4 Hz) for use by `epoxy_visualizer.py`. Includes `UnknownGate` obstacle watchdog with pause-resume execution.
- `flat_fan_spraying_v3_node`, `v2`, `v1` -- Earlier iterations retained for reference.

**Path planning nodes** (latest: cartesian_path_planner_cubes_v4):
- `cartesian_path_planner_cubes_v4_node` -- Generates Cartesian paths over a discretised cube grid. Computes MoveIt trajectories with configurable velocity/acceleration scaling.
- `cartesian_path_planner_trajectory_v1_node` -- Waypoint-based Cartesian planning.
- `cartesian_path_planner_cubes_test_*` -- Fixed-position variants for debugging problematic glue deposition areas.

**Utility nodes:**
- `go_home_node` -- Moves the UR10e to a predefined home configuration {0, -2.15, 2.15, -1.57, -1.57, 0} rad via MoveIt.
- `lidar_surface_scanner_node` -- Subscribes to `/lidar/scan/points`, transforms scans to base_link, accumulates point clouds during a trajectory sweep, publishes the result on `scan_cloud`.
- `safety_stop_on_unknown_node` -- Monitors `/unknown_points`. If the cluster exceeds a threshold (debounced 500ms), cancels all FollowJointTrajectory goals and calls MoveGroup::stop().
- `moving_obstacle_node` -- Publishes velocity commands to animate the human_arm simulation model.
- `self_filter_node` -- Removes robot self-collision geometry from the point cloud.
- `inspect_flatness_node` -- Surface flatness inspection logic.

#### Header Libraries

Located in `include/spraying_pathways/`:

- **types.hpp** -- Core data structures: `Point2D`, `Pose3D`, `Cube` with comparison operators for zig-zag ordering.
- **spray_trajectory.hpp** -- Trajectory generation library. Smooth velocity profiles (cubic time-scaling), quintic polynomial blends, Bezier curve direction changes, and multi-line trajectory generation with configurable accel/cruise/decel phases.
- **spraying_grid.hpp** -- Grid generation for flat-fan coverage. Rectangle corner sorting, patch-based grid decomposition, height-mapped spray deposition with power-law falloff (sigma parameter).

#### Python Scripts

- `epoxy_visualizer.py` -- Progressive RViz coating visualiser. Subscribes to `/spray_plan` (grid layout, published once by `flat_fan_spraying_v4_node`) and `/spray_current_idx` (active spray centre index, published every 250 ms during execution). Publishes a `MarkerArray` on `/epoxy_coating_markers` showing coloured cubes that grow in real time as the robot sprays. Layer counts are persisted to `/tmp/epoxy_layers.json` so successive runs accumulate visually (yellow → orange → deep orange → dark red). Text labels `L2`, `L3`, etc. appear above cells with more than one layer. Reset with `rm /tmp/epoxy_layers.json`.
- `pointcloud_transform_and_unknown_filter_v3.py` -- Transforms depth camera point clouds to base_link, parses the Gz world file for known geometry, separates unknown objects. Used in bringup_v5 launch.
- `obstacles_tracking.py` -- DBSCAN clustering on the unknown points cloud. Publishes obstacle centroids as visualisation markers. Can cancel active FollowJointTrajectory goals.
- `unknown_object_detector.py` -- Parses world SDF for known objects and robot URDF for collision geometry. Publishes markers for both.
- `trajectory_logger.py` -- Subscribes `/joint_states`, logs joint positions/velocities, end-effector pose and speed to CSV.
- `ee_velocity_monitor.py` -- Monitors end-effector velocity via TF, logs to CSV.
- `depth_dip_detector_v1.py` -- Detects surface dips/depressions in depth data.
- `plot_trajectory.py`, `plot_trajectory_log.py` -- Matplotlib visualisation of logged trajectories.

#### URDF / Xacro

- `my_robot.urdf.xacro` -- Master robot assembly. Includes the UR10e description, two depth cameras (fixed overhead + wrist-mounted), and optionally a 2D LiDAR.
- `depth_camera.urdf.xacro` -- Fixed overhead depth camera. Gz Harmonic sensor type `depth_camera`, 424x240 at 5 Hz, 60-degree FOV, 0.05-20m range. Rate reduced from 30 Hz to maintain real-time factor on CPU-only rendering.
- `depth_camera_ee.urdf.xacro` -- Wrist-mounted depth camera on `wrist_3_link`. 320x240 at 5 Hz.
- `2d_lidar.urdf.xacro` -- 2D LiDAR sensor. Gz Harmonic sensor type `gpu_lidar`.

#### Gz Harmonic Assets

**Worlds:**
- `table_world.world` -- Active simulation world. UR10e, wood_table, black_table, box_0_5 spray panel, human_arm dynamic obstacle. Requires `gz::sim::systems::Sensors` with `ogre2` render engine for depth camera support.
- `first_test_case.world` -- Earlier world retained for reference.

**Models:**
- `human_arm/` -- Articulated human arm obstacle with STL/OBJ meshes. Controlled via `gz::sim::systems::VelocityControl` (cmd_vel topic bridged via ros_gz_bridge).
- `black_table/`, `wood_table/` -- Workspace tables.
- `box_0_5/` -- 0.5m test box for path planning validation.

#### Configuration

- `controllers.yaml` -- MoveIt controller definitions. `scaled_joint_trajectory_controller` (for real UR hardware) and `joint_trajectory_controller` (for simulation).
- `gz_bridge.yaml` -- ros_gz_bridge topic mappings: depth cameras (image, depth, points, camera_info), LiDAR (points, scan), clock, and human_arm cmd_vel.
- `ur10e/` -- UR10e-specific kinematic, joint limit, and physical parameter files.

#### Launch Files

- **bringup_v5.launch.py** (recommended) -- Full pipeline: robot description, MoveIt config, Gz Harmonic simulation, ros_gz_bridge, RViz, point cloud processing, obstacle tracking, and epoxy visualiser. Node startup sequence: Gz + RViz + MoveIt at t=0s, epoxy visualiser at t=5s, point cloud filter at t=10s, obstacle tracking at t=13s.
- **bringup_v4.launch.py** -- Previous version, uses point cloud transform node.
- **bringup_v3.launch.py** -- Adds ros_gz_bridge.
- **bringup_v2.launch.py**, **bringup.launch.py** -- Earlier iterations.

### ur_simulation_gazebo

Gz Harmonic simulation configuration for Universal Robots manipulators.

- `ur_sim_control.launch.py` -- Spawns UR robot in Gz Harmonic with ros2_control. Configurable via `ur_type` argument (default: ur10e).
- `ur_sim_moveit.launch.py` -- Launches MoveIt planning with RViz for the UR simulation.
- `config/ur_controllers.yaml` -- Controller definitions linking to ur_controllers ROS package.

### rapseb_hri_safety

Optional human-robot interaction safety guard. See the [package README](ros2_ws/src/rapseb_hri_safety/README.md) for full details.

Monitors human proximity via ros4hri body tracking (`/humans/bodies/tracked`) and enforces ISO 10218 safety zones by pausing/resuming the trajectory controller through the controller_manager service interface. Optionally adjusts the UR speed slider on real hardware.

| Zone | Distance | Action |
|------|----------|--------|
| Z1   | < 1.0 m  | Deactivate controller, speed slider 0% |
| Z2   | < 1.5 m  | Reduce speed slider to 20% |
| Z3   | >= 1.5 m | Normal operation |

Hard dependency on `hri_msgs` (ros4hri). Does not affect the spraying pipeline if not launched.

## Deployment

### Prerequisites

- Docker and Docker Compose
- X11 forwarding for GUI (RViz, Gz Harmonic)
- Host with GPU recommended for Gz Harmonic rendering

### Build and Run

```bash
# Build the container
cd Dockerfile
docker image build -t my-vulcanexus:jazzy-desktop .

# Run (from the repo root, where ros2_ws/ is located)
xhost +local:docker # Required for X11 forwarding (not wsl2)
cd ..
docker run -it --rm --name vulcanexus-container --user vulcanexus_user \
  -v $PWD/ros2_ws:/ros2_ws -w /ros2_ws \
  --network=host --ipc=host \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  my-vulcanexus:jazzy-desktop
```

### First-Time Setup

Clone `gz_ros2_control` from source into the workspace before building. The apt package (1.2.17) has a threading bug in `GazeboSimSystem::initSim`; the source build from the jazzy branch fixes it.

```bash
cd ros2_ws/src
git clone https://github.com/ros-controls/gz_ros2_control.git --branch jazzy --depth 1 gz_ros2_control
```

Then build inside the container:

```bash
cd /ros2_ws
rm -rf build/ log/ install/
rosdep update && rosdep install --ignore-src --from-paths . -y
colcon build --symlink-install --parallel-workers 1 --executor sequential # Depending on system memory, you might not need the flags
source install/setup.bash
```

### Running the Simulation

Terminal 1 -- Launch the full pipeline:
```bash
ros2 launch spraying_pathways bringup_v5.launch.py
```

This starts everything automatically, including the epoxy visualiser and obstacle tracking. Wait for the system to fully initialise (~15 seconds) before proceeding.

Terminal 2 -- Send robot to home position:
```bash
ros2 run spraying_pathways go_home_node
```

Terminal 3 -- Execute spraying:
```bash
ros2 run spraying_pathways flat_fan_spraying_v4_node
```

In RViz: click **Add** → **MarkerArray** and add the following topics to see the full pipeline:
- `/epoxy_coating_markers` -- progressive coating visualisation
- `/obstacle_centroids` -- detected obstacles (red spheres)
- `/known_objects_markers` -- known world geometry (green)
- `/robot_objects_markers` -- robot self-filter volumes (blue)

Coloured cubes appear on the panel in RViz as the robot moves over each cell. Run the spray node again without restarting the visualiser to add a second layer (colours shift yellow → orange → red). Reset accumulated layers with:
```bash
rm /tmp/epoxy_layers.json
```

### Testing Obstacle Detection

The `human_arm` simulation model is controlled via velocity commands on `/human_arm/cmd_vel`.

**Automated bounce** -- sweeps the arm along the Y axis (±3 m at 0.2 m/s):
```bash
ros2 run spraying_pathways moving_obstacle_node
```

**Manual velocity** -- publishes continuously until Ctrl+C:
```bash
ros2 topic pub /human_arm/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -0.2, z: 0.0}}"
```

**Stop the arm:**
```bash
ros2 topic pub --once /human_arm/cmd_vel geometry_msgs/msg/Twist "{}"
```

When the arm enters the camera's FOV, `flat_fan_spraying_v4_node` pauses automatically and resumes once the obstacle clears. Red spheres appear on `/obstacle_centroids` in RViz while the obstacle is detected.

To change the arm's starting position permanently, edit the `<pose>` of the `human_arm` include in `worlds/table_world.world`.

### Opening Additional Terminals

```bash
docker exec -it vulcanexus-container /bin/bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Running with FIWARE (Optional)

The spraying pipeline can be connected to the RAPSEB FIWARE stack for data logging and Grafana dashboards. See the [rapseb-fiware README](../rapseb-fiware/README.md) for integration instructions.

The integration point is the `/rapseb/spray_status` topic (std_msgs/String, JSON payload). The FIWARE ROS bridge node subscribes to this topic and pushes spray data to Orion-LD.

## Key ROS Topics

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| /joint_states | sensor_msgs/JointState | ros2_control | Robot joint positions and velocities |
| /lidar/scan/points | sensor_msgs/PointCloud2 | ros_gz_bridge | LiDAR point cloud |
| /depth_camera/points | sensor_msgs/PointCloud2 | ros_gz_bridge | Overhead depth camera point cloud |
| /wrist_camera/points | sensor_msgs/PointCloud2 | ros_gz_bridge | Wrist depth camera point cloud |
| /unknown_points | sensor_msgs/PointCloud2 | pointcloud filter | Unclassified points (potential obstacles) |
| /obstacle_centroids | visualization_msgs/MarkerArray | obstacles_tracking | DBSCAN cluster centroids of detected obstacles (red spheres in RViz) |
| scan_cloud | sensor_msgs/PointCloud2 | lidar_surface_scanner | Accumulated scan result |
| /spray_plan | std_msgs/Float64MultiArray | flat_fan_spraying_v4 | Grid layout: [size_x, size_y, N, x0, y0, x1, y1, ...]. Transient-local QoS. |
| /spray_current_idx | std_msgs/Int32 | flat_fan_spraying_v4 | Index of spray centre nearest the end-effector, published at ~4 Hz during execution |
| /epoxy_coating_markers | visualization_msgs/MarkerArray | epoxy_visualizer | Live RViz coating cubes coloured by layer count |
| /rapseb/robot_mode | std_msgs/String | hri_safety_guard | NORMAL, REDUCED, or STOPPED |
| /rapseb/spray_status | std_msgs/String | spraying nodes | JSON spray events (FIWARE integration) |
| /humans/bodies/tracked | hri_msgs/IdsList | ros4hri | Tracked human body IDs |

## Technology Stack

| Component | Version | Purpose |
|-----------|---------|---------|
| ROS 2 | Jazzy | Robot middleware |
| Vulcanexus | Jazzy Desktop | ROS 2 distribution with DDS |
| Gz Harmonic | 8.x | Physics simulation |
| MoveIt 2 | Jazzy | Motion planning |
| ros2_control | Jazzy | Controller framework |
| gz_ros2_control | Jazzy | Gz Harmonic ros2_control bridge |
| ros_gz_bridge | Jazzy | Gz transport to ROS 2 bridge |
| Universal Robots | UR10e | 6-DOF manipulator |
| PCL | 1.14 | Point cloud processing |

## Maintainer

Parity Platform P.C. (info@parityplatform.com)
