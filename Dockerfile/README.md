## Building the Docker Image

To build the Docker image required for this project, navigate to the directory named `Dockerfile` (which contains the actual `Dockerfile`) and run the following command:

```bash
docker image build -t my-vulcanexus:jazzy-desktop .
```

## Running the Docker Container

To run the Docker container based on the previously built image, make sure you are located inside the directory that contains the `ros2_ws` folder. This folder is required because it will be mounted into the container at runtime. The `ros2_ws` directory is included in the GitHub project.

On Linux (not WSL2), allow X11 GUI forwarding first:
```bash
xhost +local:docker
```

```bash
docker run -it --rm --name vulcanexus-container --user vulcanexus_user \
  -v $PWD/ros2_ws:/ros2_ws -w /ros2_ws \
  --network=host --ipc=host \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  my-vulcanexus:jazzy-desktop
```

## First-Time Container Setup

Clone `gz_ros2_control` from source before building. The apt package (1.2.17) has a threading bug in `GazeboSimSystem::initSim`; the source build from the jazzy branch fixes it. Run this on the **host** (not inside the container), from the repo root:

```bash
cd ros2_ws/src
git clone https://github.com/ros-controls/gz_ros2_control.git --branch jazzy --depth 1 gz_ros2_control
```

Then inside the container:

```bash
cd /ros2_ws
rm -rf build/ log/ install/
rosdep update && rosdep install --ignore-src --from-paths . -y
colcon build --symlink-install --executor sequential
source install/setup.bash
```

## Opening a New Terminal Inside the Running Container


```bash
docker exec -it vulcanexus-container /bin/bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Rebuilding After Changes

With `--symlink-install`, Python scripts in `scripts/` are symlinked directly from source — edits take effect immediately without a rebuild. A full rebuild is only required after C++ source changes or when adding new packages.

```bash
cd /ros2_ws
colcon build --symlink-install --executor sequential
source install/setup.bash
```

If the build is stale or you hit unexplained errors, do a clean rebuild:

```bash
rm -rf build/ log/ install/
colcon build --symlink-install --executor sequential
source install/setup.bash
```

## Visualizing the Robot in Gz Harmonic or RViz

```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur10e
ros2 launch ur_description view_ur.launch.py ur_type:=ur10
```

## Launching the Robot in Gazebo using `spraying_pathways` package with custom World and custom Urdf file

```bash
ros2 launch spraying_pathways bringup_v5.launch.py
```

## Sending the Robot to its Home Position

```bash
ros2 run spraying_pathways go_home_node
```

## Executing the Flat-Fan Spray with Progressive Visualisation

Make sure the robot is at its home position before starting.

The epoxy visualiser and obstacle tracking nodes now start automatically as part of `bringup_v5.launch.py`. No separate terminal is needed for them.

Run the spraying node:
```bash
ros2 run spraying_pathways flat_fan_spraying_v4_node
```

In RViz: click **Add** → **MarkerArray** and add these topics:

| Topic | What you see |
|-------|-------------|
| `/epoxy_coating_markers` | Coloured cubes on the panel growing in real time |
| `/obstacle_centroids` | Red spheres at detected obstacle positions |
| `/known_objects_markers` | Green shapes (table, panel -- already known) |
| `/robot_objects_markers` | Blue shapes (robot self-filter volumes) |

Cube colours encode how many times each cell has been coated:

| Colour | Layer count |
|--------|-------------|
| Yellow (semi-transparent) | 1 |
| Orange | 2 |
| Deep orange | 3 |
| Dark red | 4+ |

Each additional run of `flat_fan_spraying_v4_node` adds another layer on top. Layer data is saved to `/tmp/epoxy_layers.json` between runs. To reset all layer history:
```bash
rm /tmp/epoxy_layers.json
```

## Testing Obstacle Detection

The `human_arm` model in the simulation world is controlled via velocity commands on `/human_arm/cmd_vel` (bridged from ROS 2 to Gz via ros_gz_bridge).

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

When the arm enters the camera's field of view, `pointcloud_transform_and_unknown_filter_v3.py` will publish points on `/unknown_points`. `obstacles_tracking.py` clusters them and `flat_fan_spraying_v4_node` pauses automatically. The spraying resumes once the obstacle clears.

To change the arm's starting position, edit the `<pose>` of the `human_arm` include in `worlds/table_world.world`.

## Executing Surface Scan or Scan & Glue Spraying

Before running any of the commands below, make sure the robot is already at its home position.

```bash
ros2 run spraying_pathways cartesian_path_planner_trajectory_v1_node
ros2 run spraying_pathways cartesian_path_planner_cubes_v2_node
ros2 run spraying_pathways cartesian_path_planner_cubes_v3_node
ros2 run spraying_pathways cartesian_path_planner_cubes_v4_node
```
For fixed position of problematic glue cubes run the scripts below

```bash
ros2 run spraying_pathways cartesian_path_planner_cubes_test_v1_node
ros2 run spraying_pathways cartesian_path_planner_cubes_test_v2_node
ros2 run spraying_pathways cartesian_path_planner_cubes_test_go_v1_node
ros2 run spraying_pathways cartesian_path_planner_cubes_test_go_v2_node
```
## Executing Lidar Scan

Before running the command below, make sure the robot is already at its home position.

```bash
  ros2 run spraying_pathways lidar_surface_scanner_node
```
