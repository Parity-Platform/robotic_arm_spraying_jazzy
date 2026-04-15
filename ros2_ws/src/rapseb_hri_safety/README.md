# rapseb_hri_safety

Human-Robot Interaction safety guard for the RAPSEB UR10e spraying workcell. Monitors human proximity using ros4hri body tracking and enforces ISO 10218-compliant safety zones by pausing/resuming the trajectory controller and (optionally) adjusting the UR speed slider.

This package is an **optional add-on**. The spraying pipeline runs independently; this node only intervenes when a tracked human enters the configured zones.

## Safety zones

| Zone | Distance            | Action                                           |
|------|---------------------|--------------------------------------------------|
| Z1   | < 1.0 m (default)   | Deactivate trajectory controller, speed slider 0 |
| Z2   | < 1.5 m (default)   | Reduce speed slider to 20 %                      |
| Z3   | >= 1.5 m            | Normal operation                                 |

Distances are Euclidean from `base_link` to the tracked human TF frame.

## Dependencies

- ROS 2 Jazzy
- ros4hri + hri_msgs (hard dependency)
- controller_manager_msgs
- tf2_ros
- ur_dashboard_msgs (optional, for speed slider on real UR hardware)

## Build

```bash
cd ros2_ws
colcon build --packages-select rapseb_hri_safety
source install/setup.bash
```

## Launch

```bash
ros2 launch rapseb_hri_safety hri_safety_guard.launch.py
```

Override parameters in the launch file or via command line:

```bash
ros2 launch rapseb_hri_safety hri_safety_guard.launch.py \
    stop_distance_m:=0.8 warn_distance_m:=1.2
```

## Parameters

| Parameter               | Default                                | Description                          |
|-------------------------|----------------------------------------|--------------------------------------|
| base_frame              | base_link                              | Robot base TF frame                  |
| stop_distance_m         | 1.0                                    | Z1 threshold (metres)                |
| warn_distance_m         | 1.5                                    | Z2 threshold (metres)                |
| controller_manager_ns   | /controller_manager                    | controller_manager namespace         |
| trajectory_controller   | scaled_joint_trajectory_controller     | Controller to activate/deactivate    |
| speed_slider_service    | (empty)                                | UR dashboard speed slider service    |
| reduced_speed_pct       | 20                                     | Speed % for Z2 zone                  |
| monitor_rate_hz         | 20.0                                   | Guard loop frequency                 |

## Published topics

| Topic               | Type              | Description        |
|---------------------|-------------------|--------------------|
| /rapseb/robot_mode  | std_msgs/String   | NORMAL, REDUCED, or STOPPED |

## Maintainer

Parity Platform P.C. (info@parityplatform.com)
