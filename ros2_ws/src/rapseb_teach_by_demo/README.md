# rapseb_teach_by_demo

Teach-by-demonstration pipeline for the RAPSEB autonomous epoxy spraying workcell. Lets a manual PHEE operator record a spraying trajectory with a Logitech F710 gamepad, then converts the recording into a `trajectory_output.csv` that the existing `spraying_pathways` executor can replay. The main spraying pipeline stays fully functional without this package; it is an optional input source.

## Why

The D3 evaluation asked for a documented imitation learning pipeline. The previous demonstrator used an algorithmic flat-fan coverage grid only. This package closes the loop: an experienced PHEE operator shows the robot how they would spray a specific panel, and the system captures the path plus the spraying start/stop moments, cleans the signal, and exports a trajectory in the exact schema the spraying executor already consumes.

## Hardware

Logitech F710 wireless gamepad in XInput mode (MODE LED off, back switch set to `X`). The stock ROS 2 `joy` driver handles it without extra setup. Chosen over the Xbox or Logitech F310 because it is wireless, battery powered, and has two analogue triggers (needed for press-and-hold spray).

## Button map

| Input | Function |
|---|---|
| Left stick X/Y | Cartesian Y / X jog (jog mode only) |
| Right stick X/Y | Yaw / Z jog (jog mode only) |
| Right trigger (hold) | Spray ON |
| A | Start a new recording |
| B | Stop the recording and flush to CSV |
| X | Panic: zero twist |
| Y | Toggle UR freedrive (real hardware only) |
| LB / RB | Decrease / increase jog scale |
| Start | Mark a segment boundary (new spray pass) |
| Back (hold) | Boost jog scale temporarily |
| Logo | Emergency cancel all active goals |

## Modes

- **Kinesthetic (default, real UR10e).** The UR is put into freedrive via the dashboard client. The operator physically moves the tool along the panel surface, pressing the right trigger where spraying should occur. The gamepad does not drive the arm in this mode, it only signals spray and segment markers.
- **Jog (simulation, myCobot, or robots without freedrive).** The gamepad publishes Cartesian twist into MoveIt Servo, which streams joint trajectories to the controller. Used during development and for the myCobot validation cell.

## Runtime pipeline

```
Logitech F710 ── joy_node ── joy_teleop ──┬── /servo_node/delta_twist_cmds (jog mode)
                                          ├── /rapseb/spray_trigger (Bool, press and hold)
                                          ├── /rapseb/segment_marker (Empty)
                                          └── /rapseb/record_start | record_stop

UR10e / myCobot ── /tf ────────────────────── demo_recorder ── demos/demo_*_raw.csv

demos/demo_*_raw.csv ── trajectory_extractor ── trajectory_output.csv
                                                 (spray flag appended)
```

## Build

```bash
cd robotic_arm_spraying_jazzy/ros2_ws
colcon build --packages-select rapseb_teach_by_demo --symlink-install
source install/setup.bash
```

Python dependencies: `numpy` (required), `scipy` (optional, used for the zero-phase Butterworth low-pass; falls back to a moving average if missing).

## Record a demo

On the real workcell (ur_robot_driver running, dashboard client available):

```bash
ros2 launch rapseb_teach_by_demo teach_demo.launch.py mode:=kinesthetic
```

1. Press **Y** to enable freedrive on the UR.
2. Press **A** to start recording.
3. Guide the tool along the panel. Press and hold the **right trigger** where you want the robot to spray.
4. Press **Start** at each pass boundary if you want segment markers.
5. Press **B** to stop. The raw CSV is written to `demos/demo_YYYYMMDD_HHMMSS_raw.csv`.
6. Press **Y** to disable freedrive.

In simulation or on the myCobot:

```bash
ros2 launch rapseb_teach_by_demo teach_demo.launch.py mode:=jog
```

MoveIt Servo must be running (part of the main bringup). The gamepad now drives the arm, and spray trigger plus recording controls behave identically.

## Post-process the demo

```bash
ros2 run rapseb_teach_by_demo trajectory_extractor demos/demo_20260415_103211_raw.csv \
  --output trajectory_output.csv \
  --cutoff-hz 8.0 \
  --dt 0.025
```

Optional panel frame override:

```bash
... --panel-origin 0.5,0.3,0.1 --panel-normal 0,0,1
```

Output CSV matches the `spraying_pathways` executor schema, with one extra column:

```
time [s], x [m], y [m], speed [m/s], acceleration [m/s^2], spray [bool]
```

The main spraying node can either be extended to honour the spray flag, or the `replay_publisher` node can be run alongside it to broadcast `/rapseb/spray_status` per waypoint.

## Quality metrics

The extractor preserves the trajectory within the tolerances already reported in D2 and D3:

- Resampled at 25 ms, matching the MoveIt Servo publish period.
- Zero-phase Butterworth low-pass at 8 Hz removes hand jitter below the sub-millimetre level of the UR10e repeatability.
- Spray flag majority-voted in each output bin to avoid single-sample glitches.

The synthetic-demo test (`test/test_trajectory_extractor.py`) checks header conformance, monotonic time, non-negative speed, oscillation preservation, and spray flag coverage.

## Limits and future work

- The first release captures 2D plane spraying. Curved panels are projected onto a single plane; a multi-patch extension is already supported by the main `spraying_grid.hpp` and can be added in a follow-up by emitting a patch index column.
- Behavioural cloning with a learned policy is scoped for a D4 follow-up. The current release stays deterministic (record -> smooth -> replay) because PHEE operators want a traceable, reviewable trajectory file per batch. The file format leaves room for a future `source` column (`human`, `algorithmic`, `learned`) without breaking the executor.
