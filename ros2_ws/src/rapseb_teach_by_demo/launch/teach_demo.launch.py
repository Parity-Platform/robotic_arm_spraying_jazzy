"""Bring up joy driver, joy_teleop, demo_recorder and the freedrive gateway.

Modes:
  mode:=kinesthetic   (default on real UR10e) - freedrive gateway active,
                      Servo is not launched; gamepad only signals spray and
                      segment markers.
  mode:=jog           (simulation / myCobot) - MoveIt Servo is expected to be
                      running from the main bringup; gamepad jogs the arm.

Usage:
  ros2 launch rapseb_teach_by_demo teach_demo.launch.py mode:=kinesthetic
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    device = LaunchConfiguration('device').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)

    share = get_package_share_directory('rapseb_teach_by_demo')
    joy_cfg = os.path.join(share, 'config', 'logitech_f710.yaml')

    nodes = [
        Node(
            package='joy', executable='joy_node', name='joy',
            parameters=[{'device_id': int(device), 'deadzone': 0.02}],
        ),
        Node(
            package='rapseb_teach_by_demo', executable='joy_teleop',
            name='joy_teleop', parameters=[joy_cfg],
        ),
        Node(
            package='rapseb_teach_by_demo', executable='demo_recorder',
            name='demo_recorder',
            parameters=[{
                'base_frame': 'base_link',
                'ee_frame': 'tool0',
                'sample_rate_hz': 100.0,
                'output_dir': output_dir,
            }],
        ),
        Node(
            package='rapseb_teach_by_demo', executable='freedrive_gateway',
            name='freedrive_gateway',
            parameters=[{'simulation': mode == 'jog'}],
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='kinesthetic',
                              choices=['kinesthetic', 'jog']),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('output_dir', default_value='demos'),
        OpaqueFunction(function=_launch_setup),
    ])
