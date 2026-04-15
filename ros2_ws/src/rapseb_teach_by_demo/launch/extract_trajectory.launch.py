"""Run the trajectory_extractor in its CLI mode via ros2 launch.

Example:
  ros2 launch rapseb_teach_by_demo extract_trajectory.launch.py \
      input_csv:=/path/to/demo_20260415_103211_raw.csv \
      output_csv:=trajectory_output.csv
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    input_csv = LaunchConfiguration('input_csv')
    output_csv = LaunchConfiguration('output_csv')
    cutoff = LaunchConfiguration('cutoff_hz')
    dt_out = LaunchConfiguration('dt')
    return LaunchDescription([
        DeclareLaunchArgument('input_csv'),
        DeclareLaunchArgument('output_csv', default_value='trajectory_output.csv'),
        DeclareLaunchArgument('cutoff_hz', default_value='8.0'),
        DeclareLaunchArgument('dt', default_value='0.025'),
        ExecuteProcess(cmd=[
            'ros2', 'run', 'rapseb_teach_by_demo', 'trajectory_extractor',
            input_csv,
            '--output', output_csv,
            '--cutoff-hz', cutoff,
            '--dt', dt_out,
        ], output='screen'),
    ])
