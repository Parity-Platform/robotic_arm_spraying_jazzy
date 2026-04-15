from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rapseb_hri_safety',
            executable='hri_safety_guard',
            name='hri_safety_guard',
            output='screen',
            parameters=[
                {'base_frame': 'base_link'},
                {'stop_distance_m': 1.0},
                {'warn_distance_m': 1.5},
                {'controller_manager_ns': '/controller_manager'},
                {'trajectory_controller': 'scaled_joint_trajectory_controller'},
                # Uncomment for real UR hardware with ur_robot_driver:
                # {'speed_slider_service': '/dashboard_client/set_speed_slider_fraction'},
                {'reduced_speed_pct': 20},
                {'monitor_rate_hz': 20.0},
            ],
        )
    ])
