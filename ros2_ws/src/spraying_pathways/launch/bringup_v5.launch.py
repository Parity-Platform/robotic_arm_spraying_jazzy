from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import yaml
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # LaunchConfigurations
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    render_engine = LaunchConfiguration("render_engine")

    # Robot description
    initial_joint_controllers = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), "config", controllers_file
    ])
    initial_positions_file_abs = PathJoinSubstitution([
        FindPackageShare(runtime_config_package), "config", initial_positions_file
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), " ",
        "safety_limits:=", safety_limits, " ",
        "safety_pos_margin:=", safety_pos_margin, " ",
        "safety_k_position:=", safety_k_position, " ",
        "name:=ur ur_type:=", ur_type, " ",
        "prefix:=", prefix, " ",
        "sim_gazebo:=true ",
        "simulation_controllers:=", initial_joint_controllers, " ",
        "initial_positions_file:=", initial_positions_file_abs, " ",

        # These 4 lines are essential:
        "joint_limit_params:=", PathJoinSubstitution([
            FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"
        ]), " ",
        "kinematics_params:=", PathJoinSubstitution([
            FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"
        ]), " ",
        "physical_params:=", PathJoinSubstitution([
            FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"
        ]), " ",
        "visual_params:=", PathJoinSubstitution([
            FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"
        ])
    ])


    robot_description = {"robot_description": robot_description_content}

    # Robot semantic description (SRDF)
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare(moveit_config_package), "srdf", moveit_config_file
        ]), " ",
        "name:=ur ",
        "prefix:=", prefix, " ",
    ])
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    publish_robot_description_semantic_param = {
        "publish_robot_description_semantic": publish_robot_description_semantic
    }

    src_spraying_pathways_pkg = get_package_share_directory('spraying_pathways')

    # === Load MoveIt Configs ===
    moveit_config_pkg = get_package_share_directory('ur_moveit_config')

    with open(os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml'), 'r') as f:
        kinematics = yaml.safe_load(f)

    with open(os.path.join(moveit_config_pkg, 'config', 'joint_limits.yaml'), 'r') as f:
        joint_limits = yaml.safe_load(f)

    with open(os.path.join(moveit_config_pkg, 'config', 'ompl_planning.yaml'), 'r') as f:
        ompl = yaml.safe_load(f)

    with open(os.path.join(src_spraying_pathways_pkg, 'config', 'controllers.yaml'), 'r') as f:
        controllers = yaml.safe_load(f)

    robot_description_kinematics = {
        "robot_description_kinematics": kinematics
    }

    robot_description_planning = {
        "robot_description_planning": joint_limits
    }
    ompl_config = {
        "move_group": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
            "start_state_max_bounds_error": 0.1,
            **ompl,
        }
    }

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 10.0,
        "trajectory_execution.allowed_goal_duration_margin": 10.0,
        "trajectory_execution.allowed_start_tolerance": 0.05,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    initial_joint_controller_start = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_stop = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    spawn_ur = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "ur", "-topic", "robot_description", "-x", "0.25", "-y", "0", "-z", "0.715"],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            initial_joint_controllers,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic_param,
            robot_description_kinematics,
            robot_description_planning,
            ompl_config,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", os.path.join(moveit_config_pkg, "config", "moveit.rviz")],
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic_param,
            robot_description_kinematics,
            robot_description_planning,
            ompl_config,
            {"use_sim_time": True},
        ],
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments={"gz_args": ["-r --render-engine ", render_engine, " ", world_file]}.items() # ogre2 needed for depth_camera sensor; use ogre if ogre2 crashes
    )

    # Sensor topic bridge (Gz <-> ROS 2)
    gz_bridge_config = os.path.join(src_spraying_pathways_pkg, 'config', 'gz_bridge.yaml')
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={gz_bridge_config}"],
        output="screen",
    )

    pointcloud_transform_and_unknown_filter_script = TimerAction(
        period=10.0,  # Delay in seconds to let the system stabilize
        actions=[
            ExecuteProcess(
                cmd=["ros2", "run", "spraying_pathways", "pointcloud_transform_and_unknown_filter_v3.py", "/ros2_ws/src/spraying_pathways/worlds/table_world.world"],
                shell=True,
                output="screen"
            )
        ]
    )

    # Starts after the transform node (10s) so /unknown_points is already flowing
    obstacles_tracking_node = TimerAction(
        period=13.0,
        actions=[
            Node(
                package="spraying_pathways",
                executable="obstacles_tracking.py",
                name="obstacles_tracking",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    # Waits for /spray_plan and /spray_current_idx from the spraying C++ node
    epoxy_visualizer_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="spraying_pathways",
                executable="epoxy_visualizer.py",
                name="epoxy_visualizer",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ]
    )




    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_ur,
            on_exit=[
                joint_state_broadcaster,
                initial_joint_controller_start,
                initial_joint_controller_stop,
            ],
        )
    )

    return [
        gzserver,
        gz_bridge_node,
        robot_state_publisher_node,
        ros2_control_node,
        spawn_ur,
        spawn_controllers,
        move_group_node,
        rviz_node,
        pointcloud_transform_and_unknown_filter_script,
        obstacles_tracking_node,
        epoxy_visualizer_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur10e"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("runtime_config_package", default_value="ur_simulation_gazebo"),
        DeclareLaunchArgument("controllers_file", default_value="ur_controllers.yaml"),
        DeclareLaunchArgument("initial_positions_file", default_value=PathJoinSubstitution([
            FindPackageShare("ur_description"), "config", "initial_positions.yaml"
        ])),
        DeclareLaunchArgument("description_package", default_value="spraying_pathways"),
        DeclareLaunchArgument("description_file", default_value="my_robot.urdf.xacro"),
        DeclareLaunchArgument("prefix", default_value="\"\""),
        DeclareLaunchArgument("start_joint_controller", default_value="true"),
        DeclareLaunchArgument("initial_joint_controller", default_value="joint_trajectory_controller"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("moveit_config_package", default_value="ur_moveit_config"),
        DeclareLaunchArgument("moveit_config_file", default_value="ur.srdf.xacro"),
        DeclareLaunchArgument("world_file", default_value="/ros2_ws/src/spraying_pathways/worlds/table_world.world"),
        DeclareLaunchArgument("publish_robot_description_semantic", default_value="true", description="Whether to publish the SRDF description on /robot_description_semantic."),
        DeclareLaunchArgument("render_engine", default_value="ogre", description="Gazebo render engine. Use ogre2 if ogre crashes on your setup (depth camera might not work)."),
        OpaqueFunction(function=launch_setup),
    ])