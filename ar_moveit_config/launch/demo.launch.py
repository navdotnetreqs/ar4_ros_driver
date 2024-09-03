import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

def load_yaml(package_name, file_name):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_name)
    with open(absolute_file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)

def launch_setup(context, *args, **kwargs):
    ar_model_config = LaunchConfiguration("ar_model")
    include_gripper = LaunchConfiguration("include_gripper")
    include_track = LaunchConfiguration("include_track")

    robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    PathJoinSubstitution([
        FindPackageShare("ar_moveit_config"), "urdf", "fake_ar.urdf.xacro"
    ]),
    " ",
    "ar_model:=",
    ar_model_config,
    " ",
    "include_gripper:=",
    include_gripper,
    " ",
    "include_track:=",
    include_track,
    ])
    robot_description = {"robot_description": robot_description_content}

   # MoveIt Configuration
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("ar_moveit_config"), "srdf", "ar.srdf.xacro"]),
        " ",
        "name:=",
        ar_model_config,
        " ",
        "include_gripper:=",
        include_gripper,
        " ",
        "include_track:=",
        include_track,
    ])
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "ar_moveit_config",
            os.path.join("config", "kinematics.yaml"),
        )
    }

    robot_description_planning = {
        "robot_description_planning":
        load_yaml(
            "ar_moveit_config",
            os.path.join("config", "joint_limits.yaml"),
        )
    }

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters":
            """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ar_moveit_config",
                                   "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Controllers file depends on whether track is enabled
    if include_track.perform(context) == 'True':
        controllers_file = "config/track_controllers.yaml"
    else:
        controllers_file = "config/controllers.yaml"

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("ar_moveit_config", controllers_file)

    moveit_controllers = {
        "moveit_simple_controller_manager":
        controllers_yaml,
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory("ar_moveit_config"),
                             "rviz")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # # Static TF
    # static_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_publisher",
    #     output="log",
    #     arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    # )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Controllers file depends on whether track is enabled
    if include_track.perform(context) == 'True':
        controllers_file = "track_controllers.yaml"
    else:
        controllers_file = "controllers.yaml"

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ar_hardware_interface"),
        "config",
        controllers_file,
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(ros2_controllers_path, allow_substs=True),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
        ],
    )

    joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-c",
            "/controller_manager",
        ],
    )

    # Warehouse mongodb server
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {
                "warehouse_port": 33829
            },
            {
                "warehouse_host": "localhost"
            },
            {
                "warehouse_plugin":
                "warehouse_ros_mongo::MongoDatabaseConnection"
            },
        ],
        output="screen",
        condition=IfCondition(db_config),
    )

    nodes_to_start = [rviz_node, robot_state_publisher, run_move_group_node, ros2_control_node, mongodb_server_node,joint_state_broadcaster_spawner, joint_controller_spawner, gripper_controller_spawner]
    return nodes_to_start


def generate_launch_description():

    # Command-line arguments
    db_arg = DeclareLaunchArgument("db",
                                   default_value="False",
                                   description="Database flag")
    declared_arguments = []
    declared_arguments.append(db_arg)
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Make MoveIt use simulation time. This is needed "+\
                "for trajectory planing in simulation.",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "include_gripper",
            default_value="True",
            description="Run the servo gripper",
            choices=["True", "False"],
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "include_track",
            default_value="True",
            description="Include a linear track",
            choices=["True", "False"],
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="moveit.rviz",
            description="Full path to the RViz configuration file to use",
        ))
    declared_arguments.append(
        DeclareLaunchArgument("ar_model",
                              default_value="ar4_mk3",
                              choices=["ar4", "ar4_mk3"],
                              description="Model of AR4"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])