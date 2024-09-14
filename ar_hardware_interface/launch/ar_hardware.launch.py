from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

def launch_setup(context, *args, **kwargs):
    serial_port = LaunchConfiguration("serial_port")
    calibrate = LaunchConfiguration("calibrate")
    include_gripper = LaunchConfiguration("include_gripper")
    include_track = LaunchConfiguration("include_track")
    arduino_serial_port = LaunchConfiguration("arduino_serial_port")
    ar_model_config = LaunchConfiguration("ar_model")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ar_hardware_interface"), "urdf", "ar.urdf.xacro"
        ]),
        " ",
        "ar_model:=",
        ar_model_config,
        " ",
        "serial_port:=",
        serial_port,
        " ",
        "calibrate:=",
        calibrate,
        " ",
        "include_gripper:=",
        include_gripper,
        " ",
        "include_track:=",
        include_track,
        " ",
        "arduino_serial_port:=",
        arduino_serial_port,
    ])
    robot_description = {"robot_description": robot_description_content}

    # Controllers file depends on whether track is enabled
    if include_track.perform(context) == 'True':
        controllers_file = "track_controllers.yaml"
    else:
        controllers_file = "controllers.yaml"

    joint_controllers_cfg = PathJoinSubstitution([
        FindPackageShare("ar_hardware_interface"), "config", controllers_file
    ])

    update_rate_config_file = PathJoinSubstitution([
        FindPackageShare("ar_hardware_interface"),
        "config",
        "controller_update_rate.yaml",
    ])

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            update_rate_config_file,
            robot_description,
            ParameterFile(joint_controllers_cfg, allow_substs=True),
        ],
        output="screen",
    )

    spawn_joint_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        condition=IfCondition(include_gripper),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    nodes_to_start = [controller_manager_node,spawn_joint_controller,gripper_controller_spawner,robot_state_publisher_node,joint_state_broadcaster]
    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port to connect to the robot",
        ))
    declared_arguments.append(
        DeclareLaunchArgument(
            "calibrate",
            default_value="True",
            description="Calibrate the robot on startup",
            choices=["True", "False"],
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
            "arduino_serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port of the Arduino nano for the servo gripper",
        ))
    declared_arguments.append(
        DeclareLaunchArgument("ar_model",
                              default_value="ar4_mk3",
                              choices=["ar4", "ar4_mk3"],
                              description="Model of AR4"))
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
