import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():


    ar_model_arg = DeclareLaunchArgument("ar_model",
                                         default_value="ar4_mk3",
                                         choices=["ar4", "ar4_mk3"],
                                         description="Model of AR4")
    ar_model_config = LaunchConfiguration("ar_model")


    # Configure the node
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ar_description"), "urdf", "ar_gazebo.urdf.xacro"
        ]),
        " ",
        "ar_model:=",
        ar_model_config,
        " "
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{
            "use_sim_time": True
        }, robot_description],
    )


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch",
             "/gazebo.launch.py"]), )
# Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ar",
        arguments=[
            "-entity", "ar", "-topic", "robot_description", "-timeout", "60"
        ],
        output="screen",
    )

#    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
#                    arguments=['-topic', 'robot_description',
#                                '-entity', 'ar_robot'],
#                    output='screen')






    # Run the node
    return LaunchDescription([
        ar_model_arg,
        gazebo,
        robot_state_publisher_node,
        gazebo_spawn_robot
    ])