# Copyright (c) 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# I really couldn't find another way than this to set the yaml file to be used based on launch-config-argument..
def launch_setup(context, *args, **kwargs):
    ar_model_config = LaunchConfiguration("ar_model")
    include_gripper = LaunchConfiguration("include_gripper")
    include_track = LaunchConfiguration("include_track")

    # Controllers file depends on whether track is enabled
    if include_track.perform(context) == 'True':
        controllers_file = "track_controllers.yaml"
    else:
        controllers_file = "controllers.yaml"

    initial_joint_controllers = PathJoinSubstitution([
        FindPackageShare("ar_hardware_interface"), "config", controllers_file
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ar_description"), "urdf", "ar_gazebo.urdf.xacro"
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
        " ",
        "simulation_controllers:=",
        initial_joint_controllers,
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", "-c", "/controller_manager",
            "--controller-manager-timeout", "60"
        ],
        parameters=[{
            "use_sim_time": True
        }],
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller", "-c", "/controller_manager",
            "--controller-manager-timeout", "60"
        ],
        parameters=[{
            "use_sim_time": True
        }],
    )

    gripper_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller", "-c", "/controller_manager",
            "--controller-manager-timeout", "60"
        ],
        parameters=[{
            "use_sim_time": True
        }],
    )


    # Gazebo nodes
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
        parameters=[{
            "use_sim_time": True
        }],
    )
    nodes_to_start = [robot_state_publisher_node,joint_state_broadcaster_spawner,initial_joint_controller_spawner_started,gripper_joint_controller_spawner_started,gazebo,gazebo_spawn_robot]
    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("ar_model",
                                         default_value="ar4_mk3",
                                         choices=["ar4", "ar4_mk3"],
                                         description="Model of AR4"))
    declared_arguments.append(DeclareLaunchArgument(
                                         "include_gripper",
                                         default_value="True",
                                         description="Run the servo gripper",
                                         choices=["True", "False"]))
    declared_arguments.append(DeclareLaunchArgument(
                                        "include_track",
                                        default_value="True",
                                        description="Include a linear track",
                                        choices=["True", "False"]))
    

    return LaunchDescription(declared_arguments+[OpaqueFunction(function=launch_setup)])
