# Copyright (c) 2021 PickNik, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Denis Stogl

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    robot_type = LaunchConfiguration("robot_type")
    robot_ip_1 = LaunchConfiguration("robot_ip_1")
    robot_ip_2 = LaunchConfiguration("robot_ip_2")
    dof_1 = LaunchConfiguration("dof_1")
    dof_2 = LaunchConfiguration("dof_2")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    gripper_1 = LaunchConfiguration("gripper_1")
    gripper_2 = LaunchConfiguration("gripper_2")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    gripper_joint_name_1 = LaunchConfiguration("gripper_joint_name_1")
    gripper_joint_name_2 = LaunchConfiguration("gripper_joint_name_2")
    launch_rviz = LaunchConfiguration("launch_rviz")
    controllers_file_1 = LaunchConfiguration("controllers_file_1")
    controllers_file_2 = LaunchConfiguration("controllers_file_2")
    prefix_1 = LaunchConfiguration("prefix_1")
    prefix_2 = LaunchConfiguration("prefix_2")

    base_launch_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("kortex_bringup"), "launch", "kortex_control.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "robot_type": robot_type,
            "robot_ip": robot_ip_1,
            "dof": dof_1,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "gripper": gripper_1,
            "use_internal_bus_gripper_comm": use_internal_bus_gripper_comm,
            "gripper_max_velocity": gripper_max_velocity,
            "gripper_max_force": gripper_max_force,
            "gripper_joint_name": gripper_joint_name_1,
            "launch_rviz": launch_rviz,
            "controllers_file": controllers_file_1,
            "description_file": "gen3.xacro",
            "prefix": prefix_1,
        }.items(),
    )

    base_launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("kortex_bringup"), "launch", "kortex_control.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "robot_type": robot_type,
            "robot_ip": robot_ip_2,
            "dof": dof_2,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "gripper": gripper_2,
            "use_internal_bus_gripper_comm": use_internal_bus_gripper_comm,
            "gripper_max_velocity": gripper_max_velocity,
            "gripper_max_force": gripper_max_force,
            "gripper_joint_name": gripper_joint_name_2,
            "launch_rviz": launch_rviz,
            "controllers_file": controllers_file_2,
            "description_file": "gen3.xacro",
            "prefix": prefix_2,
        }.items(),
    )

    launch_files = [base_launch_1, base_launch_2]

    return launch_files


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="gen3",
            description="Type/series of robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip_1",
            default_value="",
            description="IP address by which robot_1 can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip_2",
            default_value="",
            description="IP address by which robot_2 can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "dof_1", default_value="7", description="DoF of the first robotic arm"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "dof_2", default_value="7", description="DoF of the second robotic arm"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file_1",
            default_value="ros2_controllers_parametric.yaml",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file_2",
            default_value="ros2_controllers_parametric.yaml",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_1",
            default_value="",
            description="Name of the gripper attached to the arm",
            choices=["", "robotiq_2f_85", "robotiq_2f_140"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_2",
            default_value="",
            description="Name of the gripper attached to the arm",
            choices=["", "robotiq_2f_85", "robotiq_2f_140"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_joint_name_1",
            default_value="robotiq_85_left_knuckle_joint",  # Adding prefix here breaks things
            description="Name of the gripper attached to the arm",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_joint_name_2",
            default_value="robotiq_85_left_knuckle_joint",  # Adding prefix here breaks things
            description="Name of the gripper attached to the arm",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_internal_bus_gripper_comm",
            default_value="true",
            description="Use internal bus for gripper communication?",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_velocity",
            default_value="100.0",
            description="Max velocity for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_max_force",
            default_value="100.0",
            description="Max force for gripper commands",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix_1", default_value="arm_1_", description="Prefix to differentiate arms"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix_2", default_value="arm_2_", description="Prefix to differentiate arms"
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
