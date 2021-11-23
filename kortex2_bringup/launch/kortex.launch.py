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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    ThisLaunchFileDir,
)

kortex_common_configurable_parameters = [
    {"name": "robot_type", "default": None, "description": "Type/series of robot."},
    {
        "name": "robot_ip",
        "default": None,
        "description": "IP address by which the robot can be reached.",
    },
    {
        "name": "description_package",
        "default": "kortex_description",
        "description": "Description package with robot URDF/XACRO files. Usually the argument is not set, it enables use of a custom description.",
    },
    {
        "name": "moveit_config_package",
        "default": "gen3_robotiq_2f_85_move_it_config",
        "description": "MoveIt configuration package for the robot. Usually the argument is not set, it enables use "
        "of a custom config package.",
    },
    {
        "name": "description_file",
        "default": "gen3.xacro",
        "description": "URDF/XACRO description file with the robot.",
    },
    {
        "name": "prefix",
        "default": '""',
        "description": "Prefix of the joint names, useful for multi-robot setup. If changed than also joint names in the controllers' configuration have to be updated.",
    },
    {
        "name": "gripper",
        "default": "robotiq_2f_85",
        "description": "Name of the gripper attached to the arm",
    },
    {
        "name": "use_fake_hardware",
        "default": "false",
        "description": "Start robot with fake hardware mirroring command to its states.",
    },
    {
        "name": "fake_sensor_commands",
        "default": "false",
        "description": "Enable fake command interfaces for sensors used for simple simulations. Used only if 'use_fake_hardware' parameter is true.",
    },
]

kortex_moveit_configurable_parameters = [
    {
        "name": "moveit_config_file",
        "default": "gen3_robotiq_2f_85.srdf.xacro",
        "description": "MoveIt SRDF/XACRO description file with the robot.",
    },
]
kortex_control_configurable_parameters = [
    {
        "name": "runtime_config_package",
        "default": "kortex2_bringup",
        "description": "Package with the controller's configuration in 'config' folder. Usually the argument is not set, it enables use of a custom setup.",
    },
    {
        "name": "controllers_file",
        "default": "kortex_controllers.yaml",
        "description": "YAML file with the controllers configuration.",
    },
    {
        "name": "robot_controller",
        "default": "joint_trajectory_controller",
        "description": "Robot controller to start.",
    },
    {
        "name": "robot_gripper_controller",
        "default": "gripper_controller",
        "description": "Robot hand controller to start.",
    },
]


def declare_configurable_parameters(parameters):
    return [
        DeclareLaunchArgument(
            param["name"],
            default_value=param["default"],
            description=param["description"],
        )
        for param in parameters
    ]


def set_configurable_parameters(parameters):
    return {param["name"]: LaunchConfiguration(param["name"]) for param in parameters}


def generate_launch_description():
    kortex_moveit_launch_arguments = set_configurable_parameters(
        kortex_common_configurable_parameters + kortex_moveit_configurable_parameters
    )
    kortex_moveit_launch_arguments.update({"launch_rviz": "true"})
    kortex_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ThisLaunchFileDir(), "kortex_moveit.launch.py"])
        ),
        launch_arguments=kortex_moveit_launch_arguments.items(),
    )

    kortex_control_launch_arguments = set_configurable_parameters(
        kortex_common_configurable_parameters + kortex_control_configurable_parameters
    )
    kortex_control_launch_arguments.update({"launch_rviz": "false"})
    kortex_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ThisLaunchFileDir(), "kortex_control.launch.py"])
        ),
        launch_arguments=kortex_control_launch_arguments.items(),
    )

    declared_arguments = declare_configurable_parameters(kortex_common_configurable_parameters)
    declared_arguments.extend(
        declare_configurable_parameters(kortex_moveit_configurable_parameters)
    )
    declared_arguments.extend(
        declare_configurable_parameters(kortex_control_configurable_parameters)
    )

    return LaunchDescription(declared_arguments + [kortex_moveit, kortex_control])
