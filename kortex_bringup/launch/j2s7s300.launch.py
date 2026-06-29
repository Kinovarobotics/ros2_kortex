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
# Adapted from kortex_bringup/launch/gen3.launch.py
# Original: Kinovarobotics/ros2_kortex (Apache 2.0)
# Modifications: AABL Lab
#   - j2s7s300 arm (Gen2 USB, no robot_ip)
#   - gripper_com_port arg for serial Robotiq (/dev/robotiq or /dev/tty_gripper)
#   - use_internal_bus_gripper_comm always false (serial Robotiq)
#   - No fault_controller, no twist_controller

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


def generate_launch_description():
    declared_arguments = []

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
            description="Enable fake command interfaces for sensors used for simple simulations. "
            "Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_com_port",
            default_value="/dev/robotiq",
            description="Serial port for the Robotiq gripper. "
            "Use /dev/tty_gripper for beep/boop robots.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            description="Arm trajectory controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="",
            description="Gripper attached to arm. Pass robotiq_2f_85 when gripper is connected.",
        )
    )

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    gripper_com_port = LaunchConfiguration("gripper_com_port")
    robot_controller = LaunchConfiguration("robot_controller")
    controllers_file = LaunchConfiguration("controllers_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gripper = LaunchConfiguration("gripper")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/kortex_control.launch.py"]),
        launch_arguments={
            "robot_type": "j2s7s300",
            "dof": "7",
            # No robot_ip — Gen2 is USB not Ethernet
            "robot_ip": "",
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "robot_controller": robot_controller,
            "controllers_file": controllers_file,
            "robot_pos_controller": "",
            "description_file": "j2s7s300_robotiq_2f_85.xacro",
            "gripper": gripper,
            "gripper_joint_name": "robotiq_85_left_knuckle_joint",
            # Serial Robotiq — never use internal bus
            "use_internal_bus_gripper_comm": "false",
            "gripper_max_velocity": "100.0",
            "gripper_max_force": "100.0",
            "gripper_com_port": gripper_com_port,
            "launch_rviz": launch_rviz,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [base_launch])