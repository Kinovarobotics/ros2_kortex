# Copyright (c) 2024 PickNik, Inc.
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
# Author: Anthony Baker

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = []
    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Use Ignition for simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated clock",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    # Initialize Arguments
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_ignition = LaunchConfiguration("sim_ignition")

    description_arguments = {
        "robot_ip": "xxx.yyy.zzz.www",
        "use_fake_hardware": "false",
        "gripper": "gen3_lite_2f",
        "dof": "6",
        "sim_ignition": sim_ignition,
    }

    moveit_config = (
        MoveItConfigsBuilder("gen3_lite", package_name="kinova_gen3_lite_moveit_config")
        .robot_description(mappings=description_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        arguments=[
            "--ros-args",
            "--log-level",
            "fatal",
        ],  # MoveIt is spamming the log because of unknown '*_mimic' joints
        condition=IfCondition(launch_rviz),
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("kinova_gen3_lite_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(declared_arguments + [move_group_node, rviz_node])
