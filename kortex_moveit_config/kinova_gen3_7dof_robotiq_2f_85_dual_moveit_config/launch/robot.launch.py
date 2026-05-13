# Copyright (c) 2026 Kinova inc.
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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


PACKAGE_NAME = "kinova_gen3_7dof_robotiq_2f_85_dual_moveit_config"


def launch_setup(context, *args, **kwargs):
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")
    left_prefix = LaunchConfiguration("left_prefix")
    right_prefix = LaunchConfiguration("right_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")
    gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    gripper_max_force = LaunchConfiguration("gripper_max_force")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # URDF xacro mappings: forwarded to multiple_robots/kortex_dual_robots.xacro.
    urdf_mappings = {
        "left_arm": "gen3",
        "left_dof": "7",
        "left_gripper": "robotiq_2f_85",
        "left_gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "left_robot_ip": left_robot_ip.perform(context),
        "left_prefix": left_prefix.perform(context),
        "right_arm": "gen3",
        "right_dof": "7",
        "right_gripper": "robotiq_2f_85",
        "right_gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "right_robot_ip": right_robot_ip.perform(context),
        "right_prefix": right_prefix.perform(context),
        "use_fake_hardware": use_fake_hardware.perform(context),
        "use_internal_bus_gripper_comm": use_internal_bus_gripper_comm.perform(context),
        "gripper_max_velocity": gripper_max_velocity.perform(context),
        "gripper_max_force": gripper_max_force.perform(context),
    }

    srdf_mappings = {
        "left_prefix": left_prefix.perform(context),
        "right_prefix": right_prefix.perform(context),
    }

    moveit_config = (
        MoveItConfigsBuilder("dual_gen3", package_name=PACKAGE_NAME)
        .robot_description(mappings=urdf_mappings)
        .robot_description_semantic(
            file_path="config/dual_gen3.srdf.xacro", mappings=srdf_mappings
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    moveit_config.moveit_cpp.update({"use_sim_time": use_sim_time.perform(context) == "true"})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "config", "ros2_controllers.yaml"
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    def _spawner(name, inactive=False):
        args = [name, "-c", "/controller_manager"]
        if inactive:
            args.append("--inactive")
        return Node(package="controller_manager", executable="spawner", arguments=args)

    joint_state_broadcaster_spawner = _spawner("joint_state_broadcaster")

    left_arm_spawner = _spawner("left_arm_controller")
    right_arm_spawner = _spawner("right_arm_controller")
    left_gripper_spawner = _spawner("left_gripper_controller")
    right_gripper_spawner = _spawner("right_gripper_controller")
    left_twist_spawner = _spawner("left_twist_controller", inactive=True)
    right_twist_spawner = _spawner("right_twist_controller", inactive=True)

    left_fault_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_fault_controller", "-c", "/controller_manager"],
        condition=UnlessCondition(use_fake_hardware),
    )
    right_fault_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_fault_controller", "-c", "/controller_manager"],
        condition=UnlessCondition(use_fake_hardware),
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    delay_rviz_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    nodes_to_start = [
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_rviz_after_jsb,
        left_arm_spawner,
        right_arm_spawner,
        left_gripper_spawner,
        right_gripper_spawner,
        #left_twist_spawner,
        #right_twist_spawner,
        #left_fault_spawner,
        #right_fault_spawner,
        move_group_node,
    ]
    return nodes_to_start


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(
            "left_robot_ip",
            default_value="192.168.1.10",
            description="IP address of the left arm.",
        ),
        DeclareLaunchArgument(
            "right_robot_ip",
            default_value="192.168.1.11",
            description="IP address of the right arm.",
        ),
        DeclareLaunchArgument(
            "left_prefix",
            default_value="left_",
            description="Prefix applied to all left-arm joints/links.",
        ),
        DeclareLaunchArgument(
            "right_prefix",
            default_value="right_",
            description="Prefix applied to all right-arm joints/links.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use mock hardware. Defaults to true so the package can be run without physical arms.",
        ),
        DeclareLaunchArgument(
            "use_internal_bus_gripper_comm",
            default_value="true",
            description="Use the arm's internal bus to talk to the gripper. Must be true for real hardware to expose the gripper joint to ros2_control.",
        ),
        DeclareLaunchArgument(
            "gripper_max_velocity", default_value="100.0", description="Max gripper velocity."
        ),
        DeclareLaunchArgument(
            "gripper_max_force", default_value="100.0", description="Max gripper force."
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz with MoveIt config."
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulated clock."
        ),
    ]
    return LaunchDescription(declared + [OpaqueFunction(function=launch_setup)])
