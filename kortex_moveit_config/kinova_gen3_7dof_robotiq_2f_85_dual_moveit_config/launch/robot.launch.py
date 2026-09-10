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
from launch_ros.parameter_descriptions import ParameterValue
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
    hide_padding_in_rviz = LaunchConfiguration("hide_padding_in_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_walls = LaunchConfiguration("enable_walls")
    wall_offset = LaunchConfiguration("wall_offset")
    wall_height = LaunchConfiguration("wall_height")
    wall_thickness = LaunchConfiguration("wall_thickness")
    robot_padding = LaunchConfiguration("robot_padding")
    base_separation = LaunchConfiguration("base_separation")
    left_rpy = LaunchConfiguration("left_rpy")
    right_rpy = LaunchConfiguration("right_rpy")

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
        # Workspace walls. enable_walls MUST be forwarded to the SRDF too, or the
        # walls-vs-plate disable_collisions entry would reference a link that the
        # URDF did not create.
        "enable_walls": enable_walls.perform(context),
        "wall_offset": wall_offset.perform(context),
        "wall_height": wall_height.perform(context),
        "wall_thickness": wall_thickness.perform(context),
        # Mounting geometry. Correct these from measurement rather than padding
        # over the error: inter-arm clearance tracks base_separation error ~1:1,
        # and loses ~4.3 mm per degree of relative yaw at the demo pose.
        "base_separation": base_separation.perform(context),
        "left_rpy": left_rpy.perform(context),
        "right_rpy": right_rpy.perform(context),
    }

    srdf_mappings = {
        "left_prefix": left_prefix.perform(context),
        "right_prefix": right_prefix.perform(context),
        "struct_name": "dual_arm_structure",
        "enable_walls": enable_walls.perform(context),
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

    # Collision padding. MoveIt's CollisionEnv defaults to 0.0 padding, and nothing
    # in this config overrode it, so collision meant actual mesh contact with no
    # margin whatsoever -- planned paths were free to pass the other arm by a
    # fraction of a millimetre. Any tracking error then becomes a real collision.
    # PlanningSceneMonitor::configureDefaultPadding() reads these four names.
    padding_params = {
        "robot_description_planning.default_robot_padding": ParameterValue(
            robot_padding, value_type=float
        ),
        "robot_description_planning.default_attached_padding": ParameterValue(
            robot_padding, value_type=float
        ),
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), padding_params],
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

    # Display-only relay of the planning scene, with scripts/arm_padding.py's
    # attached bodies stripped out so RViz stops drawing them. It publishes on
    # /display_planning_scene; point MotionPlanning's "Planning Scene Topic"
    # there (config/moveit.rviz still names monitored_planning_scene, so this is
    # a one-time change in the RViz GUI unless that file is updated too).
    #
    # This changes NOTHING about planning: move_group plans against its own
    # internal scene, not against what it publishes, so the margin stays fully
    # enforced whether or not the padding is drawn. arm_padding.py status reads
    # the real scene and remains the source of truth.
    #
    # The node runs either way; the argument only decides which prefix it hides.
    # Always relaying means /display_planning_scene is a valid scene topic in
    # both states, so turning the filtering off cannot leave RViz staring at a
    # dead topic.
    scene_display_filter = Node(
        package=PACKAGE_NAME,
        executable="scene_display_filter.py",
        name="scene_display_filter",
        output="log",
        arguments=[
            "--prefix",
            "pad__" if hide_padding_in_rviz.perform(context).lower() in ("true", "1")
            else "",
        ],
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
        scene_display_filter,
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
            "hide_padding_in_rviz",
            default_value="true",
            description=(
                "Hide scripts/arm_padding.py's inter-arm margin geometry from RViz. The "
                "padding is attached to the robot, and RViz has no per-attached-object "
                "visibility switch: Robot Alpha hides the whole scene robot with it, and a "
                "PlanningScene object_colors alpha of 0 hides it on the scene robot but NOT "
                "on the orange goal ghost, which renders attached bodies with the default "
                "colour and ignores that map. So instead a relay node republishes the scene "
                "on /display_planning_scene with those objects removed -- set MotionPlanning's "
                "Planning Scene Topic to that. DISPLAY ONLY: the margin is still enforced "
                "either way, because move_group plans against its own internal scene. Set "
                "false to relay the scene unfiltered (the topic stays valid, padding visible)."
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulated clock."
        ),
        DeclareLaunchArgument(
            "enable_walls",
            default_value="true",
            description="Add the workspace walls to the robot model so MoveIt plans around them.",
        ),
        DeclareLaunchArgument(
            "wall_offset",
            default_value="0.7",
            description=(
                "Distance in metres from each arm's base centre to the inner wall face. "
                "The arm's full horizontal reach is 1.007 m. Do not go below ~0.62: the "
                "SRDF 'Home' pose reaches 0.559 m horizontally, and inside that the start "
                "state is in collision and MoveIt refuses to plan from it."
            ),
        ),
        DeclareLaunchArgument(
            "wall_height",
            default_value="1.35",
            description=(
                "Wall height in metres above the plate top face. The arm reaches 1.291 m "
                "above its base, so anything below ~1.30 lets it arc over the top."
            ),
        ),
        DeclareLaunchArgument(
            "wall_thickness", default_value="0.02", description="Wall thickness in metres."
        ),
        DeclareLaunchArgument(
            "base_separation",
            default_value="0.585",
            description="Centre-to-centre distance between the two arm bases, metres. MEASURE THIS.",
        ),
        DeclareLaunchArgument(
            "left_rpy", default_value="0 0 0",
            description="Left arm mounting orientation correction (roll pitch yaw, radians).",
        ),
        DeclareLaunchArgument(
            "right_rpy", default_value="0 0 0",
            description="Right arm mounting orientation correction (roll pitch yaw, radians).",
        ),
        DeclareLaunchArgument(
            "robot_padding",
            default_value="0.10",
            description=(
                "Collision padding in metres applied to every robot link. Both links in a "
                "pair inflate, so the required separation between the two arms is TWICE this "
                "value: 0.020 -> 40 mm. It also inflates links against the workspace walls, "
                "so a pose with little real clearance can become unplannable -- that is the "
                "padding correctly reporting there is no margin, not a fault. "
                "Budget behind the default: 18.2 mm covers the MEASURED worst-case "
                "gripper-tip displacement from trajectory following error (max joint error "
                "11.4 mrad over 49,666 live samples at velocity 0.5); the remainder is an "
                "unmeasured allowance for URDF-vs-bench mounting error, which costs about "
                "1 mm of clearance per mm of base_separation error and ~4.3 mm per degree of "
                "relative yaw. Measure the bench and correct base_separation / left_rpy / "
                "right_rpy to earn that part back. Padding is read once at startup, so "
                "changing it needs a relaunch."
            ),
        ),
    ]
    return LaunchDescription(declared + [OpaqueFunction(function=launch_setup)])
