<!--
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
* -->

# Kortex Description
This package contains the URDF (Unified Robot Description Format), STL and configuration files for the Kortex-compatible robots.

## Usage

To load the description of a robot, you simply have to load the **ARM.xacro** or the **ARM_GRIPPER.xacro** file, with **ARM** being your arm's name (gen3, gen3_lite), and if you have a gripper, **GRIPPER** being your gripper's name (robotiq_2f_85, gen3_lite_2f).

**Arguments**:
- **sim** : If this argument is true, the Gazebo-specific files will be loaded. The default value is **false **.

For example:

- To load the Gen3 description with a Robotiq 2-F 85 gripper for simulation, you would put in your launch file :
<code><param name="robot_description" command="$(find xacro)/xacro --inorder $(find kortex_description)/robots/gen3_robotiq_2f_85.xacro sim:=true"\/></code>

- To load the Gen3 lite description, you would put in your launch file :
<code><param name="robot_description" command="$(find xacro)/xacro --inorder $(find kortex_description)/robots/gen3_lite_gen3_lite_2f.xacro sim:=false"\/></code>

## Tool frame

The `tool_frame` link refers to the tool frame used by the arm when it reports end effector position feedback.

## Xacro Parameters for `load_robot` macro

### Parameter Table
Param | Description | Default |
:---- | :---------- | :------ |
`parent` | Parent link in the URDF the arm should be attached to | - |
`origin` | Origin of the robot relative to the specified parent link | - |
`prefix` | This is an optional prefix for all joint and link names in the kortex_description. It is used to allow differentiating between different arms in the same URDF. | - |
`arm` | Name of your robot arm model. | - |
`gripper` | Name of gripper type | - |
`gripper_joint_name` | Name of gripper joint to be actuated | - |
`dof` | Number of DOFs of your robot. | - |
`vision` | Boolean value to indicate if your arm has a Vision Module. This argument only affects the visual representation of the arm in RViz. | - |
`robot_ip` | The IP address of the robot you're connection to. | - |
`username` | The username for the robot connection. | - |
`password` | The password for the robot connection. | - |
`port` | Port for Kortex hardware driver | - |
`port_realtime` | Realtime port for Kortex hardware driver | - |
`session_inactivity_timeout_ms` | The duration after which the robot will clean the client session if the client hangs up the connection brutally (should not happen with the ROS driver). | - |
`connection_inactivity_timeout_ms` | The duration after which a connection is destroyed by the robot if no communication is detected between the client and the robot. | - |
`use_internal_bus_gripper_comm` | Boolean value to indicate if your gripper will be communicated with through the internal Kinova communication interface. Set to true if the gripper is directly plugged into the kinova arm. Set to false if running in simulation or if gripper is connected to PC via USB. Setting to false will create a ros2_control instance for the gripper. | false |
`use_fake_hardware` | Boolean value to indicate whether or not the hardware components will be mocked. If true the hardware params will be ignored and the hardware components will be mocked. | false |
`fake_sensor_commands` | Boolean value. If set to true will create fake command interfaces for faking sensor measurements with an external command. | false |
`sim_gazebo` | Boolean value to indicate whether or not the gz_ros2_control/GazeboSimSystem plugin will be loaded. | false |
`sim_isaac` | Boolean value to indicate whether or not the topic_based_ros2_control/TopicBasedSystem plugin will be loaded and the "joint_commands_topic" and "joint_states_topic" parameters will be set to the `isaac_joint_commands` and `isaac_joint_states` values respectively. | false |
`isaac_joint_commands` | Name of the joint commands topic to be used by Isaac Sim. | /isaac_joint_commands |
`isaac_joint_states` | Name of the joint states topic to be used by Isaac Sim. | /isaac_joint_states |
`use_external_cable` | Boolean value that sets joint limits to avoid wrapping of external cables if true. | false |
`initial_positions` | Dictionary of initial joint positions. | {joint_1: 0.0, joint_2: 0.0, joint_3: 0.0, joint_4: 0.0, joint_5: 0.0, joint_6: 0.0, joint_7: 0.0} |
`gripper_max_velocity` | Desired velocity in percentage (0.0-100.0%) with which the position will be set. | 100.0 |
`gripper_max_force` | Desired force in percentage (0.0-100.0%) with which the position will be set. NOTE: deprecated according to the [Kortex repo](https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/GripperCyclic/MotorCommand.md). | 100.0 |
`gripper_com_port` | Specifies the USB port that the gripper is plugged in on. This will only be used if `use_internal_bus_gripper_comm` is false. | /dev/ttyUSB0 |

### Example Usage
#### Kinova gen3 with robotiq_2f_85 end effector connected via the internal Kinova communication interface
```
<robot ...>
...
  <xacro:property name="initial_positions" value="${dict(joint_1=0.0, joint_2=0.0, joint_3=-3.14, joint_4=-2.51, joint_5=0.0, joint_6=0.96, joint_7=1.57)}"/>
  <xacro:load_robot
    parent="world"
    arm="gen3"
    gripper="robotiq_2f_85"
    gripper_joint_name="robotiq_85_left_knuckle_joint"
    dof="7"
    vision="true"
    robot_ip="192.168.1.10"
    username="admin"
    password="admin"
    port="10000"
    port_realtime="10001"
    session_inactivity_timeout_ms="60000"
    connection_inactivity_timeout_ms="2000"
    use_internal_bus_gripper_comm="true"
    sim_gazebo="false"
    sim_isaac="false"
    prefix=""
    use_fake_hardware="false"
    initial_positions="${initial_positions}"
    use_external_cable="true"
    fake_sensor_commands="false"
    gripper_com_port="">
    <origin xyz="0 0 0.0" rpy="0 0 0" />
  </xacro:load_robot>
...
</robot>
```
