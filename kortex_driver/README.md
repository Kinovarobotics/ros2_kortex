# ROS 2 Kortex Driver
The ROS 2 Kortex driver implements the `ros2_control` hardware interface for a `SystemInterface`.

### Command interfaces
This driver exports commands interfaces for position, velocity, and effort interfaces for each joint defined in the URDF.
Additionally, twist interfaces are exported for the end effector for operational space control.
Several additional interfaces are exported, including `set_gripper_max_velocity`, `set_gripper_max_effort` for the gripper joint,
`reset_fault/command`, and `reset_fault/async_success` for fault management.

### State interfaces
This driver exports position and velocity state interfaces for joint defined in the URDF.

Additionally, one state interface `reset_fault/internal_fault` is used for determining the robot's fault state.
