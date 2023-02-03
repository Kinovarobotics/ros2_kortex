# kortex2_controllers
Package defining custom controllers created for kinova robots.
- `kortex2_controllers::TwistController`
- `kortex2_controllers::FaultController`


## TwistController
Cartesian twist controller created to use builtin controller on the kinova robot.

### Usage
Example of instance within `ros2_control` can be found in the [configuration file](../kortex2_bringup/config/kortex_controllers.yaml).
Defined interfaces have to exist within the `hardware_interface::SystemInterface` implemented for the robot (e.g. driver).
Stop any commanding controller and then start `TwistController`to use kinova's builtin cartesian twist controller.

#### Note on controllers
Exclusiveness logic is within the driver, and running joint-based and twist controller at the same time is not possible.
Controller Manager (`ros2_control`) will try to activate both on request, but the hardware should reject it.

```
ros2 control switch_controllers --activate streaming_controller --deactivate joint_trajectory_controller
```

By publishing `geometry_msgs::msg::TwistStamped` to `/streaming_controller/commands`, the commands will be piped to the driver
and streamed to the robot controller.

## FaultController
Controller used to reset faults on the kinova arm.

### Usage
Example of instance within `ros2_control` can be found in the [configuration file](../kortex2_bringup/config/kortex_controllers.yaml).
Interfaces are already hardcoded in the `hardware_interface::SystemInterface` implemented for the robot (e.g. driver)

Spawn `FaultController` if it is not already spawned

```
ros2 run controller_manager spawner fault_controller
```

On the topic `/fault_controller/internal_fault` of type `example_interfaces::msg::Bool` the
information if robot is currently faulted can be found.

Deactivate all the controllers using robot.
```
ros2 control swithc_controllers --deactivate joint_trajectory_controller gripper_controller
```

Call the service for resetting the fault (`example_interfaces::srv::Trigger`).

```
ros2 service call /fault_controller/reset_fault example_interfaces/srv/Trigger
```

Activate controllers to enable robot control.
```
ros2 control swithc_controllers --activate joint_trajectory_controller gripper_controller
```
