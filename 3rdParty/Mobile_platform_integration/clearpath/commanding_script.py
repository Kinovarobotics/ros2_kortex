# Copyright (c) 2024 Kinova, Inc.
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
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.action import GripperCommand
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
import time
import math
import subprocess  # To call ignition transport command
from std_msgs.msg import Int32


class KeyboardListener(Node):
    def __init__(self, arm_control, husky_control, gripper_control):
        super().__init__("keyboard_listener")
        self.subscription = self.create_subscription(
            Int32, "/keyboard/keypress", self.keyboard_callback, 10
        )
        self.arm_control = arm_control  # Reference to ArmControl instance
        self.husky_control = husky_control  # Reference to HuskyRobotControl instance
        self.gripper_control = gripper_control  # Reference to GripperControl instance

    def keyboard_callback(self, msg):
        key_code = msg.data
        try:
            key_char = chr(key_code)  # Convert ASCII integer to character
            self.get_logger().info(f"Key pressed: {key_char} (code: {key_code})")

            # Map key characters to specific actions
            if key_char == "H":  # Move the arm to the target HORIZONTAL configuration
                self.arm_control.move_arm_to(
                    [0.229, 1.311, 0.118, 2.413, 0.298, -2.100, -1.520], 5
                )
            elif key_char == "V":  # Move the arm to the target VERTICAL configuration
                self.arm_control.move_arm_to(
                    [
                        0.05565855,
                        0.58595939,
                        0.01642355,
                        1.78789783,
                        0.0384496,
                        0.77841685,
                        -1.53400479,
                    ],
                    5,
                )

            elif key_char == "O":  # OPEN the gripper
                self.gripper_control.command_gripper(position=0.1, max_effort=100.0)
            elif key_char == "C":  # CLOSE the gripper
                self.gripper_control.command_gripper(position=0.5, max_effort=100.0)
            elif key_char == "I":  # Move the arm to the INITIAL configuration
                self.arm_control.move_arm_to(
                    [
                        -0.23921483,
                        0.85088292,
                        0.1007404,
                        2.40223628,
                        -0.07318166,
                        -1.70646077,
                        -1.63699667,
                    ],
                    5,
                )
            elif key_char == "R":  # RESET the robot and grasp box positions
                self.husky_control.reset_poses()
            elif key_char not in ["W", "A", "S", "D"]:
                self.get_logger().info(f"Key {key_char} is not defined for any action.")

        except ValueError:
            # Print a warning message instead of exiting the program
            self.get_logger().error(
                f"Key code {key_code} cannot be converted to a valid character."
            )


class HuskyRobotControl(Node):
    def __init__(self):
        super().__init__("husky_robot_control")
        self.cmd_vel_pub = self.create_publisher(Twist, "/a200_0000/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/a200_0000/platform/odom/filtered", self.odom_callback, 10
        )
        self.current_position = None
        self.target_position = None

    def odom_callback(self, msg):
        """Store the robot's current position."""
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def move_to(self, target_x, target_y, tolerance=0.1):
        """Move the robot to a target (x, y) position."""
        self.target_position = (target_x, target_y)

        # Spin until the robot reaches the target position
        while not self.reached_target(tolerance):
            self.publish_velocity_command()
            rclpy.spin_once(self)

        # Stop the robot once it has reached the target
        self.stop_robot()

    def reached_target(self, tolerance):
        """Check if the robot has reached the target position."""
        if self.current_position is None:
            return False

        current_x, current_y = self.current_position
        target_x, target_y = self.target_position

        # Calculate the Euclidean distance between the current and target positions
        distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

        self.get_logger().info(f"Distance to target: {distance}")

        return distance < tolerance

    def publish_velocity_command(self):
        """Publish velocity commands to move the robot toward the target."""
        if self.current_position is None or self.target_position is None:
            return

        current_x, current_y = self.current_position
        target_x, target_y = self.target_position

        # Calculate the direction to the target
        delta_x = target_x - current_x
        delta_y = target_y - current_y
        angle_to_target = math.atan2(delta_y, delta_x)

        # Command a forward velocity
        msg = Twist()
        msg.linear.x = 1.0  # Forward speed (you can adjust this value)

        # Control the angular velocity to face the target
        msg.angular.z = angle_to_target

        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        """Publish a zero velocity command to stop the robot."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("Target reached, stopping the robot.")

    def reset_poses(self):
        """Reset the robot and the grip box positions in Ignition Gazebo."""
        try:
            # Reset the robot's pose
            subprocess.run(
                [
                    "ign",
                    "service",
                    "-s",
                    "/world/warehouse/set_pose",
                    "--reqtype",
                    "ignition.msgs.Pose",
                    "--reptype",
                    "ignition.msgs.Boolean",
                    "--timeout",
                    "5000",
                    "--req",
                    "position: {x: 0.000925, y: 0.000067, z: 0.132280}, "
                    "orientation: {x: 0, y: 0, z: 0.000007, w: 1}, "
                    'name: "a200_0000/robot"',
                ]
            )
            print("Robot pose reset successfully.")

            # Reset the grip box's pose
            subprocess.run(
                [
                    "ign",
                    "service",
                    "-s",
                    "/world/warehouse/set_pose",
                    "--reqtype",
                    "ignition.msgs.Pose",
                    "--reptype",
                    "ignition.msgs.Boolean",
                    "--timeout",
                    "5000",
                    "--req",
                    "position: {x: 3.0, y: 0.0, z: 0.53}, "
                    "orientation: {x: 0, y: 0, z: 0, w: 1}, "
                    'name: "grip_box"',
                ]
            )
            print("Grip box pose reset successfully.")

        except Exception as e:
            print(f"Failed to reset poses: {e}")


class ArmControl(Node):
    def __init__(self):
        super().__init__("arm_control")
        self.arm_pub = self.create_publisher(
            JointTrajectory, "/a200_0000/arm_0_joint_trajectory_controller/joint_trajectory", 10
        )
        # Subscribe to the state topic to monitor the arm trajectory
        self.arm_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            "/a200_0000/arm_0_joint_trajectory_controller/state",
            self.arm_state_callback,
            10,
        )
        self.current_state = None

    def arm_state_callback(self, msg):
        """Store the latest state of the arm."""
        self.current_state = msg

    def move_arm_to(self, positions, duration):
        """Send a command to move the arm to specified positions and wait for completion."""
        traj = JointTrajectory()
        traj.joint_names = [
            "arm_0_joint_1",
            "arm_0_joint_2",
            "arm_0_joint_3",
            "arm_0_joint_4",
            "arm_0_joint_5",
            "arm_0_joint_6",
            "arm_0_joint_7",
        ]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration

        traj.points.append(point)
        self.arm_pub.publish(traj)

        # Wait for the arm to complete the movement (optional)
        # self.wait_for_trajectory_completion(positions)

    def wait_for_trajectory_completion(self, positions):
        """Block execution until the arm reaches the target trajectory."""
        self.get_logger().info("Waiting for arm to reach target trajectory...")
        tolerance = 0.02  # Define how close the joints need to be to the goal

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.current_state is not None:
                # Check if the actual positions are close enough to the desired positions
                actual_positions = self.current_state.actual.positions

                if all(abs(a - d) < tolerance for a, d in zip(actual_positions, positions)):
                    self.get_logger().info("Target trajectory reached.")
                    break

            time.sleep(0.1)  # Add some delay to avoid CPU overuse

        time.sleep(2)


class GripperControl(Node):
    def __init__(self):
        super().__init__("gripper_control")
        self.gripper_client = ActionClient(
            self, GripperCommand, "/a200_0000/robotiq_85_controller/gripper_cmd"
        )

    def command_gripper(self, position, max_effort):
        """Send a command to control the gripper position and effort."""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.gripper_client.wait_for_server()
        self.gripper_client.send_goal_async(goal_msg)
        time.sleep(4)


def main():
    rclpy.init()

    # Initialize control nodes
    husky_control = HuskyRobotControl()
    arm_control = ArmControl()
    gripper_control = GripperControl()
    keyboard_listener = KeyboardListener(arm_control, husky_control, gripper_control)

    try:
        # Gripper will open and close for showcasing
        gripper_control.command_gripper(position=0.1, max_effort=100.0)
        gripper_control.command_gripper(position=0.5, max_effort=100.0)
        # Loop the code until Ctrl+C is pressed
        while rclpy.ok():
            # Step 1: Trigger arm movement by keypress event or any other logic
            rclpy.spin_once(keyboard_listener, timeout_sec=1.0)

            # You can also include any other periodic checks or actions here
            # The rest of the robotic arm control and gripper logic
            # will be triggered when certain keys are pressed

    except KeyboardInterrupt:
        print("User interrupted the program (Ctrl+C)")

    finally:
        # Cleanup and shutdown nodes properly
        husky_control.destroy_node()
        arm_control.destroy_node()
        gripper_control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
