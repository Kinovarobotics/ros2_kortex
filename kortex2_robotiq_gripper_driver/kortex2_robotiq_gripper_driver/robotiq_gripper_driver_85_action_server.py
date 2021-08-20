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
# Author: Bren Pierce

import threading
import time
import numpy as np
from math import fabs

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from control_msgs.action import GripperCommand
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class Robotiq85ActionServer(Node):
    def __init__(self):
        super().__init__('robotiq_gripper_driver_85_action_server')

        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('position_tolerance', 0.005)

        self._timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self._position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value

        self.create_subscription(JointState, "/joint_states", self._update_gripper_stat, 10)
        self._gripper_pub = self.create_publisher(Float64MultiArray, '/hand_controller/commands', 10)

        self._gripper_position = None

        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/hand_controller/gripper_cmd',
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            execute_callback=self._execute_callback,
            callback_group=ReentrantCallbackGroup())


        self.get_logger().info('Gripper server ready')


    def get_time(self):
        time_msg = self.get_clock().now().to_msg()
        return float(time_msg.sec) + (float(time_msg.nanosec) * 1e-9)


    def shutdown(self):
        self.get_logger().info("Shutdown gripper")
        self._gripper.shutdown()


    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


    def _goal_callback(self, goal_request):
        self.get_logger().info('Gripper received goal request')
        return GoalResponse.ACCEPT


    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Gripper received cancel request')
        return CancelResponse.ACCEPT


    def _execute_callback(self, goal_handle):
        self.get_logger().info('Gripper executing goal...')

        # Send goal to gripper
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [goal_handle.request.command.position]
        self.get_logger().info("Got goal position: " + str(goal_handle.request.command.position))
        self._gripper_pub.publish(cmd_msg)

        # Feedback msg to the client
        feedback_msg = GripperCommand.Feedback()
        feedback_msg.reached_goal = False

        # update at 100Hz
        rate = self.create_rate(100)
        start_time = self.get_time()

        while rclpy.ok():
            dt = self.get_time() - start_time
            # print("cb:", dt)

            if not (dt < self._timeout):
                self.get_logger().warn('Gripper timeout reached')
                break


            if self._gripper_position is None:
                self.get_logger().warn("No gripper feedback yet")
            else:
                feedback_msg.position = self._gripper_position
                # Position tolerance achieved or object grasped
                if (fabs(goal_handle.request.command.position - feedback_msg.position) < self._position_tolerance):
                    feedback_msg.reached_goal = True
                    self.get_logger().info('Goal achieved: %r'% feedback_msg.reached_goal)

                goal_handle.publish_feedback(feedback_msg)

            # Check to see if we have reached the goal.s
            if feedback_msg.reached_goal:
                self.get_logger().debug('Reached goal, exiting loop')
                break;

            rate.sleep()

        # Copy the feedback current state to the result msg.
        result_msg = GripperCommand.Result()
        result_msg.reached_goal = feedback_msg.reached_goal
        result_msg.stalled = feedback_msg.stalled
        result_msg.position = feedback_msg.position
        result_msg.effort = feedback_msg.effort

        if result_msg.reached_goal:
            self.get_logger().info('Setting action to succeeded')
            goal_handle.succeed()
        else:
            self.get_logger().info('Setting action to abort')
            goal_handle.abort()

        return result_msg


    def _update_gripper_stat(self, data):
        self._gripper_position = None
        # Check to make sure that the joint is the gripper finger
        if str(data.name[2]) == "finger_joint":
            self._gripper_position = data.position[2]


def main(args=None):
    rclpy.init(args=args)

    action_server = Robotiq85ActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)

    action_server.shutdown()
    action_server.destroy()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
