#!/usr/bin/env python3

# Copyright 2023 PickNik Inc.
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
# Author: Lovro Ivanov

# Description: After a robot has been loaded, this will generate and send a
# joint trajectory message to the robot one time that includes a large jump
# in joint space from the current state. The delta in command sent to the arm can
# be specified by the ros parameter "goal_offset".
# The purpose of this node is to fault the robot so the developer can test clearing faults.

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__("test_huge_joint_space_jump")
        # Declare all parameters

        # name of the controller to publish to
        self.declare_parameter("controller_name", "joint_trajectory_controller")

        # seconds to wait until message is published
        self.declare_parameter("wait_sec_to_publish", 3)

        # how big is the jump in the joint space from the current state
        self.declare_parameter("goal_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # name of the joints that should be included in the command
        self.declare_parameter("joints", rclpy.Parameter.Type.STRING_ARRAY)

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        wait_sec_between_publish = self.get_parameter("wait_sec_to_publish").value
        self.goal_offset = self.get_parameter("goal_offset").value
        self.joints = self.get_parameter("joints").value
        self.joint_state_fbk = JointState()

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        self.joint_state_msg_received = False
        self.command_sent = False

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.get_logger().info(
            'Publishing 1 goal on topic "{}" every {} s'.format(
                publish_topic, wait_sec_between_publish
            )
        )

        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 10
        )

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        if self.joint_state_msg_received and not self.command_sent:

            self.get_logger().info("Publishing command...")
            traj = JointTrajectory()
            traj.joint_names = self.joints
            point = JointTrajectoryPoint()
            for idx, name in enumerate(traj.joint_names):
                if name in self.joint_state_fbk.name:
                    pos = (
                        self.joint_state_fbk.position[self.joint_state_fbk.name.index(name)]
                        + self.goal_offset[idx]
                    )
                    point.positions.append(pos)

            point.velocities = [0.0] * len(point.positions)
            point.time_from_start = Duration(sec=0)

            traj.points.append(point)
            self.publisher_.publish(traj)

            self.command_sent = True

        elif not self.joint_state_msg_received:
            self.get_logger().warn(
                'Start configuration could not be acquired! Check "joint_state" topic!'
            )
        else:
            self.get_logger().info("Command already sent!")

    def joint_state_callback(self, msg):

        if not self.joint_state_msg_received:
            self.joint_state_fbk.header = msg.header
            for i in range(len(msg.name)):
                self.joint_state_fbk.name.append(msg.name[i])
                self.joint_state_fbk.position.append(msg.position[i])
            self.joint_state_msg_received = True
        else:
            return


def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
