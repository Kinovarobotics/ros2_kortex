#!/usr/bin/env python3
"""Move both Gen3 arms simultaneously between named joint configurations.

Planning uses the `both_arms` group, so MoveIt plans all 14 joints as one
problem and checks the arms against each other along the whole path -- that is
what makes the motion collision-aware rather than two independent arm moves
that happen to run at the same time.

Execution is a single trajectory; MoveIt's simple controller manager splits it
across left_arm_controller and right_arm_controller (disjoint joint sets) and
starts both at once.

Usage:
    ./dual_arm_demo.py                    # home, then home -> demo
    ./dual_arm_demo.py --to demo          # single move to demo
    ./dual_arm_demo.py --plan-only        # preview in RViz, do not execute
    ./dual_arm_demo.py --capture demo     # re-record `demo` from /joint_states
    ./dual_arm_demo.py --velocity 0.2     # slower (default 0.1)
"""

import argparse
import os
import signal
import sys

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions

GROUP = "both_arms"
LEFT_JOINTS = [f"left_joint_{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"right_joint_{i}" for i in range(1, 8)]
ARM_JOINTS = LEFT_JOINTS + RIGHT_JOINTS

DEFAULT_POSES = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "config", "dual_arm_demo_poses.yaml"
)

# moveit_msgs/MoveItErrorCodes -> readable name
# Set by our own SIGINT handler. rclpy's default handler tears the context down
# before we get a chance to cancel, which would leave the arms executing the
# trajectory the controllers already hold -- so we install our own.
ABORT = {"requested": False}


def _on_sigint(_signum, _frame):
    ABORT["requested"] = True


ERROR_NAMES = {
    1: "SUCCESS",
    -1: "FAILURE",
    -2: "PLANNING_FAILED",
    -3: "INVALID_MOTION_PLAN",
    -4: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -5: "CONTROL_FAILED",
    -6: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -7: "TIMED_OUT",
    -8: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -31: "NO_IK_SOLUTION",
}


class DualArmMover(Node):
    def __init__(self):
        super().__init__("dual_arm_demo")
        self._client = ActionClient(self, MoveGroup, "/move_action")
        self._joint_state = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, qos_profile_sensor_data
        )

    def _on_joint_state(self, msg):
        self._joint_state = msg

    def wait_for_joint_state(self, timeout=10.0):
        """Block until a /joint_states message covering all 14 arm joints arrives."""
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            if ABORT["requested"]:
                raise RuntimeError("aborted before motion started")
            rclpy.spin_once(self, timeout_sec=0.1)
            js = self._joint_state
            if js is not None and all(j in js.name for j in ARM_JOINTS):
                return {j: js.position[js.name.index(j)] for j in ARM_JOINTS}
        raise RuntimeError(f"no /joint_states with all arm joints within {timeout:.0f}s")

    def wait_for_server(self, timeout=15.0):
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("/move_action not available -- is move_group running?")

    def _spin_until(self, future, *, handle=None):
        """Spin until `future` completes. On Ctrl-C, cancel the goal and stop.

        Returns True if the future completed normally, False if aborted.
        """
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if ABORT["requested"]:
                self.get_logger().warning("ABORT (Ctrl-C) -- cancelling goal, arms will halt")
                if handle is not None:
                    cancel = handle.cancel_goal_async()
                    deadline = self.get_clock().now().nanoseconds + int(5e9)
                    while (rclpy.ok() and not cancel.done()
                           and self.get_clock().now().nanoseconds < deadline):
                        rclpy.spin_once(self, timeout_sec=0.05)
                    self.get_logger().warning("cancel sent")
                else:
                    self.get_logger().warning("goal not yet accepted; nothing to cancel")
                self.get_logger().warning(
                    "if the arms are still moving, use scripts/dual_arm_stop.py "
                    "or the physical e-stop")
                return False
        return future.done()

    def move_to(self, target, *, plan_only, velocity, acceleration, planning_time, tolerance):
        """Plan (and optionally execute) a single both_arms move to `target`.

        Returns the planned RobotTrajectory, or None on failure.
        """
        request = MotionPlanRequest()
        request.group_name = GROUP
        request.num_planning_attempts = 10
        request.allowed_planning_time = planning_time
        request.max_velocity_scaling_factor = velocity
        request.max_acceleration_scaling_factor = acceleration
        # Plan from wherever the robot is right now.
        request.start_state.is_diff = True

        goal_constraints = Constraints(name="both_arms_joint_goal")
        for joint in ARM_JOINTS:
            goal_constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=joint,
                    position=float(target[joint]),
                    tolerance_above=tolerance,
                    tolerance_below=tolerance,
                    weight=1.0,
                )
            )
        request.goal_constraints.append(goal_constraints)

        options = PlanningOptions()
        options.plan_only = plan_only
        options.planning_scene_diff.is_diff = True
        options.planning_scene_diff.robot_state.is_diff = True

        send = self._client.send_goal_async(
            MoveGroup.Goal(request=request, planning_options=options)
        )
        if not self._spin_until(send):
            return None
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("move_group rejected the goal")
            return None

        get_result = handle.get_result_async()
        if not self._spin_until(get_result, handle=handle):
            return None
        result = get_result.result().result

        code = result.error_code.val
        if code != 1:
            self.get_logger().error(
                f"move_group failed: {ERROR_NAMES.get(code, 'UNKNOWN')} ({code})"
            )
            return None
        return result.planned_trajectory


def describe(trajectory, log):
    """Report the planned trajectory and confirm both arms move together."""
    jt = trajectory.joint_trajectory
    if not jt.points:
        log.warning("  (empty trajectory)")
        return
    duration = jt.points[-1].time_from_start
    secs = duration.sec + duration.nanosec * 1e-9
    log.info(f"  {len(jt.points)} waypoints over {secs:.2f} s, {len(jt.joint_names)} joints")

    # Time window over which each arm is actually in motion.
    def motion_window(joints):
        idx = [jt.joint_names.index(j) for j in joints if j in jt.joint_names]
        if not idx:
            return None
        start = jt.points[0].positions
        first = last = None
        for k, point in enumerate(jt.points):
            if any(abs(point.positions[i] - start[i]) > 1e-4 for i in idx):
                first = k if first is None else first
                last = k
        if first is None:
            return None
        t0 = jt.points[first].time_from_start
        t1 = jt.points[last].time_from_start
        return (t0.sec + t0.nanosec * 1e-9, t1.sec + t1.nanosec * 1e-9)

    left, right = motion_window(LEFT_JOINTS), motion_window(RIGHT_JOINTS)
    for name, window in (("left", left), ("right", right)):
        log.info(f"  {name:5s} arm moves {window[0]:.2f}..{window[1]:.2f} s" if window
                 else f"  {name:5s} arm stationary")
    if left and right:
        overlap = min(left[1], right[1]) - max(left[0], right[0])
        if overlap > 0:
            log.info(f"  -> arms move SIMULTANEOUSLY ({overlap:.2f} s of overlap)")
        else:
            log.warning("  -> arms move sequentially, not simultaneously")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--poses", default=DEFAULT_POSES, help="pose YAML (default: package config)")
    parser.add_argument("--to", dest="targets", action="append", metavar="POSE",
                        help="pose to move to; repeatable. Default: home then demo")
    parser.add_argument("--capture", metavar="POSE",
                        help="record the current joint state into the YAML under this name and exit")
    parser.add_argument("--plan-only", action="store_true", help="plan and visualise, do not execute")
    parser.add_argument("--velocity", type=float, default=0.1, help="velocity scaling (default 0.1)")
    parser.add_argument("--acceleration", type=float, default=0.1, help="acceleration scaling (default 0.1)")
    parser.add_argument("--planning-time", type=float, default=10.0, help="seconds (default 10)")
    parser.add_argument("--tolerance", type=float, default=0.001, help="joint goal tolerance, rad")
    args = parser.parse_args()

    poses_path = os.path.normpath(args.poses)
    with open(poses_path) as handle:
        raw = handle.read()
    poses = yaml.safe_load(raw)
    # Keep the file's leading comment block across a --capture rewrite;
    # yaml.safe_dump would otherwise drop it.
    header = []
    for line in raw.splitlines(keepends=True):
        if line.strip() and not line.lstrip().startswith("#"):
            break
        header.append(line)

    # Take SIGINT ourselves so Ctrl-C cancels the goal instead of killing the
    # client and leaving the controllers running the trajectory.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _on_sigint)
    node = DualArmMover()
    log = node.get_logger()
    try:
        if args.capture:
            current = node.wait_for_joint_state()
            poses[args.capture] = {j: round(current[j], 6) for j in ARM_JOINTS}
            with open(poses_path, "w") as handle:
                handle.writelines(header)
                yaml.safe_dump(poses, handle, default_flow_style=False, sort_keys=False)
            log.info(f"captured '{args.capture}' into {poses_path}")
            for joint in ARM_JOINTS:
                log.info(f"  {joint:16s} {current[joint]:+.6f}")
            return 0

        targets = args.targets or ["home", "demo"]
        unknown = [t for t in targets if t not in poses]
        if unknown:
            log.error(f"unknown pose(s) {unknown}; available: {sorted(poses)}")
            return 2
        missing = {t: [j for j in ARM_JOINTS if j not in poses[t]] for t in targets}
        for name, joints in missing.items():
            if joints:
                log.error(f"pose '{name}' is missing joints: {joints}")
                return 2

        node.wait_for_server()
        node.wait_for_joint_state()  # fail fast if the robot is not publishing

        for step, name in enumerate(targets, 1):
            verb = "Planning" if args.plan_only else "Moving"
            log.info(f"[{step}/{len(targets)}] {verb} '{GROUP}' to '{name}'")
            trajectory = node.move_to(
                poses[name],
                plan_only=args.plan_only,
                velocity=args.velocity,
                acceleration=args.acceleration,
                planning_time=args.planning_time,
                tolerance=args.tolerance,
            )
            if trajectory is None:
                return 130 if ABORT["requested"] else 1
            describe(trajectory, log)
        log.info("done")
        return 0
    except (RuntimeError, KeyboardInterrupt) as exc:
        log.error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
