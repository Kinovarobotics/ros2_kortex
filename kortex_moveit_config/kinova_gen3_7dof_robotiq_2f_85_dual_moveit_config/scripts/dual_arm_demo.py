#!/usr/bin/env python3
"""Move both Gen3 arms simultaneously between named joint configurations.

Planning uses the `both_arms` group, so MoveIt plans all 14 joints as one
problem and checks the arms against each other along the whole path -- that is
what makes the motion collision-aware rather than two independent arm moves
that happen to run at the same time.

Execution is a single trajectory; MoveIt's simple controller manager splits it
across left_arm_controller and right_arm_controller (disjoint joint sets) and
starts both at once.

By default the script CYCLES FOREVER between the listed poses (home -> demo ->
home -> demo -> ...) until Ctrl-C. Ctrl-C cancels the in-flight goal before
exiting, so the arms halt and hold rather than running the trajectory the
controllers already hold. Use --cycles to bound the run.

Usage:
    ./dual_arm_demo.py                    # loop home <-> demo forever (Ctrl-C to stop)
    ./dual_arm_demo.py --cycles 1         # one pass: home, then demo
    ./dual_arm_demo.py --cycles 5 --dwell 1.0
    ./dual_arm_demo.py --to demo --cycles 1   # single move to demo
    ./dual_arm_demo.py --plan-only        # preview in RViz, do not execute
    ./dual_arm_demo.py --capture demo     # re-record `demo` from /joint_states
    ./dual_arm_demo.py --velocity 0.2     # faster (default 0.1)
"""

import argparse
import itertools
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
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    PlanningOptions,
    RobotState,
)

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


# Built by reflection from moveit_msgs so it can never drift from the installed
# MoveIt again. The previous hand-written table used MoveIt 1 numbering, where
# FAILURE=-1 and PLANNING_FAILED=-2. Jazzy shifts that whole block by one
# (PLANNING_FAILED=-1, INVALID_MOTION_PLAN=-2, ... PREEMPTED=-7, FAILURE=99999),
# so every code from -1 to -8 was reported under the wrong name.
ERROR_NAMES = {
    value: name
    for name, value in vars(MoveItErrorCodes).items()
    if name.isupper() and isinstance(value, int) and not name.startswith("_")
}


# Failures where asking for another plan is a sensible response: the planner is
# randomised, so a different attempt may well succeed. Everything else -- an
# invalid group, a goal in collision, a start state that is already invalid,
# PREEMPTED from our own Ctrl-C -- is deterministic or intentional, and retrying
# only burns the planning timeout again.
RETRYABLE = {
    MoveItErrorCodes.PLANNING_FAILED,                              # -1
    MoveItErrorCodes.INVALID_MOTION_PLAN,                          # -2
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE,  # -3
    MoveItErrorCodes.TIMED_OUT,                                    # -6
    MoveItErrorCodes.FAILURE,                                      # 99999
}


class DualArmMover(Node):
    def __init__(self):
        super().__init__("dual_arm_demo")
        self._client = ActionClient(self, MoveGroup, "/move_action")
        self._validity = self.create_client(GetStateValidity, "/check_state_validity")
        self._joint_state = None
        self.last_error_code = None
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
                    "if the arms are still moving, hit the physical e-stop "
                    "(see scripts/README-stop.md)")
                return False
        return future.done()

    def state_validity(self, joints, timeout=5.0):
        """Ask move_group whether a joint configuration is collision-free.

        Returns (valid, [description strings]). (None, []) if the service did not
        answer -- the caller must not read that as "safe".
        """
        if not self._validity.service_is_ready():
            if not self._validity.wait_for_service(timeout_sec=timeout):
                return None, []
        request = GetStateValidity.Request()
        request.group_name = GROUP
        state = RobotState()
        state.joint_state.name = list(joints.keys())
        state.joint_state.position = [float(v) for v in joints.values()]
        state.is_diff = True
        request.robot_state = state
        future = self._validity.call_async(request)
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while (rclpy.ok() and not future.done()
               and self.get_clock().now().nanoseconds < deadline):
            rclpy.spin_once(self, timeout_sec=0.02)
        result = future.result()
        if result is None:
            return None, []
        contacts = [
            f"{c.contact_body_1} <-> {c.contact_body_2} ({c.depth * 1000:.2f} mm)"
            for c in result.contacts
        ]
        return result.valid, contacts

    def verify_arrival(self, name, target, tolerance):
        """After an executed move, confirm the arms are where the plan said.

        The trajectory controllers report SUCCESS from the action server; that is
        not evidence the arms tracked the path. This re-reads /joint_states and
        checks both that the pose was reached and that the reached state is
        collision-free. Returns True if all good.
        """
        try:
            reached = self.wait_for_joint_state(timeout=5.0)
        except RuntimeError as exc:
            self.get_logger().error(f"  cannot verify arrival: {exc}")
            return False
        worst_joint, worst = None, 0.0
        for joint in ARM_JOINTS:
            error = abs(reached[joint] - float(target[joint]))
            if error > worst:
                worst_joint, worst = joint, error
        ok = True
        if worst > tolerance:
            self.get_logger().error(
                f"  did NOT reach '{name}': {worst_joint} off by {worst:.4f} rad "
                f"(tolerance {tolerance:.4f})"
            )
            ok = False
        valid, contacts = self.state_validity(reached)
        if valid is None:
            self.get_logger().warning("  /check_state_validity did not answer; not verified")
        elif not valid:
            self.get_logger().error(f"  reached state is IN COLLISION after '{name}':")
            for line in contacts:
                self.get_logger().error(f"      {line}")
            ok = False
        return ok

    def dwell(self, seconds):
        """Pause between moves, staying responsive to Ctrl-C. False if aborted."""
        if seconds <= 0.0:
            return not ABORT["requested"]
        deadline = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            if ABORT["requested"]:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return not ABORT["requested"]

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
        self.last_error_code = code
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"move_group failed: {ERROR_NAMES.get(code, 'UNKNOWN')} ({code})"
            )
            return None
        return result.planned_trajectory

    def move_to_with_retries(self, target, *, retries, **kwargs):
        """move_to, re-planning on failures that a fresh plan can plausibly fix.

        OMPL is randomised, so a rejected plan is usually just an unlucky path
        rather than an impossible problem: on this cell the ValidateSolution
        response adapter rejects roughly 1 plan in 8-13 because a single
        waypoint grazes the other arm by a fraction of a millimetre. Asking for
        another plan is the correct response to that.

        Note this is NOT what `num_planning_attempts` does: that makes OMPL
        solve the problem N times inside ONE request and return the shortest
        solution, after which the pipeline validates that one solution once. A
        rejection there fails the whole request, so the retry has to live out
        here.

        Structural failures (bad group, goal in collision, start state invalid)
        are returned immediately -- replanning cannot fix them and retrying ten
        times would just cost 10x the planning timeout.
        """
        for attempt in range(1, retries + 1):
            trajectory = self.move_to(target, **kwargs)
            if trajectory is not None:
                if attempt > 1:
                    self.get_logger().info(f"  succeeded on plan attempt {attempt}/{retries}")
                return trajectory
            if ABORT["requested"]:
                return None
            code = self.last_error_code
            if code not in RETRYABLE:
                self.get_logger().error(
                    f"  {ERROR_NAMES.get(code, 'UNKNOWN')} is not fixable by replanning; giving up"
                )
                return None
            if attempt < retries:
                self.get_logger().warning(f"  replanning ({attempt}/{retries} used)")
        self.get_logger().error(f"  no valid plan after {retries} attempts")
        return None


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
    parser.add_argument("--arrival-tolerance", type=float, default=0.05, metavar="RAD",
                        help="max per-joint error accepted after an executed move (default 0.05)")
    parser.add_argument("--plan-retries", type=int, default=10, metavar="N",
                        help="re-plan up to N times when a plan is rejected (default 10). "
                             "Distinct from num_planning_attempts, which is internal to one request.")
    parser.add_argument("--cycles", type=int, default=0, metavar="N",
                        help="number of passes over the pose list; 0 (default) = loop forever")
    parser.add_argument("--dwell", type=float, default=0.0, metavar="SECONDS",
                        help="pause at each pose before moving on (default 0)")
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

        if args.cycles < 0:
            log.error("--cycles must be >= 0 (0 means loop forever)")
            return 2
        if args.plan_retries < 1:
            log.error("--plan-retries must be >= 1 (1 means no retry)")
            return 2

        node.wait_for_server()
        node.wait_for_joint_state()  # fail fast if the robot is not publishing

        # Pre-flight: a target that is already in collision (commonly because
        # robot_padding was raised and the pose has no real clearance) would
        # otherwise fail on the first plan with GOAL_IN_COLLISION, after the arms
        # had already started moving. Check every pose up front instead.
        for name in dict.fromkeys(targets):
            valid, contacts = node.state_validity(poses[name])
            if valid is None:
                log.warning(f"could not verify pose '{name}' (no /check_state_validity)")
            elif not valid:
                log.error(f"pose '{name}' is IN COLLISION -- refusing to start:")
                for line in contacts:
                    log.error(f"    {line}")
                log.error("re-capture the pose, or lower robot_padding")
                return 2
            else:
                log.info(f"pose '{name}' verified collision-free")

        forever = args.cycles == 0
        if forever:
            log.warning(
                f"LOOPING FOREVER over {' -> '.join(targets)} at velocity scaling "
                f"{args.velocity}. Ctrl-C to stop (cancels the in-flight goal). "
                "For anything worse, use the physical e-stop -- see scripts/README-stop.md."
            )
        else:
            log.info(f"{args.cycles} cycle(s) over {' -> '.join(targets)}")

        completed = 0
        cycles = itertools.count(1) if forever else range(1, args.cycles + 1)
        try:
            for cycle in cycles:
                total = "inf" if forever else str(args.cycles)
                for step, name in enumerate(targets, 1):
                    verb = "Planning" if args.plan_only else "Moving"
                    log.info(
                        f"[cycle {cycle}/{total}] [{step}/{len(targets)}] "
                        f"{verb} '{GROUP}' to '{name}'"
                    )
                    trajectory = node.move_to_with_retries(
                        poses[name],
                        retries=args.plan_retries,
                        plan_only=args.plan_only,
                        velocity=args.velocity,
                        acceleration=args.acceleration,
                        planning_time=args.planning_time,
                        tolerance=args.tolerance,
                    )
                    if trajectory is None:
                        # Stop on the first failure rather than retrying blindly:
                        # a plan that failed once on real hardware usually means the
                        # scene or the arm state is not what the script assumes.
                        log.error(f"stopping after {completed} completed cycle(s)")
                        return 130 if ABORT["requested"] else 1
                    describe(trajectory, log)
                    if not args.plan_only and not node.verify_arrival(
                        name, poses[name], args.arrival_tolerance
                    ):
                        log.error(
                            "  execution did not match the plan -- stopping before commanding "
                            "another motion from an unverified state"
                        )
                        log.error(f"stopping after {completed} completed cycle(s)")
                        return 1
                    if not node.dwell(args.dwell):
                        log.warning(f"ABORT -- stopping after {completed} completed cycle(s)")
                        return 130
                completed = cycle
        finally:
            if forever:
                log.info(f"completed {completed} full cycle(s)")
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
