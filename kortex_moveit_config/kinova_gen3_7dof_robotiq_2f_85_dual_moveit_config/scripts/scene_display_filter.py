#!/usr/bin/env python3
"""Republish the planning scene with the inter-arm padding stripped out, for RViz.

WHY
    scripts/arm_padding.py enforces inter-arm margin by attaching invisible-in-
    principle bodies to one arm. RViz draws them, and there is no per-attached-
    object visibility switch:

      * `Scene Robot -> Robot Alpha` hides the whole scene robot with them.
      * `Attached Body Color` is RGB only; its alpha comes from Robot Alpha.
      * A `PlanningScene.object_colors` entry with alpha 0 does hide them on the
        SCENE robot -- but the Query Start/Goal states are separate
        RobotStateVisualizations that render attached bodies with the DEFAULT
        attached-object colour and never consult that per-object map. So the
        orange goal ghost keeps drawing the cylinders opaque.

    The fix that covers every case is to not tell RViz about them at all.

WHAT IT DOES
    Subscribes to the monitored planning scene, drops every attached collision
    object whose id starts with `pad__` (and its colour entry), and republishes
    on --out. Point RViz's MotionPlanning "Planning Scene Topic" at that topic.

    This is DISPLAY ONLY. move_group plans against its own internal scene, not
    against what it publishes, so the margin stays fully enforced -- the padding
    still blocks goals, it just stops being drawn. Verify any time with
    `arm_padding.py status`, which reads the real scene, not this one.

    Because the objects are filtered out of the diffs too, RViz never learns they
    exist, so they are absent from the scene robot AND from the query start/goal
    states.

USAGE
    ./scene_display_filter.py &
    # then in RViz: MotionPlanning -> Planning Scene Topic -> /display_planning_scene

    ./scene_display_filter.py --prefix pad__ --in /monitored_planning_scene \
                              --out /display_planning_scene
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from moveit_msgs.msg import PlanningScene


class SceneFilter(Node):
    def __init__(self, topic_in, topic_out, prefix):
        super().__init__("scene_display_filter")
        self.prefix = prefix
        self.seen = 0
        self.stripped = 0

        # Match the PlanningSceneMonitor's publisher: RELIABLE / VOLATILE.
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        qos.history = HistoryPolicy.KEEP_LAST

        self.pub = self.create_publisher(PlanningScene, topic_out, qos)
        self.create_subscription(PlanningScene, topic_in, self.on_scene, qos)
        self.create_timer(10.0, self.report)
        what = f"filtering '{prefix}*' out of" if prefix else "relaying (no filtering)"
        self.get_logger().info(
            f"{what} {topic_in} -> {topic_out}; "
            f"point RViz's Planning Scene Topic at {topic_out}")

    def on_scene(self, msg):
        self.seen += 1
        if not self.prefix:
            # Pass-through. An EMPTY prefix must mean "hide nothing", never
            # "hide everything" -- str.startswith("") is True for every id, so
            # the naive filter would strip the entire scene and leave RViz
            # blank. This is the disabled state of the launch argument.
            self.pub.publish(msg)
            return
        before = len(msg.robot_state.attached_collision_objects)
        msg.robot_state.attached_collision_objects = [
            a for a in msg.robot_state.attached_collision_objects
            if not a.object.id.startswith(self.prefix)
        ]
        # Drop the colour entries too, so RViz is not left holding colours for
        # objects it has never heard of.
        msg.object_colors = [c for c in msg.object_colors
                             if not c.id.startswith(self.prefix)]
        # A padded object detached into the world would still be drawn; strip
        # those as well. (arm_padding.py purges them, but a crash mid-remove
        # could leave one behind.)
        msg.world.collision_objects = [o for o in msg.world.collision_objects
                                       if not o.id.startswith(self.prefix)]
        self.stripped += before - len(msg.robot_state.attached_collision_objects)
        self.pub.publish(msg)

    def report(self):
        self.get_logger().info(
            f"{self.seen} scenes relayed, {self.stripped} padding objects stripped")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--in", dest="topic_in", default="/monitored_planning_scene")
    parser.add_argument("--out", dest="topic_out", default="/display_planning_scene")
    parser.add_argument("--prefix", default="pad__",
                        help="object id prefix to hide (default pad__, matching "
                             "arm_padding.py). Empty means relay everything unchanged, so "
                             "the topic stays usable when hiding is switched off.")
    args = parser.parse_args()

    rclpy.init()
    node = SceneFilter(args.topic_in, args.topic_out, args.prefix)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
