#!/usr/bin/env python3
"""Real collision padding between the two arms -- the thing `robot_padding` cannot do.

WHY robot_padding DOES NOT WORK HERE
    MoveIt pads the robot only for robot-vs-WORLD collision. Self-collision is
    checked against the UNPADDED model (PlanningScene::checkCollision pads for
    the world check, then hands the rest to getCollisionEnvUnpadded()). Both
    arms are links of ONE robot (`dual_gen3`), so arm-vs-arm is self-collision
    and robot_padding contributes exactly zero to it.

    Measured on this cell 2026-09-10: swinging the arms together, first contact
    is at the SAME joint angle with 0.00 m and with 0.40 m of robot_padding,
    while a 5 cm world box 22 cm below the left gripper starts colliding between
    0.15 and 0.20 -- padding working perfectly, for the world.

WHAT THIS DOES
    Self-collision DOES include attached objects; that is how MoveIt stops a
    grasped part from hitting the robot. So this attaches to each link of ONE
    arm a set of spheres that provably contain that link's own collision mesh
    with --margin metres to spare -- each triangle sits inside one sphere, and a
    ball containing a triangle's three vertices contains the triangle.

    One-sided is deliberate and is what makes the number exact. Every arm-vs-arm
    contact pair has one link from each arm, and the left one is always the
    inflated copy, so the guaranteed separation is exactly --margin. Padding
    both arms would give 2x--margin for twice the collision-checking cost.
    (--side both if you want that.)

    Each copy's touch_links names its own arm's links, its sibling copies and
    the static furniture, so it can only ever collide with the OTHER arm.
    Clearance against the walls, the mounting plate and the table is unchanged.

    Unlike a clearance-optimising planner this is a hard constraint, and it
    binds every planner, RViz and /check_state_validity alike.

USAGE
    ./arm_padding.py add --margin 0.01 --verify   # apply 10 mm and measure it
    ./arm_padding.py status
    ./arm_padding.py remove

    --verify reports which named poses survive the margin and the angle at
    which the arms first collide swinging toward each other (bare-mesh baseline
    on this cell: 25 deg; lower means more margin is being enforced).

    Because inflation is uniform, the largest --margin at which a pose is still
    reachable IS that pose's true inter-arm clearance. `add --margin X --verify`
    over a few values measures the cell.

NOTE
    The padding lives in the planning scene: apply it BEFORE launching a motion
    script, and it survives until `remove` or a move_group restart.
"""

import argparse
import math
import os
import re
import struct
import subprocess
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive

PREFIXES = ("left_", "right_")
STATIC_LINKS = ["world", "dual_arm_structure_link", "workspace_walls_link"]
OBJECT_PREFIX = "pad__"
WELD = 1e-6  # metres; STL stores every triangle's vertices separately


# --------------------------------------------------------------------------- #
# mesh handling
# --------------------------------------------------------------------------- #

def resolve_package_uri(uri, cache={}):
    pkg, rel = uri[len("package://"):].split("/", 1)
    if pkg not in cache:
        cache[pkg] = subprocess.run(
            ["ros2", "pkg", "prefix", "--share", pkg],
            capture_output=True, text=True, check=True).stdout.strip()
    return os.path.join(cache[pkg], rel)


def load_stl(path):
    """-> (V, F). Binary STL, with an ASCII fallback.

    The URDF points collision geometry at .dae for the arm links; every one of
    them ships an .STL sibling in the same directory, in metres and in the same
    frame, and that is far cheaper to parse than COLLADA. The gripper links
    already reference .stl directly.
    """
    with open(path, "rb") as handle:
        blob = handle.read()
    count = struct.unpack("<I", blob[80:84])[0] if len(blob) >= 84 else 0
    if len(blob) == 84 + count * 50:
        raw = np.frombuffer(blob[84:], dtype=np.uint8).reshape(count, 50)
        tri = raw[:, 12:48].copy().view("<f4").reshape(count, 3, 3).astype(np.float64)
    else:  # ASCII
        nums = re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", blob)
        if not nums:
            raise RuntimeError(f"{path}: neither binary nor ASCII STL")
        tri = np.array(nums, dtype=np.float64).reshape(-1, 3, 3)
    return weld(tri)


def weld(tri):
    """Merge coincident vertices so normals can be averaged across facets."""
    flat = tri.reshape(-1, 3)
    keys = np.round(flat / WELD).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    inverse = inverse.reshape(-1)
    verts = flat[first]
    faces = inverse.reshape(-1, 3)
    faces = faces[(faces[:, 0] != faces[:, 1])
                  & (faces[:, 1] != faces[:, 2])
                  & (faces[:, 0] != faces[:, 2])]
    return verts, faces


def inflate(verts, faces, margin):
    """Push every vertex out along its area-weighted normal by `margin`.

    Area weighting falls out of the cross product, whose magnitude is twice the
    triangle area, so large facets dominate a vertex's direction -- which is what
    keeps the offset stable on the dense curved sections of these meshes.

    This is a true offset surface, unlike MoveIt's own mesh padding, which moves
    vertices radially away from the mesh centroid: that displaces every vertex by
    `padding` but mostly LENGTHWISE near the ends of a long link, so a slender
    link barely gets fatter where it matters. Here the guarantee is uniform.
    """
    e1 = verts[faces[:, 1]] - verts[faces[:, 0]]
    e2 = verts[faces[:, 2]] - verts[faces[:, 0]]
    face_normals = np.cross(e1, e2)
    normals = np.zeros_like(verts)
    for k in range(3):
        np.add.at(normals, faces[:, k], face_normals)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)

    radial = verts - verts.mean(axis=0)
    # STL winding is not guaranteed consistent; if the averaged normals mostly
    # point back at the centroid the mesh is inside-out and inflating would
    # SHRINK it. Detect and flip rather than silently removing margin.
    if float(np.sum(normals * radial)) < 0.0:
        normals = -normals

    safe = lengths[:, 0] > 1e-12
    direction = np.zeros_like(verts)
    direction[safe] = normals[safe] / lengths[safe]
    # Degenerate vertices (isolated or perfectly cancelling) fall back to radial.
    bad = ~safe
    if np.any(bad):
        rl = np.linalg.norm(radial[bad], axis=1, keepdims=True)
        direction[bad] = np.divide(radial[bad], rl, out=np.zeros_like(radial[bad]),
                                   where=rl > 1e-12)
    return verts + margin * direction


def hull_of(verts):
    from scipy.spatial import ConvexHull
    h = ConvexHull(verts)
    return h.points[h.vertices], np.searchsorted(h.vertices, h.simplices)


def box_cover(verts, faces, margin, count):
    """Cover the link with oriented boxes that provably contain it, plus `margin`.

    Boxes rather than spheres because these links are not round. half_arm_1 is
    92 x 271 x 95 mm -- a roughly SQUARE cross-section -- and the smallest sphere
    around a 92 x 95 mm square bulges about 20 mm past the flats. At a 10 mm
    margin that overshoot is larger than the margin itself, which is why the
    sphere cover blocked the `demo` pose while an inflated hull of the same link
    did not: the looseness, not the margin, was doing the blocking.

    Containment is exact. Triangles are binned along the link's principal axis
    and each slab's box is sized to reach every vertex of every triangle in it;
    a box is convex, so containing three vertices means containing the triangle.
    Growing each box by `margin` on all six faces then guarantees at least
    `margin` of clear space perpendicular to every face.

    The boxes are axis-aligned in the link's PCA frame, which for these arm
    links lines up with the physical shaft, so the fit stays tight.
    """
    centroid = verts.mean(axis=0)
    _, _, vh = np.linalg.svd(verts - centroid, full_matrices=False)
    rot = vh.T                                   # columns = principal axes
    if np.linalg.det(rot) < 0:                   # keep it right-handed
        rot[:, 2] *= -1.0
    quat = matrix_to_quat(rot)

    local = (verts - centroid) @ rot             # link -> PCA frame
    tri = local[faces]
    t = tri.mean(axis=1)[:, 0]
    lo, hi = float(t.min()), float(t.max())
    if hi - lo < 1e-9:
        count = 1
    edges = np.linspace(lo, hi, count + 1)
    slab = np.clip(np.digitize(t, edges[1:-1]), 0, count - 1)

    boxes = []
    for k in range(count):
        take = slab == k
        if not np.any(take):
            continue
        pts = tri[take].reshape(-1, 3)
        low, high = pts.min(axis=0), pts.max(axis=0)
        size = (high - low) + 2.0 * margin
        centre_local = 0.5 * (low + high)
        centre = centroid + rot @ centre_local
        boxes.append((centre, size, quat))
    return boxes


def cylinder_cover(verts, faces, margin, count):
    """Cover the link with coaxial cylinders that provably contain it, plus `margin`.

    Cylinders because the Gen3 links are ROUND in section. A box circumscribing
    a 92 mm round section reaches 65 mm from the axis at its corners against the
    link's true 46 mm -- a 19 mm overshoot, nearly twice a 10 mm margin, and that
    is what made the box cover block the `demo` pose while an inflated hull of
    the same link did not. Raising the slab count could not fix it because the
    error is transverse, not lengthwise. A cylinder has no corners to overshoot
    with, so for these links the fit is essentially the true offset surface.

    Containment is exact, as with the other covers: each triangle is binned
    along the principal axis and its slab's cylinder is grown in radius and
    height to reach every vertex; a cylinder is convex, so it then contains the
    whole triangle. Radius and half-height both get `margin`, which puts at
    least `margin` of clear space around the barrel and beyond the end caps.
    """
    centroid = verts.mean(axis=0)
    _, _, vh = np.linalg.svd(verts - centroid, full_matrices=False)
    axis = vh[0]
    # SolidPrimitive.CYLINDER runs along its own +Z, so the principal axis has
    # to land in the third column of the rotation.
    rot = np.stack([vh[1], vh[2], vh[0]], axis=1)
    if np.linalg.det(rot) < 0:
        rot[:, 0] *= -1.0
    quat = matrix_to_quat(rot)

    rel = verts - centroid
    along = rel @ axis
    radial = np.linalg.norm(rel - np.outer(along, axis), axis=1)

    tri_along = along[faces]
    t = tri_along.mean(axis=1)
    lo, hi = float(t.min()), float(t.max())
    if hi - lo < 1e-9:
        count = 1
    edges = np.linspace(lo, hi, count + 1)
    slab = np.clip(np.digitize(t, edges[1:-1]), 0, count - 1)

    out = []
    for k in range(count):
        take = slab == k
        if not np.any(take):
            continue
        idx = np.unique(faces[take])
        a_lo, a_hi = float(along[idx].min()), float(along[idx].max())
        radius = float(radial[idx].max()) + margin
        height = (a_hi - a_lo) + 2.0 * margin
        centre = centroid + axis * (0.5 * (a_lo + a_hi))
        out.append((centre, height, radius, quat))
    return out


def matrix_to_quat(m):
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        w = math.sqrt(1.0 + trace) * 0.5
        k = 0.25 / w
        return ((m[2, 1] - m[1, 2]) * k, (m[0, 2] - m[2, 0]) * k,
                (m[1, 0] - m[0, 1]) * k, w)
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    j, l = (i + 1) % 3, (i + 2) % 3
    r = math.sqrt(max(1e-12, 1.0 + m[i, i] - m[j, j] - m[l, l]))
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.5 * r
    k = 0.5 / r
    q[j] = (m[j, i] + m[i, j]) * k
    q[l] = (m[l, i] + m[i, l]) * k
    q[3] = (m[l, j] - m[j, l]) * k
    return tuple(q)


def sphere_cover(verts, faces, margin, count):
    """Cover the link with spheres that provably contain it, plus `margin`.

    Mesh-vs-mesh is what killed the first version of this tool: 17 inflated
    hulls made one /check_state_validity call 65 ms instead of ~1 ms, and OMPL
    -- which needs thousands of checks per plan at longest_valid_segment_fraction
    0.002 -- simply could not finish. Sphere-vs-mesh is a far cheaper narrow
    phase, so the padding is expressed as spheres instead.

    Containment is exact, not approximate, and that is what makes the margin a
    guarantee rather than a hope. Each TRIANGLE is assigned to one sphere (by
    where its centroid falls along the link's principal axis) and that sphere is
    then grown to reach all three of its vertices. A ball is convex, so a ball
    containing three vertices contains the whole triangle; every triangle is
    inside its own sphere, so the union contains the entire surface. Adding
    `margin` to every radius then puts at least `margin` of clear space outside
    the real link, everywhere.

    Slabs are cut along the principal axis because these links are slender: a
    single bounding sphere would be enormously loose on a 267 mm forearm.
    """
    centroid = verts.mean(axis=0)
    centred = verts - centroid
    # principal axis = direction of greatest extent
    _, _, vh = np.linalg.svd(centred - centred.mean(axis=0), full_matrices=False)
    axis = vh[0]

    tri_pts = verts[faces]                       # (n, 3, 3)
    tri_mid = tri_pts.mean(axis=1)
    t = (tri_mid - centroid) @ axis
    lo, hi = float(t.min()), float(t.max())
    span = hi - lo
    if span < 1e-9:
        count = 1
    edges = np.linspace(lo, hi, count + 1)
    slab = np.clip(np.digitize(t, edges[1:-1]), 0, count - 1)

    spheres = []
    for k in range(count):
        take = slab == k
        if not np.any(take):
            continue
        pts = tri_pts[take].reshape(-1, 3)
        # centre on the axis, at the middle of what this slab actually spans
        mid = 0.5 * (float(t[take].min()) + float(t[take].max()))
        centre = centroid + axis * mid
        radius = float(np.linalg.norm(pts - centre, axis=1).max()) + margin
        spheres.append((centre, radius))
    return spheres


def to_msg(verts, faces):
    mesh = Mesh()
    mesh.vertices = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in verts]
    mesh.triangles = [MeshTriangle(vertex_indices=[int(i), int(j), int(k)])
                      for i, j, k in faces]
    return mesh


# --------------------------------------------------------------------------- #
# URDF
# --------------------------------------------------------------------------- #

def fetch_urdf(node, timeout=20.0):
    box = []
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE
    node.create_subscription(String, "/robot_description", lambda m: box.append(m.data), qos)
    deadline = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while not box and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not box:
        raise RuntimeError(f"no /robot_description within {timeout:.0f}s")
    return box[0]


def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def collision_meshes(urdf, prefix):
    """-> [(link, mesh path, origin Pose)] for every link of `prefix` that has one."""
    out = []
    for link, body in re.findall(r'<link name="([^"]+)">(.*?)</link>', urdf, re.S):
        if not link.startswith(prefix):
            continue
        block = re.search(r"<collision>(.*?)</collision>", body, re.S)
        if not block:
            continue
        ref = re.search(r'<mesh\s+filename="([^"]+)"', block.group(1))
        if not ref:
            continue  # primitive collision geometry: nothing to inflate here
        path = resolve_package_uri(ref.group(1))
        if path.lower().endswith(".dae"):
            stl = re.sub(r"\.dae$", ".STL", path, flags=re.I)
            if not os.path.exists(stl):
                raise RuntimeError(f"{link}: no STL sibling for {os.path.basename(path)}")
            path = stl
        origin = re.search(r'<origin([^>]*)>', block.group(1))
        pose = Pose()
        pose.orientation.w = 1.0
        if origin:
            xyz = re.search(r'xyz="([^"]+)"', origin.group(1))
            rpy = re.search(r'rpy="([^"]+)"', origin.group(1))
            if xyz:
                x, y, z = (float(v) for v in xyz.group(1).split())
                pose.position.x, pose.position.y, pose.position.z = x, y, z
            if rpy:
                r, p, y = (float(v) for v in rpy.group(1).split())
                (pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w) = rpy_to_quat(r, p, y)
        out.append((link, path, pose))
    return out


# --------------------------------------------------------------------------- #

ARM_JOINTS = ([f"left_joint_{i}" for i in range(1, 8)]
              + [f"right_joint_{i}" for i in range(1, 8)])


def side_of(body):
    name = body[len(OBJECT_PREFIX):] if body.startswith(OBJECT_PREFIX) else body
    for prefix in PREFIXES:
        if name.startswith(prefix):
            return prefix
    return "static"


class ArmPadding(Node):
    def __init__(self):
        super().__init__("arm_padding")
        self.apply = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.valid = self.create_client(GetStateValidity, "/check_state_validity")
        for c in (self.apply, self.get, self.valid):
            if not c.wait_for_service(timeout_sec=15.0):
                raise RuntimeError(f"{c.srv_name} not available -- is move_group running?")

    def call(self, client, request, timeout=60.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        result = future.result()
        if result is None:
            raise RuntimeError(f"{client.srv_name} did not answer within {timeout:.0f}s")
        return result

    def push(self, attached):
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = attached
        request = ApplyPlanningScene.Request()
        request.scene = scene
        return self.call(self.apply, request).success

    def attached(self):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        scene = self.call(self.get, request).scene
        return [a for a in scene.robot_state.attached_collision_objects
                if a.object.id.startswith(OBJECT_PREFIX)]

    def stale_world(self):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        scene = self.call(self.get, request).scene
        return [o.id for o in scene.world.collision_objects
                if o.id.startswith(OBJECT_PREFIX)]

    def purge(self):
        """Delete our objects, attached copies AND world copies.

        Both halves are required. A CollisionObject.REMOVE inside an
        AttachedCollisionObject does not delete anything -- it DETACHES, and
        MoveIt then re-adds the object to the world at the pose the link
        happened to be in. Skipping the second half leaves a full set of
        invisible obstacles frozen exactly where the arm was standing, which
        then blocks every subsequent pose. That is a genuine trap: the object no
        longer appears under ROBOT_STATE_ATTACHED_OBJECTS, so a status check
        that only looks there reports a clean scene while planning is wrecked.
        """
        removed = 0
        current = self.attached()
        if current:
            self.push([AttachedCollisionObject(
                link_name=a.link_name,
                object=CollisionObject(id=a.object.id, operation=CollisionObject.REMOVE))
                for a in current])
            removed = len(current)
        ids = self.stale_world()
        if ids:
            scene = PlanningScene()
            scene.is_diff = True
            scene.world.collision_objects = [
                CollisionObject(id=i, operation=CollisionObject.REMOVE) for i in ids]
            request = ApplyPlanningScene.Request()
            request.scene = scene
            self.call(self.apply, request)
        return removed, len(ids)

    def state_valid(self, positions):
        request = GetStateValidity.Request()
        request.group_name = "both_arms"
        state = RobotState()
        state.joint_state.name = list(positions.keys())
        state.joint_state.position = [float(v) for v in positions.values()]
        state.is_diff = True
        request.robot_state = state
        result = self.call(self.valid, request)
        pairs = sorted({tuple(sorted((c.contact_body_1, c.contact_body_2)))
                        for c in result.contacts})
        return result.valid, pairs


def build(urdf, margin, sides, exact, spheres, shape, log):
    attached = []
    total_in = total_out = 0
    for prefix in sides:
        entries = collision_meshes(urdf, prefix)
        own = [l for l, _, _ in entries]
        siblings = [OBJECT_PREFIX + l for l in own]
        # the arm's OTHER links too -- an inflated forearm overlaps the elbow
        all_links = sorted({l for l in re.findall(r'<link name="([^"]+)"', urdf)
                            if l.startswith(prefix)})
        touch = all_links + siblings + STATIC_LINKS
        for link, path, pose in entries:
            verts, faces = load_stl(path)
            total_in += len(faces)
            aco = AttachedCollisionObject()
            aco.link_name = link
            aco.touch_links = touch
            obj = CollisionObject()
            obj.id = OBJECT_PREFIX + link
            obj.header.frame_id = link
            obj.operation = CollisionObject.ADD
            if exact:
                grown, gfaces = hull_of(inflate(verts, faces, margin))
                total_out += len(gfaces)
                obj.meshes = [to_msg(grown, gfaces)]
                obj.mesh_poses = [pose]
            elif shape == "sphere":
                for centre, radius in sphere_cover(verts, faces, margin, spheres):
                    prim = SolidPrimitive()
                    prim.type = SolidPrimitive.SPHERE
                    prim.dimensions = [float(radius)]
                    # the link's <collision><origin> still has to be applied
                    sp = Pose()
                    sp.position.x = float(centre[0]) + pose.position.x
                    sp.position.y = float(centre[1]) + pose.position.y
                    sp.position.z = float(centre[2]) + pose.position.z
                    sp.orientation.w = 1.0
                    obj.primitives.append(prim)
                    obj.primitive_poses.append(sp)
                total_out += len(obj.primitives)
            elif shape == "cylinder":
                for centre, height, radius, quat in cylinder_cover(
                        verts, faces, margin, spheres):
                    prim = SolidPrimitive()
                    prim.type = SolidPrimitive.CYLINDER
                    prim.dimensions = [float(height), float(radius)]
                    cp = Pose()
                    cp.position.x = float(centre[0]) + pose.position.x
                    cp.position.y = float(centre[1]) + pose.position.y
                    cp.position.z = float(centre[2]) + pose.position.z
                    (cp.orientation.x, cp.orientation.y,
                     cp.orientation.z, cp.orientation.w) = (float(v) for v in quat)
                    obj.primitives.append(prim)
                    obj.primitive_poses.append(cp)
                total_out += len(obj.primitives)
            else:
                for centre, size, quat in box_cover(verts, faces, margin, spheres):
                    prim = SolidPrimitive()
                    prim.type = SolidPrimitive.BOX
                    prim.dimensions = [float(v) for v in size]
                    bp = Pose()
                    bp.position.x = float(centre[0]) + pose.position.x
                    bp.position.y = float(centre[1]) + pose.position.y
                    bp.position.z = float(centre[2]) + pose.position.z
                    (bp.orientation.x, bp.orientation.y,
                     bp.orientation.z, bp.orientation.w) = (float(v) for v in quat)
                    obj.primitives.append(prim)
                    obj.primitive_poses.append(bp)
                total_out += len(obj.primitives)
            aco.object = obj
            attached.append(aco)
    if exact:
        log.info(f"  {len(attached)} inflated hulls, {total_in} -> {total_out} triangles "
                 f"(slow: mesh-vs-mesh)")
    else:
        log.info(f"  {len(attached)} links covered by {total_out} {shape}s "
                 f"(from {total_in} triangles)")
    return attached


def load_poses():
    import yaml
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "config", "dual_arm_demo_poses.yaml")
    with open(os.path.normpath(path)) as handle:
        return yaml.safe_load(handle)


def first_contact_angle(node, poses, step=2):
    for deg in range(0, 91, step):
        js = dict(poses["home"])
        js["left_joint_1"] = math.radians(deg)
        js["right_joint_1"] = math.radians(-deg)
        valid, pairs = node.state_valid(js)
        if not valid:
            return deg, pairs
    return None, []


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=("add", "remove", "status"))
    parser.add_argument("--margin", type=float, default=0.01, metavar="M",
                        help="guaranteed inter-arm separation in metres (default 0.01). "
                             "With --side left this is exactly the clearance enforced; "
                             "with --side both it is doubled.")
    parser.add_argument("--side", choices=("left", "right", "both"), default="left",
                        help="which arm carries the inflated copies (default left). "
                             "One side is enough: every arm-vs-arm pair has one link from "
                             "each arm, so one inflated side gives the full margin at half "
                             "the collision-checking cost.")
    parser.add_argument("--slabs", type=int, default=8, metavar="N",
                        help="primitives per link (default 8). More is a tighter fit along "
                             "slender links and a slightly slower check; fewer is looser, "
                             "which costs reachable poses, never safety.")
    parser.add_argument("--shape", choices=("cylinder", "box", "sphere"),
                        default="cylinder",
                        help="primitive used to cover each link (default cylinder, which "
                             "matches the round section of these links). Boxes overshoot "
                             "~19 mm at their corners and spheres more still, and that "
                             "looseness costs reachable poses without adding safety.")
    parser.add_argument("--exact", action="store_true",
                        help="use inflated convex hulls instead of a sphere cover. Tighter, "
                             "but mesh-vs-mesh made one validity check 65 ms on this cell "
                             "and OMPL could not plan at all. Measure before using it.")
    parser.add_argument("--verify", action="store_true",
                        help="after acting, re-check the named poses and report the angle "
                             "at which the arms first collide")
    args = parser.parse_args()

    rclpy.init()
    node = ArmPadding()
    log = node.get_logger()
    try:
        if args.action == "status":
            current = node.attached()
            if not current:
                log.info("no inter-arm padding attached; arm-vs-arm is bare-mesh contact")
            else:
                prims = sum(len(a.object.primitives) for a in current)
                tris = sum(len(m.triangles) for a in current for m in a.object.meshes)
                what = f"{prims} primitives" if prims else f"{tris} mesh triangles"
                log.info(f"inter-arm padding ACTIVE on {len(current)} links "
                         f"({', '.join(sorted({side_of(a.object.id) for a in current}))}), "
                         f"{what}")
            stale = node.stale_world()
            if stale:
                log.warning(f"{len(stale)} DETACHED copies are loose in the world and are "
                            f"blocking planning; run `remove` to delete them")
            return 0

        if args.action == "remove":
            detached, deleted = node.purge()
            if not detached and not deleted:
                log.info("nothing to remove")
            else:
                log.info(f"removed {detached} attached meshes and deleted {deleted} "
                         f"world copies")
            return 0

        if args.margin <= 0.0:
            log.error("--margin must be > 0")
            return 2
        sides = PREFIXES if args.side == "both" else (args.side + "_",)
        log.info(f"building {args.margin * 1000:.0f} mm padding on the "
                 f"{args.side} arm{'s' if args.side == 'both' else ''}")
        urdf = fetch_urdf(node)
        objects = build(urdf, args.margin, sides, args.exact, args.slabs, args.shape, log)

        # Full purge, not just a detach -- see ArmPadding.purge().
        node.purge()
        node.push(objects)
        enforced = args.margin * (2 if args.side == "both" else 1)
        log.info(f"applied -- arms must now keep {enforced * 1000:.0f} mm apart")

        if args.verify:
            poses = load_poses()
            log.info("--- effect ---")
            for name in poses:
                valid, pairs = node.state_valid(poses[name])
                cross = [f"{a} <-> {b}" for a, b in pairs if side_of(a) != side_of(b)]
                log.info(f"  pose '{name}': {'reachable' if valid else 'BLOCKED'}"
                         + (f" -- {cross[0]}" if cross else ""))
            deg, pairs = first_contact_angle(node, poses)
            if deg is None:
                log.info("  arms never collide over a 0..90 deg swing toward each other")
            else:
                cross = [p for p in pairs if side_of(p[0]) != side_of(p[1])]
                log.info(f"  arms first collide {deg} deg into a swing toward each other "
                         f"({len(cross)} of {len(pairs)} pairs arm-vs-arm)")
            log.info("  bare-mesh baseline on this cell: 25 deg. Lower = margin enforced.")
        return 0
    except (RuntimeError, subprocess.CalledProcessError) as exc:
        log.error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
