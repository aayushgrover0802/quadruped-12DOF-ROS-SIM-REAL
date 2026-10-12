"""
leg_kinematics.py -- closed-form (analytical) leg kinematics for the quadruped.

WHY THIS FILE EXISTS
--------------------
The old ik.py used Pinocchio + an iterative (numeric) solver. It was correct,
but:
  * it needs Pinocchio installed (heavy on a Jetson),
  * its answer depends on the "seed" (starting guess),
  * it can silently fail and drop a leg from the command message.

Your robot's legs turn out to be geometrically very clean (we checked this
numerically from the URDF):
  * HipRoll axis   = exactly the robot's -Y axis (Y = forward in this URDF)
  * ThighP / KneeP = exactly the robot's +X axis (X = right side of robot)
So every leg is a classic "roll + 2-link planar" leg, which has an exact
closed-form solution. That is what this file implements, using ONLY numpy.

COORDINATE FRAME (same as your URDF base_link)
----------------------------------------------
  +X = robot's RIGHT side
  +Y = robot's FORWARD direction
  +Z = UP
All positions are in metres, relative to base_link.

HOW THE GEOMETRY IS OBTAINED
----------------------------
We read the URDF XML directly, chain the joint transforms together at q = 0,
and record where each joint axis sits and where the foot is. No hand-typed
link lengths -> if you re-export the URDF from CAD, everything updates.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

LEG_NAMES = ("FL", "FR", "BL", "BR")


# ---------------------------------------------------------------------------
# Small math helpers
# ---------------------------------------------------------------------------
def _rpy_to_matrix(roll, pitch, yaw):
    """URDF convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _axis_angle_matrix(axis, angle):
    """Rotation matrix for `angle` radians about unit vector `axis` (Rodrigues)."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def _make_tf(rotation, translation):
    """Pack a 3x3 rotation + 3-vector translation into a 4x4 transform."""
    tf = np.eye(4)
    tf[:3, :3] = rotation
    tf[:3, 3] = translation
    return tf


def _wrap(angle):
    """Wrap an angle into [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# URDF parsing
# ---------------------------------------------------------------------------
class UrdfModel:
    """Tiny URDF reader: just enough to do forward kinematics on serial chains."""

    def __init__(self, urdf_path):
        # Accept either a file path or the URDF text itself (robot + payload,
        # see description/robot_description.py).
        if urdf_path.lstrip().startswith("<"):
            root = ET.fromstring(urdf_path)
        else:
            root = ET.parse(urdf_path).getroot()
        self.joints = {}          # joint name -> dict
        self.child_to_joint = {}  # child link name -> joint name
        for j in root.findall("joint"):
            origin = j.find("origin")
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
            axis_el = j.find("axis")
            axis = [float(v) for v in axis_el.get("xyz").split()] if axis_el is not None else [1, 0, 0]
            limit_el = j.find("limit")
            info = {
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "origin": _make_tf(_rpy_to_matrix(*rpy), xyz),
                "axis": np.array(axis, dtype=float),
                "lower": float(limit_el.get("lower")) if limit_el is not None and limit_el.get("lower") else -math.pi,
                "upper": float(limit_el.get("upper")) if limit_el is not None and limit_el.get("upper") else math.pi,
            }
            self.joints[info["name"]] = info
            self.child_to_joint[info["child"]] = info["name"]

        # link masses / centre of mass (used to centre the stance under the COM)
        self.inertials = {}
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                continue
            origin = inertial.find("origin")
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()] if origin is not None else [0, 0, 0]
            mass = float(inertial.find("mass").get("value"))
            self.inertials[link.get("name")] = (mass, np.array(xyz))

    def chain_to(self, link_name):
        """List of joint names from the root down to `link_name`."""
        chain = []
        while link_name in self.child_to_joint:
            jname = self.child_to_joint[link_name]
            chain.append(jname)
            link_name = self.joints[jname]["parent"]
        return list(reversed(chain))

    def link_transform(self, link_name, q):
        """4x4 pose of `link_name` in the root frame. q: dict joint name -> angle."""
        tf = np.eye(4)
        for jname in self.chain_to(link_name):
            j = self.joints[jname]
            tf = tf @ j["origin"]
            if j["type"] in ("revolute", "continuous"):
                rot = _axis_angle_matrix(j["axis"], q.get(jname, 0.0))
                tf = tf @ _make_tf(rot, np.zeros(3))
        return tf

    def joint_frame(self, joint_name, q):
        """Pose of a joint's own frame (where its axis sits) in the root frame."""
        j = self.joints[joint_name]
        parent_tf = self.link_transform(j["parent"], q)
        return parent_tf @ j["origin"]

    def center_of_mass(self, q):
        total_m, weighted = 0.0, np.zeros(3)
        for link, (mass, com_local) in self.inertials.items():
            tf = self.link_transform(link, q)
            weighted += mass * (tf[:3, :3] @ com_local + tf[:3, 3])
            total_m += mass
        return weighted / total_m, total_m


# ---------------------------------------------------------------------------
# One leg
# ---------------------------------------------------------------------------
class Leg:
    """
    Closed-form kinematics for ONE leg: HipRoll (about -Y), ThighP and KneeP
    (both about +X). Geometry is extracted from the URDF at q = 0.
    """

    def __init__(self, model, name):
        self.name = name
        self.joint_names = [f"{name}_HipRoll", f"{name}_ThighP", f"{name}_KneeP"]
        self.foot_link = f"{name}_Foot"
        self.model = model

        zero = {}
        hip_tf = model.joint_frame(self.joint_names[0], zero)
        thigh_tf = model.joint_frame(self.joint_names[1], zero)
        knee_tf = model.joint_frame(self.joint_names[2], zero)
        foot_tf = model.link_transform(self.foot_link, zero)

        # Joint axes expressed in base_link, at q = 0.
        roll_axis = hip_tf[:3, :3] @ model.joints[self.joint_names[0]]["axis"]
        pitch_axis = thigh_tf[:3, :3] @ model.joints[self.joint_names[1]]["axis"]
        knee_axis = knee_tf[:3, :3] @ model.joints[self.joint_names[2]]["axis"]

        # Sanity check: this solver assumes the clean axis layout described
        # above. If a future CAD export breaks that, fail loudly instead of
        # sending wrong angles to real motors.
        if abs(abs(roll_axis[1]) - 1.0) > 1e-3:
            raise ValueError(f"{name}: HipRoll axis {roll_axis} is not along Y")
        if abs(abs(pitch_axis[0]) - 1.0) > 1e-3 or abs(abs(knee_axis[0]) - 1.0) > 1e-3:
            raise ValueError(f"{name}: Thigh/Knee axes are not along X")

        # Sign of each axis: +1 if it points along the "expected" direction.
        self.s_roll = 1.0 if roll_axis[1] < 0 else -1.0     # expected -Y
        self.s_thigh = 1.0 if pitch_axis[0] > 0 else -1.0  # expected +X
        self.s_knee = 1.0 if knee_axis[0] > 0 else -1.0

        self.hip = hip_tf[:3, 3].copy()      # a point on the roll axis
        t = thigh_tf[:3, 3] - self.hip       # thigh pivot, relative to hip
        k = knee_tf[:3, 3] - self.hip        # knee pivot, relative to hip
        f = foot_tf[:3, 3] - self.hip        # foot, relative to hip

        # Pitch joints rotate about X, so they never change a point's X.
        # -> the foot's X offset from the hip (in the un-rolled leg) is fixed.
        self.d = f[0]
        # Planar (Y,Z) pieces of the 2-link leg
        self.thigh_yz = np.array([t[1], t[2]])
        v1 = np.array([k[1] - t[1], k[2] - t[2]])
        v2 = np.array([f[1] - k[1], f[2] - k[2]])
        self.L1, self.L2 = float(np.linalg.norm(v1)), float(np.linalg.norm(v2))
        self.a1 = math.atan2(v1[1], v1[0])  # link angle at q=0
        self.a2 = math.atan2(v2[1], v2[0])
        # Which way the knee bends at q=0 (we keep the same "knee branch")
        self.knee_sign = 1.0 if _wrap(self.a2 - self.a1) >= 0 else -1.0

        self.lower = np.array([model.joints[j]["lower"] for j in self.joint_names])
        self.upper = np.array([model.joints[j]["upper"] for j in self.joint_names])
        self.neutral_foot = foot_tf[:3, 3].copy()

    # ------------------------------------------------------------------ FK
    def fk(self, angles):
        """Foot position (base_link frame) for [roll, thigh, knee] angles."""
        roll, th1, th2 = angles
        th1 *= self.s_thigh
        th2 *= self.s_knee
        b1 = th1 + self.a1
        b2 = th1 + th2 + self.a2
        y = self.thigh_yz[0] + self.L1 * math.cos(b1) + self.L2 * math.cos(b2)
        z_r = self.thigh_yz[1] + self.L1 * math.sin(b1) + self.L2 * math.sin(b2)
        phi = roll * self.s_roll
        # roll about -Y by phi == rotate (x, z) counter-clockwise by phi
        x = self.d * math.cos(phi) - z_r * math.sin(phi)
        z = self.d * math.sin(phi) + z_r * math.cos(phi)
        return self.hip + np.array([x, y, z])

    # ------------------------------------------------------------------ IK
    def ik(self, foot_target):
        """
        Joint angles [roll, thigh, knee] that put the foot at `foot_target`
        (base_link frame, metres).

        Returns (angles: np.array(3), reachable: bool).
        If the target is out of reach we return the closest valid pose and
        reachable=False -- we NEVER return nothing, so a leg can't silently
        freeze like it could with the old numeric solver.
        """
        p = np.asarray(foot_target, dtype=float) - self.hip
        reachable = True

        # ---- 1. Hip roll: work in the X-Z plane (perpendicular to roll axis)
        r_sq = p[0] ** 2 + p[2] ** 2
        if r_sq < self.d ** 2 + 1e-9:          # target too close to roll axis
            r_sq = self.d ** 2 + 1e-9
            reachable = False
        z_r = -math.sqrt(r_sq - self.d ** 2)  # foot height in the un-rolled leg (below hip)
        phi = math.atan2(p[2], p[0]) - math.atan2(z_r, self.d)
        roll = _wrap(phi) * self.s_roll

        # ---- 2. Thigh + knee: classic planar 2-link in the Y-Z plane
        dy = p[1] - self.thigh_yz[0]
        dz = z_r - self.thigh_yz[1]
        dist_sq = dy * dy + dz * dz
        cos_g = (dist_sq - self.L1 ** 2 - self.L2 ** 2) / (2.0 * self.L1 * self.L2)
        if cos_g > 1.0 or cos_g < -1.0:
            reachable = False
            cos_g = max(-1.0, min(1.0, cos_g))
        gamma = self.knee_sign * math.acos(cos_g)  # angle between the two links
        b1 = math.atan2(dz, dy) - math.atan2(self.L2 * math.sin(gamma),
                                             self.L1 + self.L2 * math.cos(gamma))
        th1 = _wrap(b1 - self.a1)
        th2 = _wrap(gamma - (self.a2 - self.a1))

        angles = np.array([roll, th1 * self.s_thigh, th2 * self.s_knee])

        # ---- 3. Respect the URDF joint limits
        clipped = np.clip(angles, self.lower, self.upper)
        if np.any(np.abs(clipped - angles) > 1e-9):
            reachable = False
        return clipped, reachable


# ---------------------------------------------------------------------------
# Whole robot
# ---------------------------------------------------------------------------
class QuadKinematics:
    """Holds the 4 legs. This is what the gait controller uses.

    urdf_path may be a file path OR the URDF XML text (e.g. with payload merged in).
    """

    def __init__(self, urdf_path):
        self.model = UrdfModel(urdf_path)
        self.legs = {name: Leg(self.model, name) for name in LEG_NAMES}
        # A fixed, documented joint ordering for every message we publish
        self.joint_names = [j for name in LEG_NAMES for j in self.legs[name].joint_names]

    def solve(self, foot_targets):
        """
        foot_targets: dict leg -> xyz (base frame)
        returns: (dict joint_name -> angle, list of legs that were out of reach)
        """
        out, unreachable = {}, []
        for name, target in foot_targets.items():
            angles, ok = self.legs[name].ik(target)
            if not ok:
                unreachable.append(name)
            for jname, a in zip(self.legs[name].joint_names, angles):
                out[jname] = float(a)
        return out, unreachable

    def foot_positions(self, joint_angles):
        """FK for all legs. joint_angles: dict joint_name -> angle."""
        return {
            name: leg.fk([joint_angles.get(j, 0.0) for j in leg.joint_names])
            for name, leg in self.legs.items()
        }

    def center_of_mass(self, joint_angles):
        return self.model.center_of_mass(joint_angles)
