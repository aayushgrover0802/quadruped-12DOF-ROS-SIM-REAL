#!/usr/bin/env python3
"""
mujoco_check.py -- quick physics sanity check of the gait WITHOUT Isaac Sim or ROS.

It loads the same URDF into MuJoCo (pip install mujoco), drives the 12 joints
with position servos that roughly match small hobby servo motors, runs the SAME
GaitController the ROS node uses, and prints how the body moved.

Usage (from src/QuadPkg):
    python3 tools/mujoco_check.py            # numbers only
    python3 tools/mujoco_check.py --view     # also opens a 3D viewer window (needs a display)

If this says the robot walks straight and stays upright but Isaac still looks
wrong, the problem is in the Isaac scene settings, not in the controller.
"""
import argparse
import math
import os
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET



HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)
sys.path.insert(0, PKG_ROOT)

import mujoco  # noqa: E402

from QuadPkg.description.robot_description import load_robot_xml  # noqa: E402
from QuadPkg.gait.controller import GaitController, GaitParams  # noqa: E402
from QuadPkg.kinematics.leg_kinematics import QuadKinematics  # noqa: E402

URDF = os.path.join(PKG_ROOT, "urdf", "QuadURDFcad4_0.urdf")
PAYLOAD = os.path.join(PKG_ROOT, "config", "payload.yaml")


def build_model(kp, kd, max_torque, friction, robot_xml):
    tmp = tempfile.mkdtemp()
    shutil.copytree(os.path.join(os.path.dirname(URDF), "meshes"), os.path.join(tmp, "meshes"))
    src = robot_xml.replace(
        'name="QuadURDFcad4_0">',
        'name="QuadURDFcad4_0">\n<mujoco><compiler meshdir="meshes" balanceinertia="true" '
        'discardvisual="true" fusestatic="false" strippath="true"/></mujoco>', 1)
    with open(os.path.join(tmp, "q.urdf"), "w") as f:
        f.write(src)
    m = mujoco.MjModel.from_xml_path(os.path.join(tmp, "q.urdf"))
    mujoco.mj_saveLastXML(os.path.join(tmp, "q.xml"), m)

    root = ET.parse(os.path.join(tmp, "q.xml")).getroot()
    ET.SubElement(root, "option", timestep="0.002", integrator="implicitfast")
    world = root.find("worldbody")
    base = world.find("body")
    base.set("pos", "0 0 0.17")
    ET.SubElement(base, "freejoint", name="root")
    ET.SubElement(world, "geom", name="floor", type="plane", size="5 5 0.1",
                  contype="0", conaffinity="1", friction=f"{friction} 0.005 0.0001")
    ET.SubElement(world, "light", pos="0 0 3", dir="0 0 -1")
    for g in root.iter("geom"):
        if g.get("name") != "floor":
            g.set("contype", "1")
            g.set("conaffinity", "0")
            g.set("friction", f"{friction} 0.005 0.0001")
    act = ET.SubElement(root, "actuator")
    for j in root.iter("joint"):
        j.set("armature", "0.002")   # reflected inertia of a geared servo
        j.set("damping", "0.01")
        ET.SubElement(act, "position", name=j.get("name"), joint=j.get("name"),
                      kp=str(kp), kv=str(kd), forcerange=f"{-max_torque} {max_torque}")
    path = os.path.join(tmp, "quad_scene.xml")
    ET.ElementTree(root).write(path)
    return mujoco.MjModel.from_xml_path(path)


def rpy_from_quat(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--kp", type=float, default=40.0, help="servo stiffness [Nm/rad]")
    ap.add_argument("--kd", type=float, default=0.8, help="servo damping [Nm*s/rad]")
    ap.add_argument("--torque", type=float, default=4.0, help="servo torque limit [Nm]")
    ap.add_argument("--friction", type=float, default=0.9)
    ap.add_argument("--no-payload", action="store_true", help="bare robot, without config/payload.yaml")
    args = ap.parse_args()

    robot_xml = load_robot_xml(URDF, "" if args.no_payload else PAYLOAD)
    model = build_model(args.kp, args.kd, args.torque, args.friction, robot_xml)
    data = mujoco.MjData(model)
    kin = QuadKinematics(robot_xml)
    print(f"total mass in MuJoCo: {sum(model.body_mass):.3f} kg")
    ctrl = GaitController(kin, GaitParams())
    act_index = {model.actuator(i).name: i for i in range(model.nu)}

    # (duration s, mode, (vx, vy, wz), label)
    script = [
        (3.0, "sit", (0, 0, 0), "startup -> sit"),
        (2.5, "stand", (0, 0, 0), "stand up"),
        (5.0, "stand", (0.20, 0, 0), "trot forward 0.20 m/s"),
        (4.0, "stand", (0, 0, 0.8), "turn left 0.8 rad/s"),
        (4.0, "stand", (0, 0.10, 0), "strafe left 0.10 m/s"),
        (4.0, "stand", (-0.15, 0, 0), "trot backward 0.15 m/s"),
        (2.0, "stand", (0, 0, 0), "stop"),
        (2.5, "sit", (0, 0, 0), "sit down"),
    ]
    ctrl_dt, physics_per_ctrl = 0.02, int(round(0.02 / model.opt.timestep))

    viewer = None
    if args.view:
        from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)

    ok = True
    print(f"{'phase':26s} {'state':6s} {'dX(m)':>7s} {'dY(m)':>7s} {'dYaw':>6s} "
          f"{'height':>6s} {'max|roll|':>9s} {'max|pitch|':>10s}")
    for dur, mode, cmd, label in script:
        ctrl.set_mode(mode)
        ctrl.set_command(*cmd)
        start = data.qpos[:3].copy()
        _, _, yaw0 = rpy_from_quat(data.qpos[3:7])
        max_roll = max_pitch = 0.0
        for _ in range(int(dur / ctrl_dt)):
            angles = ctrl.update(ctrl_dt)
            for j, a in angles.items():
                data.ctrl[act_index[j]] = a
            for _ in range(physics_per_ctrl):
                mujoco.mj_step(model, data)
            r, p, _ = rpy_from_quat(data.qpos[3:7])
            max_roll, max_pitch = max(max_roll, abs(r)), max(max_pitch, abs(p))
            if viewer is not None:
                viewer.sync()
        d = data.qpos[:3] - start
        _, _, yaw1 = rpy_from_quat(data.qpos[3:7])
        dyaw = math.degrees((yaw1 - yaw0 + math.pi) % (2 * math.pi) - math.pi)
        # report in the ROBOT's frame at the start of the phase (URDF: +Y forward, +X right)
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        dx_r, dy_r = c * d[0] - s * d[1], s * d[0] + c * d[1]
        print(f"{label:26s} {ctrl.state:6s} {dy_r:+7.3f} {-dx_r:+7.3f} {dyaw:+6.1f} "
              f"{data.qpos[2]:6.3f} {math.degrees(max_roll):8.1f}° {math.degrees(max_pitch):9.1f}°")
        if max(max_roll, max_pitch) > math.radians(30):
            ok = False
    print("\ndX = forward, dY = left (ROS convention), dYaw = turn left in degrees")
    print("RESULT:", "robot stayed upright" if ok else "robot tipped over > 30 deg -- check gains/params")


if __name__ == "__main__":
    main()
