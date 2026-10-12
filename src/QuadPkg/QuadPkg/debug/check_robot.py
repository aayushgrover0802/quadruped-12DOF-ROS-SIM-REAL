"""
check_robot.py -- `ros2 run QuadPkg check_robot [options]`   (no ROS needed, robot can be off)

Checks the robot MODEL (URDF + config/payload.yaml + config/quad_params.yaml) and prints
warnings BEFORE you power anything. Run it every time you add/remove weight or change the URDF.

  1. URDF structure  -- the names / axes / masses / limits the controller depends on
  2. Mass table      -- body, each leg, every payload item, total
  3. Centre of mass  -- where it is vs. the middle of the 4 feet (tipping risk)
  4. Payload sanity  -- items placed outside the body shell
  5. Joint torque    -- estimated holding torque per joint in trot vs. servo stall torque
  6. Stand pose      -- joint 0 really is the stand pose, feet reachable, margin to joint limits

Options
  --urdf PATH           robot URDF        (default: installed urdf/QuadURDFcad4_0.urdf)
  --payload PATH        payload yaml      (default: installed config/payload.yaml, "" = bare robot)
  --params PATH         quad_params.yaml  (default: installed config/quad_params.yaml)
  --stall NM            servo stall torque at your battery voltage (default 2.72 = ST3215 @ 11.1 V)
  --foot-load F         fraction of total weight on ONE foot in trot (default 0.60, see quad_params)
  --add NAME MASS X Y Z try an extra mass WITHOUT editing payload.yaml, e.g.
                        --add lidar 0.17 0 0.08 0.10      (kg, metres in base_link; repeatable)
  --remove NAME         ignore one payload item, e.g. --remove jetson (repeatable)

Exit code 0 = no problems, 1 = warnings, 2 = errors (handy in scripts).
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml

from QuadPkg.description import robot_description as RD
from QuadPkg.gait.controller import GaitController, GaitParams
from QuadPkg.kinematics.leg_kinematics import LEG_NAMES, QuadKinematics

G = 9.81
BODY_SHELL = {"x": (-0.099, 0.099), "y": (-0.170, 0.190), "z": (-0.005, 0.080)}   # base_link, metres


def _defaults():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("QuadPkg")
    except Exception:                                   # running from the source tree
        share = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))
    return (os.path.join(share, "urdf", "QuadURDFcad4_0.urdf"),
            os.path.join(share, "config", "payload.yaml"),
            os.path.join(share, "config", "quad_params.yaml"))


class Report:
    def __init__(self):
        self.n_warn = self.n_err = 0

    def section(self, title):
        print(f"\n=== {title} " + "=" * max(3, 66 - len(title)))

    def ok(self, msg):
        print(f"  [ OK ] {msg}")

    def info(self, msg):
        print(f"         {msg}")

    def warn(self, msg, fix=None):
        self.n_warn += 1
        print(f"  [WARN] {msg}")
        if fix:
            print(f"         -> {fix}")

    def err(self, msg, fix=None):
        self.n_err += 1
        print(f"  [FAIL] {msg}")
        if fix:
            print(f"         -> {fix}")


def load_params(path):
    p = GaitParams()
    if path and os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        rp = data.get("quad_controller", {}).get("ros__parameters", {})
        for k, v in rp.items():
            if hasattr(p, k) and k != "phase_offsets":
                setattr(p, k, v)
    return p


def payload_items(path, add, remove):
    items = RD.load_payload(path) if path else []
    items = [it for it in items if it["name"] not in set(remove)]
    for name, m, x, y, z in add:
        items.append({"name": name, "mass": float(m), "xyz": [float(x), float(y), float(z)],
                      "size": [0.05, 0.05, 0.05]})
    return items


def build_xml(urdf, items):
    import xml.etree.ElementTree as ET
    root = ET.parse(urdf).getroot()
    RD.add_payload(root, items)
    return ET.tostring(root, encoding="unicode")


def check_structure(rep, urdf):
    import xml.etree.ElementTree as ET
    rep.section("1. URDF structure (what the controller needs)")
    root = ET.parse(urdf).getroot()
    links = {lk.get("name"): lk for lk in root.findall("link")}
    joints = {j.get("name"): j for j in root.findall("joint")}
    fatal = False
    if "base_link" not in links:
        rep.err("no link called base_link", "the body link must be named base_link")
        fatal = True
    for leg in LEG_NAMES:
        for suffix in ("_HipRoll", "_ThighP", "_KneeP"):
            j = joints.get(leg + suffix)
            if j is None:
                rep.err(f"joint {leg + suffix} missing", "joint names must be <FL|FR|BL|BR>_<HipRoll|ThighP|KneeP>")
                fatal = True
                continue
            if j.get("type") not in ("revolute", "continuous"):
                rep.err(f"{leg + suffix} is type '{j.get('type')}', must be revolute")
            lim = j.find("limit")
            if lim is None or lim.get("lower") is None:
                rep.warn(f"{leg + suffix} has no <limit lower/upper>", "add limits, otherwise +-180 deg is allowed")
        if leg + "_Foot" not in links:
            rep.err(f"link {leg}_Foot missing", "each leg must end in a link <LEG>_Foot (fixed to the knee link)")
            fatal = True
    total_ok = True
    for name, lk in links.items():
        inert = lk.find("inertial")
        if inert is None or inert.find("mass") is None:
            rep.warn(f"link {name} has no <inertial>/<mass>", "it will count as 0 kg in the COM maths")
            total_ok = False
            continue
        m = float(inert.find("mass").get("value"))
        if m <= 0:
            rep.err(f"link {name} has mass {m}", "every link needs mass > 0 (Isaac/MuJoCo misbehave otherwise)")
            total_ok = False
        ie = inert.find("inertia")
        if ie is not None:
            ixx, iyy, izz = (float(ie.get(k, 0)) for k in ("ixx", "iyy", "izz"))
            if min(ixx, iyy, izz) <= 0:
                rep.err(f"link {name}: inertia has a zero/negative diagonal", "fix ixx/iyy/izz (> 0)")
                total_ok = False
            elif ixx + iyy < izz * 0.999 or ixx + izz < iyy * 0.999 or iyy + izz < ixx * 0.999:
                rep.warn(f"link {name}: inertia breaks the triangle rule (Ixx+Iyy >= Izz ...)",
                         "physics engines may reject or 'balance' it; re-export from CAD or scale it")
    if total_ok:
        rep.ok(f"{len(links)} links, all with mass > 0 and valid inertia")
    if not fatal:
        try:
            QuadKinematics(urdf)
            rep.ok("12 joints found, HipRoll axes along Y, Thigh/Knee axes along X (closed-form IK can be used)")
        except ValueError as e:
            rep.err(f"axis layout not supported by leg_kinematics.py: {e}",
                    "HipRoll must rotate about the body's forward axis (Y), ThighP/KneeP about the side axis (X)")
            fatal = True
    return not fatal


def mass_table(rep, kin, items):
    rep.section("2. Mass table")
    groups = {"body (base_link)": 0.0}
    for leg in LEG_NAMES:
        groups[f"leg {leg}"] = 0.0
    payload = 0.0
    for link, (m, _) in kin.model.inertials.items():
        if link.startswith("payload_"):
            payload += m
        elif link[:2] in LEG_NAMES and link[2] == "_":
            groups[f"leg {link[:2]}"] += m
        else:
            groups["body (base_link)"] = groups.get("body (base_link)", 0.0) + m
    for k, v in groups.items():
        rep.info(f"{k:22s} {v:6.3f} kg")
    for it in items:
        rep.info(f"payload {it['name']:14s} {it['mass']:6.3f} kg  at ({it['xyz'][0]:+.3f}, {it['xyz'][1]:+.3f}, "
                 f"{it['xyz'][2]:+.3f}) m")
    _, total = kin.center_of_mass({})
    rep.info(f"{'TOTAL':22s} {total:6.3f} kg   (payload {payload:.3f} kg)")
    legs = [groups[f"leg {leg}"] for leg in LEG_NAMES]
    if max(legs) - min(legs) > 0.01:
        rep.warn(f"legs differ in mass by {1000 * (max(legs) - min(legs)):.0f} g",
                 "usually a CAD export mistake (e.g. one hip with mass 0) -- compare the <mass> of matching links")
    else:
        rep.ok("all four legs have the same mass (+-10 g)")
    return total


def check_payload(rep, items):
    rep.section("4. Payload placement")
    if not items:
        rep.info("no payload items (bare robot)")
        return
    for it in items:
        x, y, z = it["xyz"]
        inside_xy = BODY_SHELL["x"][0] <= x <= BODY_SHELL["x"][1] and BODY_SHELL["y"][0] <= y <= BODY_SHELL["y"][1]
        if not inside_xy:
            rep.warn(f"'{it['name']}' centre ({x:+.3f}, {y:+.3f}) is outside the body footprint",
                     "check the sign/units of xyz (metres! +X right, +Y forward, +Z up)")
        elif z > BODY_SHELL["z"][1]:
            rep.ok(f"'{it['name']}' sits on top of the body (z = {z:.3f} m) -- raises the COM, that's fine if intended")
        else:
            rep.ok(f"'{it['name']}' is inside the body shell")
        if it["mass"] > 1.5:
            rep.warn(f"'{it['name']}' weighs {it['mass']:.2f} kg -- typo? (mass is in kg, not g)")


def check_com(rep, kin, ctrl):
    rep.section("3. Centre of mass (standing pose)")
    com, _ = kin.center_of_mass(ctrl.stand_angles)
    feet = np.array([kin.legs[n].fk([ctrl.stand_angles[j] for j in kin.legs[n].joint_names]) for n in LEG_NAMES])
    centre = feet.mean(axis=0)
    half_w = (feet[:, 0].max() - feet[:, 0].min()) / 2
    half_l = (feet[:, 1].max() - feet[:, 1].min()) / 2
    dx, dy = (com[0] - centre[0]) * 1000, (com[1] - centre[1]) * 1000
    rep.info(f"COM = ({com[0] * 1000:+.1f}, {com[1] * 1000:+.1f}, {com[2] * 1000:+.1f}) mm in base_link, "
             f"height above feet {(com[2] - centre[2]) * 1000:.0f} mm")
    rep.info(f"feet rectangle: {2 * half_w * 1000:.0f} mm wide x {2 * half_l * 1000:.0f} mm long; "
             f"auto stance shift = {ctrl.p.stance_y_shift * 1000:+.1f} mm")
    if abs(dx) > 5:
        rep.warn(f"COM is {dx:+.1f} mm to the {'right' if dx > 0 else 'left'} of the feet centre",
                 "the trot tips to that side; move a payload item so its x is ~0, or balance it with another")
    else:
        rep.ok(f"COM sideways offset {dx:+.1f} mm (< 5 mm)")
    if abs(dy) > 3:
        rep.warn(f"COM is {dy:+.1f} mm {'ahead of' if dy > 0 else 'behind'} the feet centre after auto-shift",
                 "set stance_y_shift in quad_params.yaml to NaN (auto) or rebuild after editing payload")
    else:
        rep.ok(f"COM fore/aft offset {dy:+.1f} mm after auto-shift")
    if abs(ctrl.p.stance_y_shift) > 0.03:
        rep.warn(f"feet must shift {ctrl.p.stance_y_shift * 1000:+.0f} mm to get under the COM",
                 "the weight is far forward/back; move heavy items towards y = 0")
    # in trot, the COM must stay near the diagonal line between two feet; height makes it worse
    h = com[2] - centre[2]
    if h > 2.2 * half_w:
        rep.warn(f"COM is high ({h * 1000:.0f} mm) compared to the stance width ({2 * half_w * 1000:.0f} mm)",
                 "mount heavy things low, or widen the stance with stance_x_extra > 0")


def joint_torques(rep, kin, ctrl, total, stall, foot_load):
    rep.section("5. Estimated holding torque per joint (trot, one foot carrying "
                f"{foot_load:.0%} of {total:.2f} kg)")
    force = np.array([0.0, 0.0, foot_load * total * G])     # ground pushes the foot UP
    rep.info(f"foot force {force[2]:.1f} N, servo stall torque {stall:.2f} Nm")
    rep.info(f"{'joint':12s} {'torque':>8s} {'% stall':>8s}")
    worst, worst_joint = 0.0, ""
    for leg in LEG_NAMES:
        L = kin.legs[leg]
        foot = kin.model.link_transform(L.foot_link, ctrl.stand_angles)[:3, 3]
        for j in L.joint_names:
            tf = kin.model.joint_frame(j, ctrl.stand_angles)
            axis = tf[:3, :3] @ kin.model.joints[j]["axis"]
            tau = abs(float(np.dot(axis, np.cross(foot - tf[:3, 3], force))))
            pct = tau / stall * 100
            if pct > worst:
                worst, worst_joint = pct, j
            flag = "" if pct < 40 else ("  <-- high" if pct < 70 else "  <-- TOO HIGH")
            rep.info(f"{j:12s} {tau:6.2f} Nm {pct:6.0f} %{flag}")
    if "KneeP" in worst_joint:
        fix = ("remove weight, use a higher battery voltage (more stall torque), or stand a bit taller "
               "(straighter knee = shorter lever; crouching makes it WORSE)")
    elif "HipRoll" in worst_joint:
        fix = "narrow the stance (stance_x_extra < 0, e.g. -0.02) or remove weight"
    else:
        fix = "remove weight or move it closer to the middle of the body"
    if worst >= 70:
        rep.err(f"{worst_joint} needs {worst:.0f}% of stall torque just to hold the body", fix)
    elif worst >= 40:
        rep.warn(f"{worst_joint} needs {worst:.0f}% of stall torque (servos get hot in long trots)",
                 fix + "; walk slowly, watch /quad/servo_temperature. < 40% is comfortable")
    else:
        rep.ok(f"worst joint needs {worst:.0f}% of stall torque (< 40%, comfortable)")
    rep.info("(static estimate; acceleration and foot impacts add more -- tools/mujoco_check.py for dynamics)")


def check_stand(rep, kin, ctrl):
    rep.section("6. Stand pose and joint limits")
    max_dev = max(abs(v) for v in ctrl.stand_angles.values())
    if max_dev > math.radians(3):
        worst = max(ctrl.stand_angles, key=lambda j: abs(ctrl.stand_angles[j]))
        rep.warn(f"stand pose is {math.degrees(max_dev):.1f} deg away from joint zero ({worst})",
                 "joint 0 should be the stand pose (servo middle 2048). If you changed stand_height or the "
                 "payload a lot, re-run tools/rezero_urdf.py and re-set the servo middles")
    else:
        rep.ok(f"stand pose = joint zero within {math.degrees(max_dev):.2f} deg")
    _, bad_sit = kin.solve(ctrl._foot_targets(ctrl.p.sit_height))
    if bad_sit:
        rep.warn(f"sit_height {ctrl.p.sit_height} m is out of reach for legs {bad_sit}", "raise sit_height")
    _, bad_stand = kin.solve(ctrl._foot_targets(ctrl.p.stand_height))
    if bad_stand:
        rep.err(f"stand_height {ctrl.p.stand_height} m is out of reach for {bad_stand}", "lower stand_height")
    if not bad_sit and not bad_stand:
        rep.ok(f"sit ({ctrl.p.sit_height} m) and stand ({ctrl.p.stand_height} m) heights are reachable")
    for leg in LEG_NAMES:
        L = kin.legs[leg]
        for j, lo, hi in zip(L.joint_names, L.lower, L.upper):
            a = ctrl.stand_angles[j]
            margin = math.degrees(min(a - lo, hi - a))
            if margin < 15:
                rep.warn(f"{j} at stand is only {margin:.0f} deg from its URDF limit", "check the URDF <limit>")


def main(argv=None):
    d_urdf, d_payload, d_params = _defaults()
    ap = argparse.ArgumentParser(prog="check_robot", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=d_urdf)
    ap.add_argument("--payload", default=d_payload)
    ap.add_argument("--params", default=d_params)
    ap.add_argument("--stall", type=float, default=2.72)
    ap.add_argument("--foot-load", type=float, default=0.60)
    ap.add_argument("--add", nargs=5, action="append", default=[], metavar=("NAME", "KG", "X", "Y", "Z"))
    ap.add_argument("--remove", action="append", default=[])
    argv = sys.argv[1:] if argv is None else argv
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    a = ap.parse_args(argv)

    rep = Report()
    print(f"URDF    : {a.urdf}\npayload : {a.payload or '(none)'}\nparams  : {a.params}")
    if not check_structure(rep, a.urdf):
        print("\nStructure errors -- fix those first.")
        return 2
    items = payload_items(a.payload, a.add, a.remove)
    kin = QuadKinematics(build_xml(a.urdf, items))
    params = load_params(a.params)
    params.stance_y_shift = float("nan")              # always auto-centre, like the controller
    ctrl = GaitController(kin, params)
    total = mass_table(rep, kin, items)
    check_com(rep, kin, ctrl)
    check_payload(rep, items)
    joint_torques(rep, kin, ctrl, total, a.stall, a.foot_load)
    check_stand(rep, kin, ctrl)
    print(f"\nRESULT: {rep.n_err} error(s), {rep.n_warn} warning(s)")
    return 2 if rep.n_err else (1 if rep.n_warn else 0)


if __name__ == "__main__":
    sys.exit(main())
