"""
parser.py -- run this ONCE per new robot.

Reads a URDF, auto-detects each leg's kinematic chain (base_link -> ... -> foot),
extracts link lengths / joint limits / topology, and writes robot_descriptor.yaml.

Usage:
    python3 parser.py new_robot.urdf ../results/robot_descriptor.yaml
"""
import sys
import os
import numpy as np
import pinocchio as pin
import yaml


def find_leg_chains(model):
    """
    Walk the URDF's frame tree and find every chain from base_link down to a
    terminal 'Foot' frame. Returns {leg_name: [joint_name, joint_name, ...]}
    in parent-to-child order.
    """
    legs = {}
    for frame in model.frames:
        if frame.type == pin.BODY and frame.name.endswith("_Foot"):
            leg_name = frame.name.split("_")[0]  # "FR", "FL", "BR", "BL"

            # Walk up the joint tree from this frame's parent joint to the root,
            # collecting revolute joint names along the way.
            joint_chain = []
            joint_id = frame.parentJoint
            while joint_id != 0:  # 0 == "universe", the root
                jname = model.names[joint_id]
                if model.joints[joint_id].nq > 0:  # skip fixed joints (nq==0)
                    joint_chain.append(jname)
                joint_id = model.parents[joint_id]

            joint_chain.reverse()  # now base-to-foot order
            legs[leg_name] = {
                "joints": joint_chain,
                "foot_frame": frame.name,
            }
    return legs


def compute_leg_link_lengths_mm(model, joint_names, foot_frame_name):
    """
    Returns clearly-labeled leg segment lengths, not just raw joint offsets:
      hip_offset_mm : body center -> HipRoll axis (not a leg "link" per se,
                      just where the hip is mounted on the body)
      L1_thigh_mm   : ThighP axis -> KneeP axis (the "shoulder to knee" length)
      L2_shank_mm   : KneeP axis -> actual foot contact point

    L2 needed special handling: the foot is attached via a FIXED joint
    (type="fixed" in the URDF), which has zero DOF and was previously being
    skipped entirely by the revolute-only joint walk. Pinocchio still tracks
    fixed joints as FRAMES though -- the foot frame's `.placement` (relative
    to its parentJoint, which is the last revolute joint, KneeP) gives
    exactly the knee-to-foot offset we need.
    """
    result = {}

    # hip_offset and L1_thigh: same as before, straight joint-to-joint offsets
    hip_joint, thigh_joint, knee_joint = joint_names  # e.g. [HipRoll, ThighP, KneeP]
    result["hip_offset_mm"] = float(np.linalg.norm(
        model.jointPlacements[model.getJointId(hip_joint)].translation) * 1000.0)
    result["L1_thigh_mm"] = float(np.linalg.norm(
        model.jointPlacements[model.getJointId(knee_joint)].translation) * 1000.0)

    # L2_shank: knee joint -> foot frame, via the fixed joint's frame placement
    foot_frame_id = model.getFrameId(foot_frame_name)
    foot_frame = model.frames[foot_frame_id]
    result["L2_shank_mm"] = float(np.linalg.norm(foot_frame.placement.translation) * 1000.0)

    return result


def get_joint_limits(model, joint_names):
    limits = {}
    for jname in joint_names:
        jid = model.getJointId(jname)
        idx_q = model.joints[jid].idx_q
        limits[jname] = {
            "lower": float(model.lowerPositionLimit[idx_q]),
            "upper": float(model.upperPositionLimit[idx_q]),
            "effort": float(model.effortLimit[idx_q]),
            "velocity": float(model.velocityLimit[idx_q]),
        }
    return limits


def get_urdf_axes(urdf_path, joint_names):
    """
    Pinocchio represents URDF axis="0 0 1" as a specialized JointModelRZ type
    that does NOT expose a .axis attribute the same way arbitrary-axis joints
    (JointModelRevoluteUnaligned) do -- reading straight from the URDF XML is
    simpler and robust across all joint types.
    """
    import xml.etree.ElementTree as ET
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    axes = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name in joint_names:
            axis_el = joint.find("axis")
            xyz = axis_el.get("xyz") if axis_el is not None else "1 0 0"
            axes[name] = np.array([float(v) for v in xyz.split()])
    return axes


def classify_topology(urdf_path, joint_names, tol_deg=5.0):
    """
    'simple_planar' if every revolute joint in this leg rotates about
    (approximately) the same axis -- i.e. it behaves like a 2D planar leg,
    which your fast analytical IK can handle.
    'general' otherwise -- needs the numeric IK fallback (e.g. this robot's
    HipRoll axis is [0,0,1] while ThighP/KneeP rotate about a near-X axis --
    genuinely not planar, correctly flagged below).
    """
    axes_by_name = get_urdf_axes(urdf_path, joint_names)
    axes = [axes_by_name[j] for j in joint_names]
    if len(axes) < 2:
        return "simple_planar"
    ref = axes[0]
    for a in axes[1:]:
        cos_angle = np.clip(np.dot(ref, a) / (np.linalg.norm(ref) * np.linalg.norm(a)), -1, 1)
        angle_deg = np.degrees(np.arccos(abs(cos_angle)))  # abs() -- anti-parallel axes are still "same line"
        if angle_deg > tol_deg:
            return "general"
    return "simple_planar"


def main():
    if len(sys.argv) != 3:
        print("Usage: python3 parser.py <urdf_path> <output_yaml_path>")
        sys.exit(1)

    urdf_path = os.path.abspath(sys.argv[1])
    output_path = sys.argv[2]

    print(f"Loading URDF: {urdf_path}")
    model = pin.buildModelFromUrdf(urdf_path)
    print(f"  Robot name: {model.name}")
    print(f"  DOF: {model.nq}")

    legs = find_leg_chains(model)
    print(f"  Detected {len(legs)} legs: {list(legs.keys())}")

    for leg_name, leg in legs.items():
        leg["link_lengths_mm"] = compute_leg_link_lengths_mm(model, leg["joints"], leg["foot_frame"])
        leg["joint_limits"] = get_joint_limits(model, leg["joints"])
        leg["topology"] = classify_topology(urdf_path, leg["joints"])
        print(f"  {leg_name}: joints={leg['joints']}  topology={leg['topology']}")

    descriptor = {
        "robot_name": model.name,
        "urdf_path": urdf_path,
        "legs": legs,
    }

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        yaml.dump(descriptor, f, default_flow_style=False, sort_keys=False)

    print(f"\nWrote descriptor to: {output_path}")


if __name__ == "__main__":
    main()
