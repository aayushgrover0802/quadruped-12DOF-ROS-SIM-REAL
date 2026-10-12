"""
robot_description.py -- ONE place that builds "the robot as it really is":

    URDF (body + legs, zero = standing)  +  config/payload.yaml (battery, Jetson, ...)

Every node calls `load_robot_xml()` and gets the same answer, so the controller,
the hardware bridge and the Isaac export can never disagree about the robot.

Each payload item becomes a small link rigidly attached to base_link with a
fixed joint. A fixed joint adds mass but no motion -- exactly like a battery
held by velcro.
"""
import os
import xml.etree.ElementTree as ET

import yaml


def default_paths():
    """(urdf, payload) inside the installed package share dir."""
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("QuadPkg")
    return (os.path.join(share, "urdf", "QuadURDFcad4_0.urdf"),
            os.path.join(share, "config", "payload.yaml"))


def load_payload(payload_path):
    """Read payload.yaml -> list of dicts (only the enabled items)."""
    if not payload_path:
        return []
    with open(payload_path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    items = []
    for it in data.get("payload") or []:
        if not it.get("enabled", True):
            continue
        name = str(it["name"])
        mass = float(it["mass"])
        xyz = [float(v) for v in it.get("xyz", [0.0, 0.0, 0.0])]
        size = [float(v) for v in it.get("size", [0.05, 0.05, 0.05])]
        if mass <= 0.0 or len(xyz) != 3 or len(size) != 3:
            raise ValueError(f"payload item '{name}': need mass > 0, xyz[3], size[3]")
        items.append({"name": name, "mass": mass, "xyz": xyz, "size": size})
    return items


def box_inertia(mass, size):
    """Solid box: Ixx = m/12 (y^2 + z^2), etc."""
    x, y, z = size
    return (mass / 12.0 * (y * y + z * z),
            mass / 12.0 * (x * x + z * z),
            mass / 12.0 * (x * x + y * y))


def _fmt(v):
    return " ".join(f"{float(a):.6g}" for a in v)


def add_payload(root, items, parent="base_link"):
    """Append payload links + fixed joints to a parsed URDF <robot> element."""
    for it in items:
        link_name = f"payload_{it['name']}"
        link = ET.SubElement(root, "link", name=link_name)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(inertial, "mass", value=f"{it['mass']:.6g}")
        ixx, iyy, izz = box_inertia(it["mass"], it["size"])
        ET.SubElement(inertial, "inertia", ixx=f"{ixx:.6g}", ixy="0", ixz="0",
                      iyy=f"{iyy:.6g}", iyz="0", izz=f"{izz:.6g}")
        visual = ET.SubElement(link, "visual")
        ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
        geom = ET.SubElement(visual, "geometry")
        ET.SubElement(geom, "box", size=_fmt(it["size"]))
        mat = ET.SubElement(visual, "material", name=f"{link_name}_grey")
        ET.SubElement(mat, "color", rgba="0.3 0.3 0.3 1")
        # no <collision>: the payload lives INSIDE the body shell

        joint = ET.SubElement(root, "joint", name=f"{link_name}_fixed", type="fixed")
        ET.SubElement(joint, "origin", xyz=_fmt(it["xyz"]), rpy="0 0 0")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=link_name)
    return root


def load_robot_xml(urdf_path=None, payload_path=None, absolute_meshes=False):
    """URDF text with the payload merged in.

    urdf_path / payload_path = None -> use the installed package files.
    payload_path = ""                -> no payload (bare robot).
    absolute_meshes=True             -> mesh paths become absolute (for Isaac export).
    """
    if urdf_path is None or payload_path is None:
        d_urdf, d_payload = default_paths()
        urdf_path = urdf_path if urdf_path is not None else d_urdf
        payload_path = payload_path if payload_path is not None else d_payload
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    add_payload(root, load_payload(payload_path))
    if absolute_meshes:
        base = os.path.dirname(os.path.abspath(urdf_path))
        for mesh in root.iter("mesh"):
            fn = mesh.get("filename", "")
            if fn and "://" not in fn and not os.path.isabs(fn):
                mesh.set("filename", os.path.normpath(os.path.join(base, fn)))
    if hasattr(ET, "indent"):          # Python >= 3.9 only; just cosmetic
        ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")


def mass_report(kin):
    """Short text: total mass and COM at the zero (= standing) pose."""
    com, total = kin.center_of_mass({})
    mm = com * 1000.0
    return f"total mass {total:.3f} kg, COM at zero pose = ({mm[0]:+.1f}, {mm[1]:+.1f}, {mm[2]:+.1f}) mm"
