"""
setup_isaac_scene.py -- run ONCE inside Isaac Sim to fix the scene, then save.

HOW TO RUN
  1. Open isaac/scenes/NewQuad.usd in Isaac Sim (do NOT press Play yet).
  2. Window -> Script Editor -> File -> Open -> this file -> Run (Ctrl+Enter).
  3. Read the printed report, then File -> Save (Ctrl+S).

WHAT IT CHANGES (and why)
  * Joint drives: stiffness 40 Nm/rad, damping 0.8 Nm*s/rad, torque limit 4 Nm.
    The URDF importer gave ~64 Nm/rad but only 0.026 Nm*s/rad of damping
    (about 4% of what is needed) -> legs bounced and overshot every step.
    Max torque was 100 Nm, which no small servo can do -> sim was unrealistically strong.
  * Physics rate: 200 Hz instead of the default 60 Hz (small, light legs need small steps).
  * Hip masses: BL_Hip had mass 0 and FL_Hip was 0.172 kg from the CAD export;
    both now match the other two hips (0.103 kg).
  * Friction: rubber-like foot/ground material (static 1.0, dynamic 0.9, no bounce).
  * Action Graph: adds /clock publisher (so the controller runs on SIM time, not wall
    time -> gait no longer speeds up when Isaac runs slower than real time) and a
    /joint_states publisher (so the controller starts from the robot's real pose).

If your motors' rated torque is known, change MAX_TORQUE below to match it.
"""
import omni.graph.core as og
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, UsdPhysics, UsdShade

ROBOT = "/World/QuadURDFcad4_0"
ARTICULATION_ROOT = ROBOT + "/base_link"
GRAPH = "/World/ActionGraph"

KP_NM_PER_RAD = 40.0
KD_NM_S_PER_RAD = 0.8
MAX_TORQUE_NM = 4.0
PHYSICS_HZ = 200

DEG = 3.141592653589793 / 180.0   # USD angular drives use DEGREES: (Nm/rad) * (rad/deg)

stage = omni.usd.get_context().get_stage()
report = []

# ---------------------------------------------------------------- 1. physics rate
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.Scene):
        api = PhysxSchema.PhysxSceneAPI.Apply(prim)
        api.CreateTimeStepsPerSecondAttr().Set(PHYSICS_HZ)
        report.append(f"physics scene {prim.GetPath()}: {PHYSICS_HZ} Hz")

# ---------------------------------------------------------------- 2. joint drives
n_joints = 0
for prim in stage.Traverse():
    if not str(prim.GetPath()).startswith(ROBOT) or not prim.IsA(UsdPhysics.RevoluteJoint):
        continue
    drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(KP_NM_PER_RAD * DEG)
    drive.CreateDampingAttr().Set(KD_NM_S_PER_RAD * DEG)
    drive.CreateMaxForceAttr().Set(MAX_TORQUE_NM)
    n_joints += 1
report.append(f"{n_joints} joint drives: Kp={KP_NM_PER_RAD} Nm/rad, Kd={KD_NM_S_PER_RAD} Nm*s/rad, "
              f"max {MAX_TORQUE_NM} Nm")

# ---------------------------------------------------------------- 3. hip mass fix
HIP_FIX = {
    "FL_Hip": Gf.Vec3f(-0.018742, -0.00089079, -0.037245),
    "BL_Hip": Gf.Vec3f(-0.018742, -0.00089079, 0.037245),
}
for name, com in HIP_FIX.items():
    prim = stage.GetPrimAtPath(f"{ROBOT}/{name}")
    if not prim.IsValid():
        report.append(f"WARNING: {name} not found")
        continue
    mass = UsdPhysics.MassAPI.Apply(prim)
    mass.CreateMassAttr().Set(0.10265)
    mass.CreateCenterOfMassAttr().Set(com)
    mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(4.6806e-05, 4.6337e-05, 2.9038e-05))
    mass.CreatePrincipalAxesAttr().Set(Gf.Quatf(1, 0, 0, 0))
    report.append(f"{name}: mass 0.10265 kg")

art = stage.GetPrimAtPath(ARTICULATION_ROOT)
if art.IsValid():
    a = PhysxSchema.PhysxArticulationAPI.Apply(art)
    a.CreateSolverPositionIterationCountAttr().Set(32)
    a.CreateSolverVelocityIterationCountAttr().Set(4)

# ---------------------------------------------------------------- 4. friction
mat_path = Sdf.Path("/World/PhysicsMaterials/rubber")
material = UsdShade.Material.Define(stage, mat_path)
pm = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
pm.CreateStaticFrictionAttr().Set(1.0)
pm.CreateDynamicFrictionAttr().Set(0.9)
pm.CreateRestitutionAttr().Set(0.0)
n_bound = 0
for prim in stage.Traverse():
    p = str(prim.GetPath())
    is_robot_collider = p.startswith(ROBOT) and prim.GetName() == "collisions"
    is_ground = p.startswith("/World/GroundPlane") and prim.HasAPI(UsdPhysics.CollisionAPI)
    if is_robot_collider or is_ground:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, UsdShade.Tokens.strongerThanDescendants, "physics")
        n_bound += 1
report.append(f"friction material bound to {n_bound} colliders")

# ---------------------------------------------------------------- 5. action graph
# Each step is done separately and wrapped in try/except so that ONE failure
# (e.g. a node type renamed in a new Isaac version) is reported clearly instead
# of stopping the whole script. Safe to run as many times as you like.
GRAPH_NODES = {
    # our node name     : candidate type names (newest Isaac first)
    "read_sim_time": ["isaacsim.core.nodes.IsaacReadSimulationTime",
                      "omni.isaac.core_nodes.IsaacReadSimulationTime"],
    "publish_clock": ["isaacsim.ros2.bridge.ROS2PublishClock",
                      "omni.isaac.ros2_bridge.ROS2PublishClock"],
    "publish_joint_state": ["isaacsim.ros2.bridge.ROS2PublishJointState",
                            "omni.isaac.ros2_bridge.ROS2PublishJointState"],
}
GRAPH_LINKS = [
    ("on_playback_tick.outputs:tick", "publish_clock.inputs:execIn"),
    ("read_sim_time.outputs:simulationTime", "publish_clock.inputs:timeStamp"),
    ("on_playback_tick.outputs:tick", "publish_joint_state.inputs:execIn"),
    ("read_sim_time.outputs:simulationTime", "publish_joint_state.inputs:timeStamp"),
]


def _node(name):
    try:
        n = og.Controller.node(f"{GRAPH}/{name}")
        return n if n and n.is_valid() else None
    except Exception:  # noqa: BLE001
        return None


def _attr_names(node):
    return sorted(a.get_name() for a in node.get_attributes())


for name, types in GRAPH_NODES.items():
    node = _node(name)
    # A node whose type failed to load has no "inputs:execIn"/outputs -> delete and recreate it
    if node is not None and not any(a.startswith(("inputs:", "outputs:")) for a in _attr_names(node)):
        og.Controller.edit(GRAPH, {og.Controller.Keys.DELETE_NODES: [name]})
        node = None
    if node is not None:
        report.append(f"graph node '{name}' already exists ({node.get_type_name()})")
        continue
    for t in types:
        try:
            og.Controller.edit(GRAPH, {og.Controller.Keys.CREATE_NODES: [(name, t)]})
            node = _node(name)
            if node is not None and any(a.startswith("inputs:") for a in _attr_names(node)):
                report.append(f"graph node '{name}' created as {t}")
                break
            og.Controller.edit(GRAPH, {og.Controller.Keys.DELETE_NODES: [name]})
        except Exception as exc:  # noqa: BLE001
            report.append(f"could not create {name} as {t}: {exc}")
    else:
        report.append(f"ERROR: no working node type for '{name}' -- is the ROS 2 bridge extension enabled?")

for src, dst in GRAPH_LINKS:
    src_node, dst_node = _node(src.split(".")[0]), _node(dst.split(".")[0])
    if src_node is None or dst_node is None:
        report.append(f"skip link {src} -> {dst} (node missing)")
        continue
    try:
        og.Controller.connect(f"{GRAPH}/{src}", f"{GRAPH}/{dst}")
        report.append(f"linked {src} -> {dst}")
    except Exception as exc:  # noqa: BLE001
        # Already connected counts as success; otherwise show what the node really offers
        dst_attr = dst.split(".", 1)[1]
        src_attr = src.split(".", 1)[1]
        report.append(f"LINK FAILED {src} -> {dst}: {str(exc).splitlines()[0]}")
        report.append(f"   {src.split('.')[0]} attributes: {[a for a in _attr_names(src_node) if 'outputs' in a]}")
        report.append(f"   {dst.split('.')[0]} attributes: {[a for a in _attr_names(dst_node) if 'inputs' in a]}")
        report.append(f"   (looked for {src_attr} -> {dst_attr})")

try:
    og.Controller.edit(GRAPH, {og.Controller.Keys.SET_VALUES: [
        ("ros2_subscribe_joint_state.inputs:topicName", "joint_command")]})
except Exception as exc:  # noqa: BLE001
    report.append(f"could not set subscriber topic: {exc}")

# targetPrim is a relationship: try the OmniGraph way, then plain USD as fallback
if _node("publish_joint_state") is not None:
    try:
        import usdrt
        og.Controller.edit(GRAPH, {og.Controller.Keys.SET_VALUES: [
            ("publish_joint_state.inputs:targetPrim", [usdrt.Sdf.Path(ARTICULATION_ROOT)])]})
        report.append(f"publish_joint_state target = {ARTICULATION_ROOT}")
    except Exception as exc:  # noqa: BLE001
        js = stage.GetPrimAtPath(f"{GRAPH}/publish_joint_state")
        if js.IsValid():
            js.CreateRelationship("inputs:targetPrim", custom=True).SetTargets([Sdf.Path(ARTICULATION_ROOT)])
        report.append(f"publish_joint_state target set via USD fallback ({exc.__class__.__name__})")

print("\n==== setup_isaac_scene.py report ====")
for line in report:
    print("  *", line)
print("Now press Ctrl+S to save the scene.\n")
