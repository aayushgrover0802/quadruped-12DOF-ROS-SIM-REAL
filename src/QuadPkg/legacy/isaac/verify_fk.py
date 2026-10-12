from omni.isaac.core.prims import XFormPrim
import numpy as np

base = XFormPrim("/World/QuadURDFcad4_0/base_link")
foot = XFormPrim("/World/QuadURDFcad4_0/FR_Knee/FR_Foot")

base_pos, base_rot = base.get_world_pose()
foot_pos, foot_rot = foot.get_world_pose()

print("base_link world position:", base_pos)
print("FR_Foot world position:  ", foot_pos)

# Simple case: if base_link is only translated (not rotated) from origin,
# subtraction alone gives the foot position relative to base_link.
relative_pos = foot_pos - base_pos
print("FR_Foot relative to base_link:", relative_pos)
print("fk.py said:  x=+0.1335  y=+0.1286  z=-0.1332")
