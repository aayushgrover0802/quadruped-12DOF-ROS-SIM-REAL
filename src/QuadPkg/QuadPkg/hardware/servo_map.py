"""
servo_map.py -- converts between URDF joint angles (radians) and raw servo ticks.

    ticks = 2048 + offset_ticks + direction * gear_ratio * angle / RAD_PER_TICK
    angle = (ticks - 2048 - offset_ticks) * RAD_PER_TICK / (direction * gear_ratio)

  * 2048           : the servo's middle position. You set it (web app) with the robot
                     STANDING, and the URDF zero is also STANDING -> they line up.
  * offset_ticks   : small correction if the middle position was set a bit off
                     (servo_tool calib measures it for you). 1 tick = 0.088 degrees.
  * direction      : +1 or -1. Mirrored servos (left vs right side) usually turn the
                     "wrong" way -> -1. servo_tool jog tells you which.
  * gear_ratio     : 1.0 unless there is a gearbox/belt between servo and joint.

No ROS in here -> used by the bridge node, servo_tool and the tests.
"""
import math

import numpy as np
import yaml

from QuadPkg.hardware.protocol import CENTER_TICK, TICKS_PER_REV

RAD_PER_TICK = 2.0 * math.pi / TICKS_PER_REV

DEFAULT_ORDER = [
    "FL_HipRoll", "FL_ThighP", "FL_KneeP",
    "FR_HipRoll", "FR_ThighP", "FR_KneeP",
    "BL_HipRoll", "BL_ThighP", "BL_KneeP",
    "BR_HipRoll", "BR_ThighP", "BR_KneeP",
]
# Your wiring: IDs 1..12 = FL knee, thigh, hip, FR knee, thigh, hip, BL ..., BR ...
DEFAULT_IDS = [3, 2, 1, 6, 5, 4, 9, 8, 7, 12, 11, 10]


class ServoMap:
    def __init__(self, joint_order, servo_id, direction, offset_ticks, gear_ratio,
                 tick_min=0, tick_max=TICKS_PER_REV - 1):
        n = len(joint_order)
        for name, v in (("servo_id", servo_id), ("direction", direction),
                        ("offset_ticks", offset_ticks), ("gear_ratio", gear_ratio)):
            if len(v) != n:
                raise ValueError(f"{name} has {len(v)} entries, joint_order has {n}")
        if len(set(servo_id)) != n:
            raise ValueError(f"servo_id has duplicates: {list(servo_id)}")
        for d in direction:
            if d not in (1, -1, 1.0, -1.0):
                raise ValueError(f"direction must be +1 or -1, got {d}")
        self.order = list(joint_order)
        self.ids = [int(i) for i in servo_id]
        self.direction = np.array(direction, dtype=float)
        self.offset = np.array(offset_ticks, dtype=float)
        self.gear = np.array(gear_ratio, dtype=float)
        self.tick_min, self.tick_max = int(tick_min), int(tick_max)

    @property
    def scale(self):
        return self.direction * self.gear / RAD_PER_TICK      # ticks per radian, with sign

    def joint_to_ticks(self, q):
        t = CENTER_TICK + self.offset + self.scale * np.asarray(q, dtype=float)
        return np.clip(np.round(t), self.tick_min, self.tick_max).astype(int)

    def ticks_to_joint(self, ticks):
        return (np.asarray(ticks, dtype=float) - CENTER_TICK - self.offset) / self.scale

    def speed_to_joint(self, ticks_per_s):
        return np.asarray(ticks_per_s, dtype=float) / self.scale

    def index(self, joint):
        return self.order.index(joint)

    @classmethod
    def from_params(cls, p):
        """p: dict with the hardware_bridge parameters (from YAML or ROS)."""
        return cls(p.get("joint_order", DEFAULT_ORDER), p.get("servo_id", DEFAULT_IDS),
                   p.get("direction", [1.0] * 12), p.get("offset_ticks", [0] * 12),
                   p.get("gear_ratio", [1.0] * 12), p.get("tick_min", 0),
                   p.get("tick_max", TICKS_PER_REV - 1))


def load_hardware_yaml(path):
    """Read config/hardware.yaml -> plain dict of hardware_bridge parameters."""
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data["hardware_bridge"]["ros__parameters"]
