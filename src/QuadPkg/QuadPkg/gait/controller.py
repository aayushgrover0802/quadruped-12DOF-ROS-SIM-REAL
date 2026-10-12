"""
controller.py -- the brain of the robot, written WITHOUT any ROS code.

Why no ROS here?  So the exact same logic can be:
  * unit-tested on any laptop with plain `python3`,
  * checked in a physics sim (tools/mujoco_check.py),
  * wrapped by the ROS 2 node (gait/gait.py) for Isaac Sim AND the real robot.

What it does every tick (e.g. 50 times per second):
  1. Smoothly move the body height toward SIT or STAND height.
  2. Smoothly move the walking velocity toward the commanded /cmd_vel.
  3. Advance the trot clock and work out where each foot should be.
  4. Run inverse kinematics -> 12 joint angles.

Frames: robot/base_link frame from the URDF:  +X = right, +Y = forward, +Z = up.
/cmd_vel uses the ROS convention: linear.x = forward, linear.y = LEFT, angular.z = turn left.
We convert between the two in `set_command()`.
"""
import math
from dataclasses import dataclass, field

import numpy as np



@dataclass
class GaitParams:
    # --- body pose -------------------------------------------------------
    stand_height: float = 0.155   # base_link height above the foot points when standing [m]
    sit_height: float = 0.085     # ... when sitting [m]
    stance_y_shift: float = float("nan")  # forward/back foot shift; NaN = auto-centre feet under COM
    stance_x_extra: float = 0.0   # push feet further out sideways (+) or in (-) [m]
    height_transition_time: float = 1.5   # seconds for sit <-> stand
    startup_time: float = 2.0     # seconds to blend from current joints to the start pose
    start_mode: str = "stand"     # pose reached after startup: "stand" (= all joints 0) or "sit"

    # --- trot ------------------------------------------------------------
    gait_period: float = 0.5      # one full step cycle [s]
    duty_factor: float = 0.6      # fraction of the cycle each foot is on the ground (>0.5 = all 4 feet down briefly)
    step_height: float = 0.04     # how high the swinging foot lifts [m]
    max_step_length: float = 0.10 # safety clamp on foot travel per step [m]
    phase_offsets: dict = field(default_factory=lambda: {"FL": 0.0, "BR": 0.0, "FR": 0.5, "BL": 0.5})

    # --- velocity limits -------------------------------------------------
    max_vx: float = 0.25          # forward/back speed limit [m/s]
    max_vy: float = 0.12          # sideways speed limit [m/s]
    max_wz: float = 1.0           # turning speed limit [rad/s]
    max_lin_accel: float = 0.5    # how fast speed may change [m/s^2]
    max_ang_accel: float = 2.0    # [rad/s^2]
    cmd_deadband: float = 0.01    # commands smaller than this count as "stop"


def smoothstep(s):
    """0 -> 1 S-curve with zero slope at both ends (no jerks)."""
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


def _approach(current, target, max_delta):
    """Move `current` toward `target` by at most `max_delta` (rate limiter)."""
    delta = np.clip(target - current, -max_delta, max_delta)
    return current + delta


class GaitController:
    STARTUP, SIT, STAND, WALK = "STARTUP", "SIT", "STAND", "WALK"

    def __init__(self, kinematics, params=None):
        self.kin = kinematics
        self.p = params or GaitParams()

        # Where each foot rests when standing still (x, y only; z comes from height)
        self.nominal_xy = {}
        for name, leg in self.kin.legs.items():
            # +extra pushes the foot outward, -extra pulls it inward (on both sides)
            x = leg.neutral_foot[0] + self.p.stance_x_extra * math.copysign(1.0, leg.neutral_foot[0])
            self.nominal_xy[name] = np.array([x, leg.neutral_foot[1]])
        if math.isnan(self.p.stance_y_shift):
            self.p.stance_y_shift = self._auto_center_shift()
        for name in self.nominal_xy:
            self.nominal_xy[name][1] += self.p.stance_y_shift

        self.sit_angles = self._pose_angles(self.p.sit_height)
        self.stand_angles = self._pose_angles(self.p.stand_height)

        start = str(self.p.start_mode).strip().lower()
        self.mode_request = start if start in ("sit", "stand") else "stand"
        self._start_height = self.p.stand_height if self.mode_request == "stand" else self.p.sit_height
        self._start_pose = self.stand_angles if self.mode_request == "stand" else self.sit_angles

        self.state = self.STARTUP
        self.height = self._start_height
        self._h_from = self._h_to = self._start_height
        self._h_t = 1.0
        self._startup_t = 0.0
        self._start_angles = {j: 0.0 for j in self.kin.joint_names}

        self.target_vel = np.zeros(3)   # (vX, vY, wz) in ROBOT frame
        self.vel = np.zeros(3)          # smoothed version actually used
        self.phase = 0.0
        self.unreachable_count = 0
        self.last_angles = dict(self._start_pose)

    # ------------------------------------------------------------------ setup helpers
    def _foot_targets(self, height, offsets=None):
        targets = {}
        for name, xy in self.nominal_xy.items():
            t = np.array([xy[0], xy[1], -height])
            if offsets is not None:
                t = t + offsets[name]
            targets[name] = t
        return targets

    def _pose_angles(self, height):
        angles, _ = self.kin.solve(self._foot_targets(height))
        return angles

    def _auto_center_shift(self):
        """Shift all feet forward/back so the middle of the 4 feet sits under the COM.
        In a trot only 2 diagonal feet hold the robot; if the COM is not on that
        diagonal line the body tips every step (the 'weird walk' you saw)."""
        shift = 0.0
        for _ in range(3):  # COM moves a little as the legs move -> iterate
            for name, leg in self.kin.legs.items():
                self.nominal_xy[name][1] = leg.neutral_foot[1] + shift
            com, _ = self.kin.center_of_mass(self._pose_angles(self.p.stand_height))
            centre_y = np.mean([xy[1] for xy in self.nominal_xy.values()])
            shift += com[1] - centre_y
        for name, leg in self.kin.legs.items():
            self.nominal_xy[name][1] = leg.neutral_foot[1]
        return shift

    # ------------------------------------------------------------------ inputs
    def reset(self, start_angles=None):
        """(Re)start: blend from `start_angles` (e.g. measured joints) to the start pose
        (stand or sit, see GaitParams.start_mode)."""
        self.state = self.STARTUP
        self._startup_t = 0.0
        if start_angles:
            self._start_angles = {j: float(start_angles.get(j, 0.0)) for j in self.kin.joint_names}

    def set_mode(self, mode):
        mode = mode.strip().lower()
        if mode in ("sit", "stand"):
            self.mode_request = mode

    def set_command(self, vx, vy, wz):
        """ROS /cmd_vel convention in -> robot (URDF) frame out."""
        vx = float(np.clip(vx, -self.p.max_vx, self.p.max_vx))
        vy = float(np.clip(vy, -self.p.max_vy, self.p.max_vy))
        wz = float(np.clip(wz, -self.p.max_wz, self.p.max_wz))
        # ROS forward (+x) = URDF +Y ;  ROS left (+y) = URDF -X
        self.target_vel = np.array([-vy, vx, wz])

    # ------------------------------------------------------------------ main tick
    def update(self, dt):
        """Advance by dt seconds. Returns dict joint_name -> angle [rad]."""
        p = self.p

        # ---- 0. startup blend (joint space) --------------------------------
        if self.state == self.STARTUP:
            self._startup_t += dt
            s = smoothstep(self._startup_t / p.startup_time)
            angles = {j: (1 - s) * self._start_angles[j] + s * self._start_pose[j]
                      for j in self.kin.joint_names}
            if s >= 1.0:
                self.state = self.STAND if self.mode_request == "stand" else self.SIT
                self.height = self._h_from = self._h_to = self._start_height
                self._h_t = 1.0
            self.last_angles = angles
            return angles

        # ---- 1. body height (sit <-> stand), only when not walking ----------
        want_h = p.stand_height if self.mode_request == "stand" else p.sit_height
        if self.state != self.WALK and want_h != self._h_to:
            self._h_from, self._h_to, self._h_t = self.height, want_h, 0.0
        if self._h_t < 1.0:
            self._h_t = min(1.0, self._h_t + dt / p.height_transition_time)
            self.height = self._h_from + (self._h_to - self._h_from) * smoothstep(self._h_t)
        settled = self._h_t >= 1.0
        standing_ready = settled and self.mode_request == "stand" and abs(self.height - p.stand_height) < 1e-6

        # ---- 2. velocity smoothing -------------------------------------------
        goal = self.target_vel if standing_ready else np.zeros(3)
        self.vel[:2] = _approach(self.vel[:2], goal[:2], p.max_lin_accel * dt)
        self.vel[2] = _approach(self.vel[2], goal[2], p.max_ang_accel * dt)
        moving_cmd = np.linalg.norm(goal) > p.cmd_deadband
        moving_now = np.linalg.norm(self.vel) > p.cmd_deadband * 0.5

        # ---- 3. start / stop the trot ------------------------------------------
        if self.state != self.WALK:
            self.state = self.STAND if standing_ready else self.SIT
            if standing_ready and moving_cmd:
                self.state, self.phase = self.WALK, 0.0
        offsets = None
        if self.state == self.WALK:
            self.phase = (self.phase + dt / p.gait_period) % 1.0
            if not moving_cmd and not moving_now and self._all_feet_down(self.phase):
                self.state = self.STAND          # stop only when all 4 feet are on the ground
            else:
                offsets = self._trot_offsets()

        # ---- 4. inverse kinematics --------------------------------------------
        angles, bad = self.kin.solve(self._foot_targets(self.height, offsets))
        self.unreachable_count += len(bad)
        self.last_angles = angles
        return angles

    # ------------------------------------------------------------------ trot maths
    def _all_feet_down(self, phase):
        """True if the gait clock is in a window where every foot is in stance."""
        beta = self.p.duty_factor
        return all(((phase + off) % 1.0) < beta for off in self.p.phase_offsets.values())

    def _trot_offsets(self):
        p = self.p
        vX, vY, wz = self.vel
        t_stance = p.duty_factor * p.gait_period
        offsets = {}
        for name, off in p.phase_offsets.items():
            rx, ry = self.nominal_xy[name]
            # foot velocity relative to ground = body velocity + turning part (w x r)
            step = np.array([vX - wz * ry, vY + wz * rx]) * t_stance
            length = np.linalg.norm(step)
            if length > p.max_step_length:
                step *= p.max_step_length / length

            leg_phase = (self.phase + off) % 1.0
            if leg_phase < p.duty_factor:
                # STANCE: foot on ground, slides backward under the body
                s = leg_phase / p.duty_factor
                xy = step * (0.5 - s)
                z = 0.0
            else:
                # SWING: foot in the air, moves forward on a cycloid curve
                s = (leg_phase - p.duty_factor) / (1.0 - p.duty_factor)
                xy = step * (-0.5 + (s - math.sin(2 * math.pi * s) / (2 * math.pi)))
                z = p.step_height * 0.5 * (1.0 - math.cos(2 * math.pi * s))
            offsets[name] = np.array([xy[0], xy[1], z])
        return offsets
