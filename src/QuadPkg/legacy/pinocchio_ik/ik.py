"""
ik.py -- generic numeric IK, works for any leg regardless of topology
(simple_planar or general). Solves via damped least-squares over the
Jacobian: the standard, robust approach when you don't have (or don't
trust) a closed-form analytical solution.
"""
import pinocchio as pin
import numpy as np
from fk import FKSolver


class IKSolver:
    def __init__(self, descriptor):
        self.descriptor = descriptor
        self.fk = FKSolver(descriptor)
        self.model = self.fk.model
        self.data = self.fk.data

    def solve(self, leg_name, target_xyz, q_init=None, max_iters=100,
              tol=1e-4, damping=1e-6, step_scale=1.0):
        """
        Solve for the joint angles of `leg_name` that place its foot at
        `target_xyz` (meters, relative to base_link -- same frame FK uses).

        Returns: (success: bool, q_full: np.array, final_error_mm: float)
        """
        leg = self.descriptor["legs"][leg_name]
        joint_names = leg["joints"]
        foot_frame_id = self.model.getFrameId(leg["foot_frame"])

        q = self.fk.neutral_q() if q_init is None else q_init.copy()
        target = np.array(target_xyz)

        # Only this leg's own joint indices get updated -- every other
        # joint stays wherever it was, so solving one leg never disturbs
        # the others.
        leg_idx = [self.model.joints[self.model.getJointId(j)].idx_q for j in joint_names]

        current_step_scale = step_scale
        prev_error_norm = float("inf")
        stall_count = 0

        for i in range(max_iters):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            current_pos = self.data.oMf[foot_frame_id].translation
            error = target - current_pos
            error_norm = float(np.linalg.norm(error))

            if error_norm < tol:
                return True, q, float(error_norm * 1000.0)

            # If error isn't improving, we're likely stuck against a joint
            # limit (a fixed step size just re-clamps to the same spot
            # forever, as it did before this fix). Back off the step size
            # so the solver can settle into a smaller, still-useful
            # correction instead of oscillating uselessly at full size.
            if error_norm >= prev_error_norm - 1e-9:
                stall_count += 1
                if stall_count >= 2:
                    current_step_scale *= 0.5
                    stall_count = 0
            else:
                stall_count = 0
            prev_error_norm = error_norm

            # Give up gracefully once the step is too small to matter --
            # this means the target is likely outside the reachable,
            # limit-constrained workspace, not that we need more iterations.
            if current_step_scale < 1e-4:
                return False, q, float(error_norm * 1000.0)

            J_full = pin.computeFrameJacobian(
                self.model, self.data, q, foot_frame_id, pin.LOCAL_WORLD_ALIGNED
            )
            J = J_full[:3, :][:, leg_idx]  # position rows only, this leg's columns only

            # Damped least-squares: robust near singularities (e.g. leg
            # fully extended), unlike a plain pseudo-inverse which blows
            # up there.
            JJt = J @ J.T
            damped_inv = J.T @ np.linalg.inv(JJt + damping * np.eye(3))
            dq = current_step_scale * (damped_inv @ error)

            for k, idx in enumerate(leg_idx):
                q[idx] += dq[k]
                # clamp to this joint's real limits from the descriptor --
                # without this, the solver can happily walk q outside the
                # physically valid range while still "converging"
                limits = leg["joint_limits"][joint_names[k]]
                q[idx] = np.clip(q[idx], limits["lower"], limits["upper"])

        # Ran out of iterations without converging
        final_error_mm = float(np.linalg.norm(error) * 1000.0)
        return False, q, final_error_mm

    def get_leg_angles(self, leg_name, q_full):
        """Convenience: pull out just this leg's angles from a full q vector."""
        leg = self.descriptor["legs"][leg_name]
        leg_idx = [self.model.joints[self.model.getJointId(j)].idx_q for j in leg["joints"]]
        return {leg["joints"][k]: float(q_full[idx]) for k, idx in enumerate(leg_idx)}
