"""

"""
import pinocchio as pin
import numpy as np


class FKSolver:
    def __init__(self, descriptor):
        """
        descriptor: the dict loaded from robot_descriptor.yaml
        """
        self.descriptor = descriptor
        self.model = pin.buildModelFromUrdf(descriptor["urdf_path"])
        self.data = self.model.createData()

    def foot_position(self, leg_name, q_full):
       
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)

        foot_frame_name = self.descriptor["legs"][leg_name]["foot_frame"]
        frame_id = self.model.getFrameId(foot_frame_name)
        return self.data.oMf[frame_id].translation.copy()

    def all_foot_positions(self, q_full):
       
        pin.forwardKinematics(self.model, self.data, q_full)
        pin.updateFramePlacements(self.model, self.data)
        positions = {}
        for leg_name, leg in self.descriptor["legs"].items():
            frame_id = self.model.getFrameId(leg["foot_frame"])
            positions[leg_name] = self.data.oMf[frame_id].translation.copy()
        return positions

    def joint_index(self, joint_name):
       
        jid = self.model.getJointId(joint_name)
        return self.model.joints[jid].idx_q

    def neutral_q(self):
        return pin.neutral(self.model)
