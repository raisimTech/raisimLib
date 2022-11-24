# Lint as: python3
"""A torque based stance controller framework."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from typing import Any, Sequence, Tuple

import numpy as np
# import time

from raisimGymTorch.mpc.mpc_controller_DAgger_all import gait_generator as gait_generator_lib
from raisimGymTorch.mpc.mpc_controller_DAgger_all import leg_controller
from raisimGymTorch.mpc.mpc_controller_DAgger_all import qp_torque_optimizer

from raisimGymTorch.mpc.mpc_controller_DAgger_all import a1
_FORCE_DIMENSION = 3
KP = np.array((0., 0., 100., 100., 100., 0.))
KD = np.array((40., 30., 10., 10., 10., 30.))
MAX_DDQ = np.array((10., 10., 10., 20., 20., 20.))
MIN_DDQ = -MAX_DDQ


class TorqueStanceLegController(leg_controller.LegController):
  def __init__(
      self,
      gait_generator: Any,
      state_estimator: Any,
      desired_speed: Tuple[float, float] = (0, 0),
      desired_twisting_speed: float = 0,
      desired_body_height: float = 0.45,
      num_legs: int = 4,
      friction_coeffs: Sequence[float] = (0.45, 0.45, 0.45, 0.45),
  ):
    self._gait_generator = gait_generator
    self._state_estimator = state_estimator
    self.desired_speed = desired_speed
    self.desired_twisting_speed = desired_twisting_speed

    self._desired_body_height = desired_body_height
    self._num_legs = num_legs
    self._friction_coeffs = np.array(friction_coeffs)

  def reset(self, current_time):
    del current_time

  def update(self, current_time):
    del current_time

  def _estimate_robot_height(self, contacts, orientation, joint_pos):
    if np.sum(contacts) == 0:
      # All foot in air, no way to estimate
      return self._desired_body_height
    else:
      base_orientation = orientation
      rot_mat = a1.pybullet.getMatrixFromQuaternion(base_orientation)
      rot_mat = np.array(rot_mat).reshape((3, 3))

      
      foot_positions = a1.GetFootPositionsInBaseFrame(joint_pos)
      foot_positions_world_frame = (rot_mat.dot(foot_positions.T)).T
      useful_heights = contacts * (-foot_positions_world_frame[:, 2])
      return np.sum(useful_heights) / np.sum(contacts)

  def get_action(self,orientation, roll_pitch_yaw_rate,joint_pos):
    """Computes the torque for stance legs."""
    # Actual q and dq
    contacts = np.array(
        [(leg_state in (gait_generator_lib.LegState.STANCE,
                        gait_generator_lib.LegState.EARLY_CONTACT))
         for leg_state in self._gait_generator.desired_leg_state],
        dtype=np.int32)
    orientation = a1.get_pybullet_quat(orientation)
    robot_com_position = np.array(
        (0., 0., self._estimate_robot_height(contacts,orientation, joint_pos)))
    robot_com_velocity = self._state_estimator.com_velocity_body_frame
    robot_com_roll_pitch_yaw = np.array(a1.pybullet.getEulerFromQuaternion(orientation))
    robot_com_roll_pitch_yaw[2] = 0  # To prevent yaw drifting
    robot_com_roll_pitch_yaw_rate = roll_pitch_yaw_rate
    robot_q = np.hstack((robot_com_position, robot_com_roll_pitch_yaw))
    robot_dq = np.hstack((robot_com_velocity, robot_com_roll_pitch_yaw_rate))

    # Desired q and dq
    desired_com_position = np.array((0., 0., self._desired_body_height),
                                    dtype=np.float64)
    desired_com_velocity = np.array(
        (self.desired_speed[0], self.desired_speed[1], 0.), dtype=np.float64)
    desired_com_roll_pitch_yaw = np.array((0., 0., 0.), dtype=np.float64)
    desired_com_angular_velocity = np.array(
        (0., 0., self.desired_twisting_speed), dtype=np.float64)
    desired_q = np.hstack((desired_com_position, desired_com_roll_pitch_yaw))
    desired_dq = np.hstack(
        (desired_com_velocity, desired_com_angular_velocity))
    # Desired ddq
    desired_ddq = KP * (desired_q - robot_q) + KD * (desired_dq - robot_dq)
    desired_ddq = np.clip(desired_ddq, MIN_DDQ, MAX_DDQ)
    contact_forces = qp_torque_optimizer.compute_contact_force(desired_ddq, contacts=contacts,angles = joint_pos)

    action = {}
    for leg_id, force in enumerate(contact_forces):
      motor_torques = a1.MapContactForceToJointTorques(leg_id, force, joint_pos)
      for joint_id, torque in motor_torques.items():
        action[joint_id] = (0, 0, 0, 0, torque)
    return action, contact_forces
