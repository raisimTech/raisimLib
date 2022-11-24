"""A model based controller framework."""

from __future__ import absolute_import
from __future__ import division
#from __future__ import google_type_annotations
from __future__ import print_function

import os
import inspect

from raisimGymTorch.mpc_BC_perturb.robot_helper import NUM_MOTORS
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)
import numpy as np
import time
from typing import Any, Callable
from raisimGymTorch.mpc_BC_perturb import robot_helper

class LocomotionController(object):
  """Generates the quadruped locomotion.

  The actual effect of this controller depends on the composition of each
  individual subcomponent.

  """
  def __init__(
      self,
      gait_generator,
      state_estimator,
      swing_leg_controller,
      stance_leg_controller,
      curr_time,
  ):
    """Initializes the class.

    Args:
      robot: A robot instance.
      gait_generator: Generates the leg swing/stance pattern.
      state_estimator: Estimates the state of the robot (e.g. center of mass
        position or velocity that may not be observable from sensors).
      swing_leg_controller: Generates motor actions for swing legs.
      stance_leg_controller: Generates motor actions for stance legs.
      clock: A real or fake clock source.
    """
    self._reset_time = curr_time
    self._time_since_reset = 0
    self._gait_generator = gait_generator
    self._state_estimator = state_estimator
    self._swing_leg_controller = swing_leg_controller
    self._stance_leg_controller = stance_leg_controller

  @property
  def swing_leg_controller(self):
    return self._swing_leg_controller

  @property
  def stance_leg_controller(self):
    return self._stance_leg_controller

  @property
  def gait_generator(self):
    return self._gait_generator

  @property
  def state_estimator(self):
    return self._state_estimator

  def reset(self,curr_time,initial_joint_pos):
    self._reset_time = curr_time
    self._time_since_reset = 0.0
    self._gait_generator.reset(self._time_since_reset)
    self._state_estimator.reset(self._time_since_reset)
    self._swing_leg_controller.reset(self._time_since_reset,initial_joint_pos)
    self._stance_leg_controller.reset(self._time_since_reset)

  def update(self,ctrl_time_step,obs):
    self._time_since_reset = ctrl_time_step - self._reset_time

    footContacts = obs[4:8]
    baseLinVel = obs[20:23]
    jointPos = obs[8:20]
    footContacts = (footContacts==1)
    self._gait_generator.update(self._time_since_reset,footContacts)
    self._state_estimator.update(self._time_since_reset,baseLinVel)
    self._swing_leg_controller.update(self._time_since_reset, jointPos)
    self._stance_leg_controller.update(self._time_since_reset)

  def get_action(self,obs):
    """Returns the control ouputs (e.g. positions/torques) for all motors."""
    baseAngVel = obs[23:26]
    bodyOrientation = obs[0:4]
    jointPos = obs[8:20]
    swing_action = self._swing_leg_controller.get_action(baseAngVel[2])
    stance_action, qp_sol = self._stance_leg_controller.get_action(bodyOrientation,baseAngVel,jointPos)
    
    action = []
    for joint_id in range(NUM_MOTORS):
      if joint_id in swing_action:
        action.extend(swing_action[joint_id])
      else:
        assert joint_id in stance_action
        action.extend(stance_action[joint_id])
    action = np.array(action, dtype=np.float32)

    return action, qp_sol

  def get_contact_force(self,obs):
    """Returns the control ouputs (e.g. positions/torques) for all motors."""
    baseAngVel = obs[23:26]
    bodyOrientation = obs[0:4]
    jointPos = obs[8:20]
    _, qp_sol = self._stance_leg_controller.get_action(bodyOrientation,baseAngVel,jointPos)
    return qp_sol

  def get_control_action(self,contact_force,obs):
    """Returns the control ouputs (e.g. positions/torques) for all motors."""
    baseAngVel = obs[23:26]
    jointPos = obs[8:20]
    swing_action = self._swing_leg_controller.get_action(baseAngVel[2])

    stance_action = {}
    for leg_id, force in enumerate(contact_force):
      motor_torques = robot_helper.MapContactForceToJointTorques(leg_id, force, jointPos)
      for joint_id, torque in motor_torques.items():
        stance_action[joint_id] = (0, 0, 0, 0, torque)
    
    action = []
    for joint_id in range(NUM_MOTORS):
      if joint_id in swing_action:
        action.extend(swing_action[joint_id])
      else:
        assert joint_id in stance_action
        action.extend(stance_action[joint_id])
    action = np.array(action, dtype=np.float32)

    return action
  
