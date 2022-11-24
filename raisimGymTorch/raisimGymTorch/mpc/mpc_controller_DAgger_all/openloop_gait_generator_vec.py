"""Gait pattern planning module."""

from __future__ import absolute_import
from __future__ import division
#from __future__ import google_type_annotations
from __future__ import print_function

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import logging
import math

import numpy as np
from typing import Any, Sequence

from raisimGymTorch.mpc.mpc_controller_DAgger_all import gait_generator

LAIKAGO_TROTTING = (
    gait_generator.LegState.SWING,
    gait_generator.LegState.STANCE,
    gait_generator.LegState.STANCE,
    gait_generator.LegState.SWING,
)
_NOMINAL_STANCE_DURATION = (0.3, 0.3, 0.3, 0.3)
_NOMINAL_DUTY_FACTOR = (0.5, 0.5, 0.5, 0.5)
_NOMINAL_CONTACT_DETECTION_PHASE = 0.1
SWING = 0.
STANCE = 1.
# A swing leg that collides with the ground.
EARLY_CONTACT = 2.
# A stance leg that loses contact.
LOSE_CONTACT = 3.

class OpenloopGaitGenerator_vec(gait_generator.GaitGenerator):
  """Generates openloop gaits for quadruped robots.

  A flexible open-loop gait generator. Each leg has its own cycle and duty
  factor. And the state of each leg alternates between stance and swing. One can
  easily formuate a set of common quadruped gaits like trotting, pacing,
  pronking, bounding, etc by tweaking the input parameters.
  """
  def __init__(
      self,
      num_env,
      stance_duration: Sequence[float] = _NOMINAL_STANCE_DURATION,
      duty_factor: Sequence[float] = _NOMINAL_DUTY_FACTOR,
      initial_leg_state: Sequence[gait_generator.LegState] = LAIKAGO_TROTTING,
      initial_leg_phase: Sequence[float] = (0, 0, 0, 0),
      contact_detection_phase_threshold:
      float = _NOMINAL_CONTACT_DETECTION_PHASE,
  ):
    """Initializes the class.

    Args:
      robot: A quadruped robot that at least implements the GetFootContacts API
        and num_legs property.
      stance_duration: The desired stance duration.
      duty_factor: The ratio  stance_duration / total_gait_cycle.
      initial_leg_state: The desired initial swing/stance state of legs indexed
        by their id.
      initial_leg_phase: The desired initial phase [0, 1] of the legs within the
        full swing + stance cycle.
      contact_detection_phase_threshold: Updates the state of each leg based on
        contact info, when the current normalized phase is greater than this
        threshold. This is essential to remove false positives in contact
        detection when phase switches. For example, a swing foot at at the
        beginning of the gait cycle might be still on the ground.
    """
    self.num_env = num_env
    self._stance_duration = np.tile(stance_duration,(self.num_env,1)) # (num_env,4)
    self._duty_factor = np.tile(duty_factor,(self.num_env,1))
    self._swing_duration = (self._stance_duration / self._duty_factor) - np.array(self._stance_duration)
    if len(initial_leg_phase) != 4:
      raise ValueError(
          "The number of leg phases should be the same as number of legs.")
    self._initial_leg_phase = np.tile(initial_leg_phase,(self.num_env,1))
    if len(initial_leg_state) != 4:
      raise ValueError(
          "The number of leg states should be the same of number of legs.")
    self._initial_leg_state = np.tile(initial_leg_state,(self.num_env,1))
    self._next_leg_state = np.zeros((self.num_env,4))
    # The ratio in cycle is duty factor if initial state of the leg is STANCE,
    # and 1 - duty_factory if the initial state of the leg is SWING.
    self._initial_state_ratio_in_cycle = np.zeros((self.num_env,4))
    self._initial_state_ratio_in_cycle[self._initial_leg_state == SWING] = \
                                          1-self._duty_factor[self._initial_leg_state == SWING]
    self._next_leg_state[self._initial_leg_state == SWING] = STANCE
    self._initial_state_ratio_in_cycle[self._initial_leg_state != SWING] = \
                                          self._duty_factor[self._initial_leg_state != SWING]
    self._next_leg_state[self._initial_leg_state != SWING] = SWING

    self._contact_detection_phase_threshold = contact_detection_phase_threshold

    # The normalized phase within swing or stance duration.
    self._normalized_phase = None
    self._leg_state = None
    self._desired_leg_state = None
    self.full_cycle_period = self._stance_duration/ self._duty_factor
    self.dt_init_phase = self._initial_leg_phase * self.full_cycle_period
    self.reset(0)

  def reset(self, current_time):
    # The normalized phase within swing or stance duration.
    self._normalized_phase = np.zeros((self.num_env,4))
    self._leg_state = self._initial_leg_state.copy()
    self._desired_leg_state = self._initial_leg_state.copy()
  
  def reset_indiv(self, current_time,i):
    # The normalized phase within swing or stance duration.
    self._normalized_phase[i] = np.zeros(4)
    self._leg_state[i] = self._initial_leg_state[i]
    self._desired_leg_state[i] = self._initial_leg_state[i]

  @property
  def desired_leg_state(self) -> Sequence[gait_generator.LegState]:
    """The desired leg SWING/STANCE states.

    Returns:
      The SWING/STANCE states for all legs.

    """
    return self._desired_leg_state

  @property
  def leg_state(self) -> Sequence[gait_generator.LegState]:
    """The leg state after considering contact with ground.

    Returns:
      The actual state of each leg after accounting for contacts.
    """
    return self._leg_state

  @property
  def swing_duration(self) -> Sequence[float]:
    return self._swing_duration

  @property
  def stance_duration(self) -> Sequence[float]:
    return self._stance_duration

  @property
  def normalized_phase(self) -> Sequence[float]:
    """The phase within the current swing or stance cycle.

    Reflects the leg's phase within the curren swing or stance stage. For
    example, at the end of the current swing duration, the phase will
    be set to 1 for all swing legs. Same for stance legs.

    Returns:
      Normalized leg phase for all legs.

    """
    return self._normalized_phase

  def update(self, current_time):
    augmented_time = current_time.reshape(self.num_env,1) + self.dt_init_phase
    phase_in_full_cycle = np.fmod(augmented_time,self.full_cycle_period) / self.full_cycle_period
    
    self._desired_leg_state[phase_in_full_cycle < self._initial_state_ratio_in_cycle] = \
               self._initial_leg_state[phase_in_full_cycle < self._initial_state_ratio_in_cycle]
    self._normalized_phase[phase_in_full_cycle < self._initial_state_ratio_in_cycle] = \
              (phase_in_full_cycle / self._initial_state_ratio_in_cycle)[phase_in_full_cycle < self._initial_state_ratio_in_cycle]
    self._desired_leg_state[phase_in_full_cycle >= self._initial_state_ratio_in_cycle] = \
              self._next_leg_state[phase_in_full_cycle >= self._initial_state_ratio_in_cycle]
    self._normalized_phase[phase_in_full_cycle >= self._initial_state_ratio_in_cycle] = \
              ((phase_in_full_cycle -self._initial_state_ratio_in_cycle) / (1 - self._initial_state_ratio_in_cycle))[phase_in_full_cycle >= self._initial_state_ratio_in_cycle]
  
