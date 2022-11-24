# coding=utf-8
# Copyright 2020 The Google Research Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Motion data class for processing motion clips."""
import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import json
import logging
import math
import enum
import numpy as np

from motion_imitation.utilities import pose3d
from motion_imitation.utilities import motion_util
from pybullet_utils import transformations


class LoopMode(enum.Enum):
  """Specifies if a motion should loop or stop at the last frame."""
  Clamp = 0
  Wrap = 1


class MotionData(object):
  """Motion data representing a pose trajectory for a character.

  The pose includes:
  [root position, root orientation, joint poses (e.g. rotations)]
  """

  POS_SIZE = 3
  ROT_SIZE = 4
  VEL_SIZE = 3
  ANG_VEL_SIZE = 3

  _LOOP_MODE_KEY = "LoopMode"
  _FRAME_DURATION_KEY = "FrameDuration"
  _FRAMES_KEY = "Frames"
  _ENABLE_CYCLE_OFFSET_POSITION_KEY = "EnableCycleOffsetPosition"
  _ENABLE_CYCLE_OFFSET_ROTATION_KEY = "EnableCycleOffsetRotation"

  def __init__(self, motion_file):
    """Initialize motion data.

    Args:
      motion_file: The path to the motion data file.
    """
    self._loop_mode = LoopMode.Clamp
    self._frame_duration = 0
    self._frames = None
    self._frame_vels = None

    self.load(motion_file)

    # precompute the net changes in root position and rotation over the course
    # of the motion
    self._cycle_delta_pos = self._calc_cycle_delta_pos()
    self._cycle_delta_heading = self._calc_cycle_delta_heading()

    return

  def load(self, motion_file):
    """Load motion data from file.

    The file must be in JSON format.

    Args:
      motion_file: The path to the motion data file.
    """

    logging.info("Loading motion from: {:s}".format(motion_file))
    with open(motion_file, "r") as f:
      motion_json = json.load(f)

      self._loop_mode = LoopMode[motion_json[self._LOOP_MODE_KEY]]
      self._frame_duration = float(motion_json[self._FRAME_DURATION_KEY])

      if self._ENABLE_CYCLE_OFFSET_POSITION_KEY in motion_json:
        self._enable_cycle_offset_pos = bool(
            motion_json[self._ENABLE_CYCLE_OFFSET_POSITION_KEY])
      else:
        self._enable_cycle_offset_pos = False

      if self._ENABLE_CYCLE_OFFSET_ROTATION_KEY in motion_json:
        self._enable_cycle_offset_rot = bool(
            motion_json[self._ENABLE_CYCLE_OFFSET_ROTATION_KEY])
      else:
        self._enable_cycle_offset_rot = False

      self._frames = np.array(motion_json[self._FRAMES_KEY])
      self._postprocess_frames(self._frames)

      self._frame_vels = self._calc_frame_vels()

      assert (self._frames.shape[0] > 0), "Must have at least 1 frame."
      assert (self._frames.shape[1] > self.POS_SIZE +
              self.ROT_SIZE), "Frames have too few degrees of freedom."
      assert (self._frame_duration > 0), "Frame duration must be positive."

      logging.info("Loaded motion from {:s}.".format(motion_file))

    return

  def get_num_frames(self):
    """Get the number of frames in the motion data.

    Returns:
      Number of frames in motion data.

    """
    return self._frames.shape[0]

  def get_frame_size(self):
    """Get the size of each frame.

    Returns:
      Size of each frame in motion data.

    """
    return self._frames.shape[-1]

  def get_frame_vel_size(self):
    """Get the size of the root velocity in each frame.

    Returns:
      Size of root velocity.

    """
    return self.get_frame_size() - self.POS_SIZE - self.ROT_SIZE \
           + self.VEL_SIZE + self.ANG_VEL_SIZE

  def get_frame_duration(self):
    """Get the duration (seconds) of a single rame.

    Returns:
      The duration of a frame.

    """
    return self._frame_duration

  def get_frame(self, f):
    """Get a specific frame that represents the character's pose at that point

    in time.

    Args:
      f: Index of the frame.

    Returns:
      The selected frame.

    """
    return self._frames[f, :]

  def get_frame_vel(self, f):
    """Get the velocities of each joint at a specific frame.

    Args:
      f: Index of the frame.

    Returns:
      The selected frame velocity.

    """
    return self._frame_vels[f, :]

  def get_frame_time(self, f):
    """Get the start time of a specified frame

    Args:
      f: Index of the frame.

    Returns:
      Start time of the frame.

    """
    return f * self.get_frame_duration()

  def get_frames(self):
    """Get all frames.

    Returns:
      All frames in reference motion.

    """
    return self._frames

  def get_duration(self):
    """Get the duration (seconds) of the entire motion.

    Returns:
      The duration of the motion.

    """
    frame_dur = self.get_frame_duration()
    num_frames = self.get_num_frames()
    motion_dur = frame_dur * (num_frames - 1)
    return motion_dur

  def calc_phase(self, time):
    """Calaculates the phase for a given point in time.

    The phase is a scalar
    value between [0, 1], with 0 denoting the start of a motion, and 1 the end
    of a motion.

    Args:
      time: The time to be used when computing the phase.

    Returns:
      The duration of the motion.

    """
    dur = self.get_duration()
    phase = time / dur

    if self.enable_loop():
      phase -= np.floor(phase)
    else:
      phase = np.clip(phase, 0.0, 1.0)

    return phase

  def calc_cycle_count(self, time):
    """Calculates the number of cycles completed of a motion for a given amount

    of time.

    Args:
      time: The time elapsed since the motion began.

    Returns:
      The number of motion cycles.

    """
    dur = self.get_duration()
    phases = time / dur
    count = int(math.floor(phases))

    if not self.enable_loop():
      count = np.clip(count, 0, 1)

    return count

  def enable_loop(self):
    """Check if looping is enabled for the motion.

    Returns:
      Boolean indicating if looping is enabled.

    """
    loop = (self._loop_mode is LoopMode.Wrap)
    return loop

  def is_over(self, time):
    """Check if a motion has ended after a specific point in time.

    Args:
      time: Time elapsed since the motion began.

    Returns:
      Boolean indicating if the motion is over.

    """
    over = (not self.enable_loop()) and (time >= self.get_duration())
    return over

  def get_frame_root_pos(self, frame):
    """Get the root position from a frame.

    Args:
      frame: Frame from which the root position is to be extracted.

    Returns:
      Root position from the given frame.

    """
    root_pos = frame[:self.POS_SIZE].copy()
    return root_pos

  def set_frame_root_pos(self, root_pos, out_frame):
    """Set the root position for a frame.

    Args:
      root_pos: Root position to be set for a frame
      out_frame: Frame in which the root position is to be set.
    """
    out_frame[:self.POS_SIZE] = root_pos
    return

  def get_frame_root_rot(self, frame):
    """Get the root rotation from a frame.

    Args:
      frame: Frame from which the root rotation is to be extracted.

    Returns:
      Root rotation (quaternion) from the given frame.

    """
    root_rot = frame[self.POS_SIZE:(self.POS_SIZE + self.ROT_SIZE)].copy()
    return root_rot

  def set_frame_root_rot(self, root_rot, out_frame):
    """Set the root rotation for a frame.

    Args:
      root_rot: Root rotation to be set for a frame
      out_frame: Frame in which the root rotation is to be set.
    """
    out_frame[self.POS_SIZE:(self.POS_SIZE + self.ROT_SIZE)] = root_rot
    return

  def get_frame_joints(self, frame):
    """Get the pose of each joint from a frame.

    Args:
      frame: Frame from which the joint pose is to be extracted.

    Returns:
      Array containing the pose of each joint in the given frame.

    """
    joints = frame[(self.POS_SIZE + self.ROT_SIZE):].copy()
    return joints

  def set_frame_joints(self, joints, out_frame):
    """Set the joint pose for a frame.

    Args:
      joints: Pose of each joint to be set for a frame.
      out_frame: Frame in which the joint poses is to be set.
    """
    out_frame[(self.POS_SIZE + self.ROT_SIZE):] = joints
    return

  def get_frame_root_vel(self, frame):
    """Get the root linear velocity from a frame.

    Args:
      frame: Frame from which the root linear velocity is to be extracted.

    Returns:
      Root linear velocity from the given frame.

    """
    root_vel = frame[:self.VEL_SIZE].copy()
    return root_vel

  def set_frame_root_vel(self, root_vel, out_frame):
    """Set the root linear velocity for a frame.

    Args:
      root_vel: Root linear velocity to be set for a frame.
      out_frame: Frame in which the root linear velocity is to be set.
    """
    out_frame[:self.VEL_SIZE] = root_vel
    return

  def get_frame_root_ang_vel(self, frame):
    """Get the root angular velocity from a frame.

    Args:
      frame: Frame from which the root position is to be extracted.

    Returns:
      Root position from the given frame.

    """
    root_ang_vel = frame[self.VEL_SIZE:(self.VEL_SIZE
                                        + self.ANG_VEL_SIZE)].copy()
    return root_ang_vel

  def set_frame_root_ang_vel(self, root_ang_vel, out_frame):
    """Set the root angular velocity for a frame.

    Args:
      root_ang_vel: Root angular velocity to be set for a frame.
      out_frame: Frame in which the root angular velocity is to be set.
    """
    out_frame[self.VEL_SIZE:(self.VEL_SIZE + self.ANG_VEL_SIZE)] = root_ang_vel
    return

  def get_frame_joints_vel(self, frame):
    """Get the velocity of each joint from a frame.

    Args:
      frame: Frame from which the joint velocities is to be extracted.

    Returns:
      Array containing the velocity of each joint in the given frame.

    """
    vel = frame[(self.VEL_SIZE + self.ANG_VEL_SIZE):].copy()
    return vel

  def set_frame_joints_vel(self, vel, out_frame):
    """Set the joint velocities for a frame.

    Args:
      vel: Joint velocities to be set for a frame.
      out_frame: Frame in which the joint velocities are to be set.
    """
    out_frame[(self.VEL_SIZE + self.ANG_VEL_SIZE):] = vel
    return

  def calc_frame(self, time):
    """Calculates the frame for a given point in time.

    Args:
      time: Time at which the frame is to be computed.
    Return: An array containing the frame for the given point in time,
      specifying the pose of the character.
    """
    f0, f1, blend = self.calc_blend_idx(time)

    frame0 = self.get_frame(f0)
    frame1 = self.get_frame(f1)
    blend_frame = self.blend_frames(frame0, frame1, blend)

    blend_root_pos = self.get_frame_root_pos(blend_frame)
    blend_root_rot = self.get_frame_root_rot(blend_frame)

    cycle_count = self.calc_cycle_count(time)
    cycle_offset_pos = self._calc_cycle_offset_pos(cycle_count)
    cycle_offset_rot = self._calc_cycle_offset_rot(cycle_count)

    blend_root_pos = pose3d.QuaternionRotatePoint(blend_root_pos,
                                                  cycle_offset_rot)
    blend_root_pos += cycle_offset_pos

    blend_root_rot = transformations.quaternion_multiply(
        cycle_offset_rot, blend_root_rot)
    blend_root_rot = motion_util.standardize_quaternion(blend_root_rot)

    self.set_frame_root_pos(blend_root_pos, blend_frame)
    self.set_frame_root_rot(blend_root_rot, blend_frame)

    return blend_frame

  def calc_frame_vel(self, time):
    """Calculates the frame velocity for a given point in time.

    Args:
      time: Time at which the velocities are to be computed.
    Return: An array containing the frame velocity for the given point in time,
      specifying the velocity of the root and all joints.
    """
    f0, f1, blend = self.calc_blend_idx(time)

    frame_vel0 = self.get_frame_vel(f0)
    frame_vel1 = self.get_frame_vel(f1)
    blend_frame_vel = self.blend_frame_vels(frame_vel0, frame_vel1, blend)

    root_vel = self.get_frame_root_vel(blend_frame_vel)
    root_ang_vel = self.get_frame_root_ang_vel(blend_frame_vel)

    cycle_count = self.calc_cycle_count(time)
    cycle_offset_rot = self._calc_cycle_offset_rot(cycle_count)
    root_vel = pose3d.QuaternionRotatePoint(root_vel, cycle_offset_rot)
    root_ang_vel = pose3d.QuaternionRotatePoint(root_ang_vel, cycle_offset_rot)

    self.set_frame_root_vel(root_vel, blend_frame_vel)
    self.set_frame_root_ang_vel(root_ang_vel, blend_frame_vel)

    return blend_frame_vel

  def blend_frames(self, frame0, frame1, blend):
    """Linearly interpolate between two frames.

    Args:
      frame0: First frame to be blended corresponds to (blend = 0).
      frame1: Second frame to be blended corresponds to (blend = 1).
      blend: Float between [0, 1], specifying the interpolation between
        the two frames.
    Returns:
      An interpolation of the two frames.
    """
    root_pos0 = self.get_frame_root_pos(frame0)
    root_pos1 = self.get_frame_root_pos(frame1)

    root_rot0 = self.get_frame_root_rot(frame0)
    root_rot1 = self.get_frame_root_rot(frame1)

    joints0 = self.get_frame_joints(frame0)
    joints1 = self.get_frame_joints(frame1)

    blend_root_pos = (1.0 - blend) * root_pos0 + blend * root_pos1
    blend_root_rot = transformations.quaternion_slerp(root_rot0, root_rot1,
                                                      blend)
    blend_joints = (1.0 - blend) * joints0 + blend * joints1

    blend_root_rot = motion_util.standardize_quaternion(blend_root_rot)

    blend_frame = np.zeros(self.get_frame_size())
    self.set_frame_root_pos(blend_root_pos, blend_frame)
    self.set_frame_root_rot(blend_root_rot, blend_frame)
    self.set_frame_joints(blend_joints, blend_frame)
    return blend_frame

  def blend_frame_vels(self, frame_vel0, frame_vel1, blend):
    """Linearly interpolate between two frame velocities.

    Args:
      frame_vel0: First frame velocities to be blended corresponds to
        (blend = 0).
      frame_vel1: Second frame velocities to be blended corresponds to
        (blend = 1).
      blend: Float between [0, 1], specifying the interpolation between
        the two frames.
    Returns:
      An interpolation of the two frame velocities.
    """
    blend_frame_vel = (1.0 - blend) * frame_vel0 + blend * frame_vel1
    return blend_frame_vel

  def _postprocess_frames(self, frames):
    """Postprocesses frames to ensure they satisfy certain properties,

    such as normalizing and standardizing all quaternions.

    Args:
      frames: Array containing frames to be processed. Each row of the array
        should represent a frame.
    Returns: An array containing the post processed frames.
    """
    num_frames = frames.shape[0]
    if num_frames > 0:
      first_frame = self._frames[0]
      pos_start = self.get_frame_root_pos(first_frame)

      for f in range(num_frames):
        curr_frame = frames[f]

        root_pos = self.get_frame_root_pos(curr_frame)
        root_pos[0] -= pos_start[0]
        root_pos[1] -= pos_start[1]

        root_rot = self.get_frame_root_rot(curr_frame)
        root_rot = pose3d.QuaternionNormalize(root_rot)
        root_rot = motion_util.standardize_quaternion(root_rot)

        self.set_frame_root_pos(root_pos, curr_frame)
        self.set_frame_root_rot(root_rot, curr_frame)

    return

  def _calc_cycle_delta_pos(self):
    """Calculates the net change in the root position after a cycle.

    Returns:
      Net translation of the root position.
    """
    first_frame = self._frames[0]
    last_frame = self._frames[-1]

    pos_start = self.get_frame_root_pos(first_frame)
    pos_end = self.get_frame_root_pos(last_frame)
    cycle_delta_pos = pos_end - pos_start
    cycle_delta_pos[2] = 0  # only translate along horizontal plane

    return cycle_delta_pos

  def _calc_cycle_delta_heading(self):
    """Calculates the net change in the root heading after a cycle.

    Returns:
      Net change in heading.
    """
    first_frame = self._frames[0]
    last_frame = self._frames[-1]

    rot_start = self.get_frame_root_rot(first_frame)
    rot_end = self.get_frame_root_rot(last_frame)
    inv_rot_start = transformations.quaternion_conjugate(rot_start)
    drot = transformations.quaternion_multiply(rot_end, inv_rot_start)
    cycle_delta_heading = motion_util.calc_heading(drot)

    return cycle_delta_heading

  def _calc_cycle_offset_pos(self, num_cycles):
    """Calculates change in the root position after a given number of cycles.

    Args:
      num_cycles: Number of cycles since the start of the motion.

    Returns:
      Net translation of the root position.
    """

    if not self._enable_cycle_offset_pos:
      cycle_offset_pos = np.zeros(3)
    else:
      if not self._enable_cycle_offset_rot:
        cycle_offset_pos = num_cycles * self._cycle_delta_pos

      else:
        cycle_offset_pos = np.zeros(3)
        for i in range(num_cycles):
          curr_heading = i * self._cycle_delta_heading
          rot = transformations.quaternion_about_axis(curr_heading, [0, 0, 1])
          curr_offset = pose3d.QuaternionRotatePoint(self._cycle_delta_pos, rot)
          cycle_offset_pos += curr_offset

    return cycle_offset_pos

  def _calc_cycle_offset_rot(self, num_cycles):
    """Calculates change in the root rotation after a given number of cycles.

    Args:
      num_cycles: Number of cycles since the start of the motion.

    Returns:
      Net rotation of the root orientation.
    """
    if not self._enable_cycle_offset_rot:
      cycle_offset_rot = np.array([0, 0, 0, 1])
    else:
      heading_offset = num_cycles * self._cycle_delta_heading
      cycle_offset_rot = transformations.quaternion_from_euler(
          0, 0, heading_offset)

    return cycle_offset_rot

  def _calc_frame_vels(self):
    """Calculates the frame velocity of each frame in the motion (self._frames).

    Return:
      An array containing velocities at each frame in self._frames.
    """
    num_frames = self.get_num_frames()
    frame_vel_size = self.get_frame_vel_size()
    dt = self.get_frame_duration()
    frame_vels = np.zeros([num_frames, frame_vel_size])

    for f in range(num_frames - 1):
      frame0 = self.get_frame(f)
      frame1 = self.get_frame(f + 1)

      root_pos0 = self.get_frame_root_pos(frame0)
      root_pos1 = self.get_frame_root_pos(frame1)

      root_rot0 = self.get_frame_root_rot(frame0)
      root_rot1 = self.get_frame_root_rot(frame1)

      joints0 = self.get_frame_joints(frame0)
      joints1 = self.get_frame_joints(frame1)

      root_vel = (root_pos1 - root_pos0) / dt

      root_rot_diff = transformations.quaternion_multiply(
          root_rot1, transformations.quaternion_conjugate(root_rot0))
      root_rot_diff_axis, root_rot_diff_angle = \
        pose3d.QuaternionToAxisAngle(root_rot_diff)
      root_ang_vel = (root_rot_diff_angle / dt) * root_rot_diff_axis

      joints_vel = (joints1 - joints0) / dt

      curr_frame_vel = np.zeros(frame_vel_size)
      self.set_frame_root_vel(root_vel, curr_frame_vel)
      self.set_frame_root_ang_vel(root_ang_vel, curr_frame_vel)
      self.set_frame_joints_vel(joints_vel, curr_frame_vel)

      frame_vels[f, :] = curr_frame_vel

    # replicate the velocity at the last frame
    if num_frames > 1:
      frame_vels[-1, :] = frame_vels[-2, :]

    return frame_vels

  def calc_blend_idx(self, time):
    """Calculate the indices of the two frames and the interpolation value that

    should be used when computing the frame at a given point in time.

    Args:
      time: Time at which the frame is to be computed.
    Return:
      f0: Start framed used for blending.
      f1: End frame used for blending.
      blend: Interpolation value used to blend between the two frames.
    """
    dur = self.get_duration()
    num_frames = self.get_num_frames()

    if not self.enable_loop() and time <= 0:
      f0 = 0
      f1 = 0
      blend = 0
    elif not self.enable_loop() and time >= dur:
      f0 = num_frames - 1
      f1 = num_frames - 1
      blend = 0
    else:
      phase = self.calc_phase(time)

      f0 = int(phase * (num_frames - 1))
      f1 = min(f0 + 1, num_frames - 1)

      norm_time = phase * dur
      time0 = self.get_frame_time(f0)
      time1 = self.get_frame_time(f1)
      assert (norm_time >= time0 - 1e-5) and (norm_time <= time1 + 1e-5)

      blend = (norm_time - time0) / (time1 - time0)

    return f0, f1, blend
