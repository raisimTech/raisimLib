import numpy as np
import math
import copy
import pybullet

NUM_MOTORS = 12
NUM_LEGS = 4
TWO_PI = 2 * math.pi
COM_OFFSET = -np.array([0.012731, 0.002186, 0.000515])
HIP_OFFSETS = np.array([[0.183, -0.047, 0.], [0.183, 0.047, 0.],
                        [-0.183, -0.047, 0.], [-0.183, 0.047, 0.]
                        ]) + COM_OFFSET
_DEFAULT_HIP_POSITIONS = (
    (0.17, -0.135, 0),
    (0.17, 0.13, 0),
    (-0.195, -0.135, 0),
    (-0.195, 0.13, 0),
)

INITIAL_POS = np.array([0, 0.9, -1.8] * NUM_LEGS)

JOINT_DIRECTIONS = np.ones(12)
HIP_JOINT_OFFSET = 0.0
UPPER_LEG_JOINT_OFFSET = 0.0
KNEE_JOINT_OFFSET = 0.0
JOINT_OFFSETS = np.array(
    [HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET] * 4)

ABDUCTION_P_GAIN = 100.0
ABDUCTION_D_GAIN = 1.
HIP_P_GAIN = 100.0
HIP_D_GAIN = 2.0
KNEE_P_GAIN = 100.0
KNEE_D_GAIN = 2.0

motor_kp = [
        ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN, ABDUCTION_P_GAIN,
        HIP_P_GAIN, KNEE_P_GAIN, ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN,
        ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN
]
motor_kd = [
    ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN, ABDUCTION_D_GAIN,
    HIP_D_GAIN, KNEE_D_GAIN, ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN,
    ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN
]

def GetFootPositionsInBaseFrame(angles):
  motor_angles = MapToMinusPiToPi(angles)
  return foot_positions_in_base_frame(motor_angles)

def MapToMinusPiToPi(angles):
  mapped_angles = copy.deepcopy(angles)
  for i in range(len(angles)):
    mapped_angles[i] = math.fmod(angles[i], TWO_PI)
    if mapped_angles[i] >= math.pi:
      mapped_angles[i] -= TWO_PI
    elif mapped_angles[i] < -math.pi:
      mapped_angles[i] += TWO_PI
  return mapped_angles


def foot_position_in_hip_frame(angles, l_hip_sign=1):
  theta_ab, theta_hip, theta_knee = angles[0], angles[1], angles[2]
  l_up = 0.2
  l_low = 0.2
  l_hip = 0.08505 * l_hip_sign
  leg_distance = np.sqrt(l_up**2 + l_low**2 +
                         2 * l_up * l_low * np.cos(theta_knee))
  eff_swing = theta_hip + theta_knee / 2

  off_x_hip = -leg_distance * np.sin(eff_swing)
  off_z_hip = -leg_distance * np.cos(eff_swing)
  off_y_hip = l_hip

  off_x = off_x_hip
  off_y = np.cos(theta_ab) * off_y_hip - np.sin(theta_ab) * off_z_hip
  off_z = np.sin(theta_ab) * off_y_hip + np.cos(theta_ab) * off_z_hip
  return np.array([off_x, off_y, off_z])

def foot_positions_in_base_frame(foot_angles):
  foot_angles = foot_angles.reshape((4, 3))
  foot_positions = np.zeros((4, 3))
  for i in range(4):
    foot_positions[i] = foot_position_in_hip_frame(foot_angles[i],
                                                   l_hip_sign=(-1)**(i + 1))
  return foot_positions + HIP_OFFSETS


def foot_position_in_hip_frame_to_joint_angle(foot_position, l_hip_sign=1):
  l_up = 0.2
  l_low = 0.2
  l_hip = 0.08505 * l_hip_sign
  x, y, z = foot_position[0], foot_position[1], foot_position[2]
  theta_knee = -np.arccos(
      (x**2 + y**2 + z**2 - l_hip**2 - l_low**2 - l_up**2) /
      (2 * l_low * l_up))
  l = np.sqrt(l_up**2 + l_low**2 + 2 * l_up * l_low * np.cos(theta_knee))
  theta_hip = np.arcsin(-x / l) - theta_knee / 2
  c1 = l_hip * y - l * np.cos(theta_hip + theta_knee / 2) * z
  s1 = l * np.cos(theta_hip + theta_knee / 2) * y + l_hip * z
  theta_ab = np.arctan2(s1, c1)
  return np.array([theta_ab, theta_hip, theta_knee])


def ComputeMotorAnglesFromFootLocalPosition(leg_id, foot_local_position):
    motors_per_leg = NUM_MOTORS // NUM_LEGS
    joint_position_idxs = \
    list(range(leg_id * motors_per_leg,
              leg_id * motors_per_leg + motors_per_leg))

    joint_angles = foot_position_in_hip_frame_to_joint_angle(
        foot_local_position - HIP_OFFSETS[leg_id],
        l_hip_sign=(-1)**(leg_id + 1))

    # Joint offset is necessary for Laikago.
    joint_angles = np.multiply(
        np.asarray(joint_angles) -
        np.asarray(JOINT_OFFSETS)[joint_position_idxs],
        JOINT_DIRECTIONS[joint_position_idxs])

    # Return the joing index (the same as when calling GetMotorAngles) as well
    # as the angles.
    return joint_position_idxs, joint_angles.tolist()


def analytical_leg_jacobian(leg_angles, leg_id):
  """
  Computes the analytical Jacobian.
  Args:
  ` leg_angles: a list of 3 numbers for current abduction, hip and knee angle.
    l_hip_sign: whether it's a left (1) or right(-1) leg.
  """
  l_up = 0.2
  l_low = 0.2
  l_hip = 0.08505 * (-1)**(leg_id + 1)

  t1, t2, t3 = leg_angles[0], leg_angles[1], leg_angles[2]
  l_eff = np.sqrt(l_up**2 + l_low**2 + 2 * l_up * l_low * np.cos(t3))
  t_eff = t2 + t3 / 2
  J = np.zeros((3, 3))
  J[0, 0] = 0
  J[0, 1] = -l_eff * np.cos(t_eff)
  J[0, 2] = l_low * l_up * np.sin(t3) * np.sin(t_eff) / l_eff - l_eff * np.cos(
      t_eff) / 2
  J[1, 0] = -l_hip * np.sin(t1) + l_eff * np.cos(t1) * np.cos(t_eff)
  J[1, 1] = -l_eff * np.sin(t1) * np.sin(t_eff)
  J[1, 2] = -l_low * l_up * np.sin(t1) * np.sin(t3) * np.cos(
      t_eff) / l_eff - l_eff * np.sin(t1) * np.sin(t_eff) / 2
  J[2, 0] = l_hip * np.cos(t1) + l_eff * np.sin(t1) * np.cos(t_eff)
  J[2, 1] = l_eff * np.sin(t_eff) * np.cos(t1)
  J[2, 2] = l_low * l_up * np.sin(t3) * np.cos(t1) * np.cos(
      t_eff) / l_eff + l_eff * np.sin(t_eff) * np.cos(t1) / 2
  return J

def ComputeJacobian(leg_id,joint_pos):
    """Compute the Jacobian for a given leg."""
    # Does not work for Minitaur which has the four bar mechanism for now.
    motor_angles = joint_pos
    return analytical_leg_jacobian(motor_angles, leg_id)

def MapContactForceToJointTorques(leg_id, contact_force,joint_pos):
    """Maps the foot contact force to the leg joint torques."""
    joint_pos = joint_pos.reshape((4,3))
    jv = ComputeJacobian(leg_id,joint_pos[leg_id])
    
    motor_torques_list = np.matmul(contact_force, jv)
    motor_torques_dict = {}
    motors_per_leg = NUM_MOTORS // NUM_LEGS
    for torque_id, joint_id in enumerate(range(leg_id * motors_per_leg, (leg_id + 1) * motors_per_leg)):
      motor_torques_dict[joint_id] = motor_torques_list[torque_id]
    return motor_torques_dict    

def get_pybullet_quat(quat):
  quat_py = []
  quat_py.extend(quat[1:])
  quat_py.append(quat[0])
  return np.array(quat_py)