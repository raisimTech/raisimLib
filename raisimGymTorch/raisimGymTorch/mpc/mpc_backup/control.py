from raisimGymTorch.mpc.mpc_controller import com_velocity_estimator
from raisimGymTorch.mpc.mpc_controller import gait_generator as gait_generator_lib
from raisimGymTorch.mpc.mpc_controller import locomotion_controller
from raisimGymTorch.mpc.mpc_controller import openloop_gait_generator
from raisimGymTorch.mpc.mpc_controller import raibert_swing_leg_controller
from raisimGymTorch.mpc.mpc_controller import torque_stance_leg_controller_quadprog as torque_stance_leg_controller
import scipy.interpolate
import numpy as np
# mpc walking parameters
POSITION_INDEX = 0
POSITION_GAIN_INDEX = 1
VELOCITY_INDEX = 2
VELOCITY_GAIN_INDEX = 3
TORQUE_INDEX = 4
MOTOR_COMMAND_DIMENSION = 5



_STANCE_DURATION_SECONDS = [
    0.3
] * 4

_DUTY_FACTOR = [0.6] * 4 # stance time / walking cycle
_INIT_PHASE_FULL_CYCLE = [0.9, 0, 0, 0.9] # trotting

_INIT_LEG_STATE = (
    gait_generator_lib.LegState.SWING,
    gait_generator_lib.LegState.STANCE,
    gait_generator_lib.LegState.STANCE,
    gait_generator_lib.LegState.SWING,
)

MPC_BODY_HEIGHT = 0.25

def _generate_example_linear_angular_speed(t):
  """Creates an example speed profile based on time for demo purpose."""
  vx = 0.6
  vy = 0.2
  wz = 0.8

  time_points = (0, 5, 10, 15, 20, 25,30)
  speed_points = ((0, 0, 0, 0), (0, 0, 0, wz), (vx, 0, 0, 0), (0, 0, 0, -wz), (0, -vy, 0, 0),
                  (0, 0, 0, 0), (0, 0, 0, wz))
#   time_points = (0,5)
#   speed_points = ((vx,0,0,0),(vx,0,0,0))
  speed = scipy.interpolate.interp1d(time_points,
                                     speed_points,
                                     kind="previous",
                                     fill_value="extrapolate",
                                     axis=0)(t)

  return speed[0:3], speed[3], False


def _setup_controller():
    """Demonstrates how to create a locomotion controller."""
    desired_speed = (0, 0)
    desired_twisting_speed = 0

    gait_generator = openloop_gait_generator.OpenloopGaitGenerator(
        stance_duration=_STANCE_DURATION_SECONDS,
        duty_factor=_DUTY_FACTOR,
        initial_leg_phase=_INIT_PHASE_FULL_CYCLE,
        initial_leg_state=_INIT_LEG_STATE)
    window_size = 20
    state_estimator = com_velocity_estimator.COMVelocityEstimator(window_size=window_size)
    sw_controller = raibert_swing_leg_controller.RaibertSwingLegController(
        gait_generator,
        state_estimator,
        desired_speed=desired_speed,
        desired_twisting_speed=desired_twisting_speed,
        desired_height=MPC_BODY_HEIGHT,
        foot_clearance=0.01)

    st_controller = torque_stance_leg_controller.TorqueStanceLegController(
        gait_generator,
        state_estimator,
        desired_speed=desired_speed,
        desired_twisting_speed=desired_twisting_speed,
        desired_body_height=MPC_BODY_HEIGHT
        #,qp_solver = mpc_osqp.QPOASES #or mpc_osqp.OSQP
        )

    controller = locomotion_controller.LocomotionController(
        gait_generator=gait_generator,
        state_estimator=state_estimator,
        swing_leg_controller=sw_controller,
        stance_leg_controller=st_controller,
        curr_time= 0.0
        )
    return controller


def _update_controller_params(controller, lin_speed, ang_speed):
  controller.swing_leg_controller.desired_speed = lin_speed
  controller.swing_leg_controller.desired_twisting_speed = ang_speed
  controller.stance_leg_controller.desired_speed = lin_speed
  controller.stance_leg_controller.desired_twisting_speed = ang_speed