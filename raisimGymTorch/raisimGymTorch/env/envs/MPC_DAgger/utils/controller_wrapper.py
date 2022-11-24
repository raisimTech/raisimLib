from mpc_controller import com_velocity_estimator
from mpc_controller import gait_generator as gait_generator_lib
from mpc_controller import locomotion_controller
from mpc_controller import openloop_gait_generator
from mpc_controller import raibert_swing_leg_controller
from mpc_controller import torque_stance_leg_controller_quadprog as torque_stance_leg_controller

###############################################################################
# TODO: Make it to class
###############################################################################
class Controller_wrapper:
    def __init__(self):
        self._STANCE_DURATION_SECONDS = [
            0.3
        ] * 4  # For faster trotting (v > 1.5 ms reduce this to 0.13s).
        self._DUTY_FACTOR = [0.6] * 4
        self._INIT_PHASE_FULL_CYCLE = [0.9, 0, 0, 0.9]
        self._INIT_LEG_STATE = (
            gait_generator_lib.LegState.SWING,
            gait_generator_lib.LegState.STANCE,
            gait_generator_lib.LegState.STANCE,
            gait_generator_lib.LegState.SWING,
        )

    def _setup_controller(self):
        """Demonstrates how to create a locomotion controller."""
        desired_speed = (0, 0)
        desired_twisting_speed = 0

        self.gait_generator = openloop_gait_generator.OpenloopGaitGenerator(
            stance_duration=self._STANCE_DURATION_SECONDS,
            duty_factor=self._DUTY_FACTOR,
            initial_leg_phase=self._INIT_PHASE_FULL_CYCLE,
            initial_leg_state=self._INIT_LEG_STATE)
        window_size = 20
        self.state_estimator = com_velocity_estimator.COMVelocityEstimator(
            window_size=window_size)
        self.sw_controller = raibert_swing_leg_controller.RaibertSwingLegController(
            self.gait_generator,
            self.state_estimator,
            desired_speed=desired_speed,
            desired_twisting_speed=desired_twisting_speed,
            desired_height=robot.MPC_BODY_HEIGHT,
            foot_clearance=0.01)

        self.st_controller = torque_stance_leg_controller.TorqueStanceLegController(
            robot,
            self.gait_generator,
            self.state_estimator,
            desired_speed=desired_speed,
            desired_twisting_speed=desired_twisting_speed,
            desired_body_height=robot.MPC_BODY_HEIGHT
            #,qp_solver = mpc_osqp.QPOASES #or mpc_osqp.OSQP
            )

        self.controller = locomotion_controller.LocomotionController(
            robot=robot,
            gait_generator=self.gait_generator,
            state_estimator=self.state_estimator,
            swing_leg_controller=self.sw_controller,
            stance_leg_controller=self.st_controller,
            clock=robot.GetTimeSinceReset)

    def _update_controller_params(self, lin_speed, ang_speed):
        self.controller.swing_leg_controller.desired_speed = lin_speed
        self.controller.swing_leg_controller.desired_twisting_speed = ang_speed
        self.controller.stance_leg_controller.desired_speed = lin_speed
        self.controller.stance_leg_controller.desired_twisting_speed = ang_speed
    
    def reset(self):
        self.controller.reset()