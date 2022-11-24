from raisimGymTorch.mpc.mpc_controller import com_velocity_estimator
from raisimGymTorch.mpc.mpc_controller import gait_generator as gait_generator_lib
from raisimGymTorch.mpc.mpc_controller import locomotion_controller
from raisimGymTorch.mpc.mpc_controller import openloop_gait_generator
from raisimGymTorch.mpc.mpc_controller import raibert_swing_leg_controller
from raisimGymTorch.mpc.mpc_controller import torque_stance_leg_controller_quadprog as torque_stance_leg_controller
from raisimGymTorch.mpc.mpc_controller import a1
import scipy.interpolate
import numpy as np
import threading

# CONTROL CONSTANTS

# Output indexing
POSITION_INDEX = 0
POSITION_GAIN_INDEX = 1
VELOCITY_INDEX = 2
VELOCITY_GAIN_INDEX = 3
TORQUE_INDEX = 4
MOTOR_COMMAND_DIMENSION = 5

# Walking constants. It goes to gait generator
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
FOOT_CLEARANCE = 0.01

# controller that can be used in runner file.
class Controller(object):
    '''
    Whole body control.
    It setup each environmental controllers.
    It has functions liek control using RL actions, control using MPC.
    '''
    def __init__(
        self,
        init_joint,
        ctrl_time_step,
        ctrl_act_dim,
        num_env
    ):
        self.ctrl_time_step = ctrl_time_step
        self.num_env = num_env
        self.ctrl_act_dim = ctrl_act_dim
        self.controller = []
        for i in range(self.num_env):
            self.controller.append(self._setup_controller())
            self.reset(0.0,init_joint,i)
        
    def _setup_controller(self):
        
        init_desired_speed = (0, 0)
        init_desired_twisting_speed = 0

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
            desired_speed=init_desired_speed,
            desired_twisting_speed=init_desired_twisting_speed,
            desired_height=MPC_BODY_HEIGHT,
            foot_clearance=FOOT_CLEARANCE)
        st_controller = torque_stance_leg_controller.TorqueStanceLegController(
            gait_generator,
            state_estimator,
            desired_speed=init_desired_speed,
            desired_twisting_speed=init_desired_twisting_speed,
            desired_body_height=MPC_BODY_HEIGHT
            )
        controller = locomotion_controller.LocomotionController(
            gait_generator=gait_generator,
            state_estimator=state_estimator,
            swing_leg_controller=sw_controller,
            stance_leg_controller=st_controller,
            curr_time= 0.0
            )
        return controller

    def reset(self,reset_time,init_joint,env_num):
        self.controller[env_num].reset(reset_time,init_joint[env_num])
                        
    def _update_controller_params(self,controller,lin_speed, ang_speed):
        controller.swing_leg_controller.desired_speed = lin_speed
        controller.swing_leg_controller.desired_twisting_speed = ang_speed
        controller.stance_leg_controller.desired_speed = lin_speed
        controller.stance_leg_controller.desired_twisting_speed = ang_speed
        
    def _generate_example_linear_angular_speed(self,t):
        """Creates an example speed profile based on time for demo purpose."""
        vx = 0.6
        vy = 0.2
        wz = 0.8

        # time_points = (0, 5, 10, 15, 20, 25,30)
        # speed_points = ((0, 0, 0, 0), (0, 0, 0, wz), (vx, 0, 0, 0), (0, 0, 0, -wz), (0, -vy, 0, 0),
        #                 (0, 0, 0, 0), (0, 0, 0, wz))
        time_points = (0,5)
        speed_points = ((vx,0,0,0),(vx,0,0,0))
        speed = scipy.interpolate.interp1d(time_points,
                                            speed_points,
                                            kind="previous",
                                            fill_value="extrapolate",
                                            axis=0)(t)

        return speed[0:3], speed[3], False

    def get_mpc_contact_force(self, ctrl_obs):
        contact_forces = []
        for i in range(self.num_env):
            indiv_obs = {}
            for key in ctrl_obs:
                indiv_obs[key] = ctrl_obs[key][i,:]
            _,contact_force = self.controller[i].get_stance_leg_action(indiv_obs)
            contact_forces.append(contact_force)
        return np.array(contact_forces)


    def control_with_conactForce(self,action,ctrl_obs,step):
        env_act = np.zeros((self.num_env,self.ctrl_act_dim))
        lin_speed, ang_speed, e_stop = self._generate_example_linear_angular_speed(self.ctrl_time_step*step)
        
        # TODO: parallelize
        for i in range(self.num_env):
            indiv_obs = {}
            for key in ctrl_obs:
                indiv_obs[key] = ctrl_obs[key][i,:]
            self._update_controller_params(self.controller[i],lin_speed,ang_speed)
            self.controller[i].update(self.ctrl_time_step*step,indiv_obs)
            action_tmp = self.controller[i].get_action_with_contactForce(action[i],indiv_obs)
            pos_action = action_tmp[POSITION_INDEX::MOTOR_COMMAND_DIMENSION].copy()
            tq_action = action_tmp[TORQUE_INDEX::MOTOR_COMMAND_DIMENSION].copy()
            kp_action = action_tmp[POSITION_GAIN_INDEX::MOTOR_COMMAND_DIMENSION].copy()
            kd_action = action_tmp[VELOCITY_GAIN_INDEX::MOTOR_COMMAND_DIMENSION].copy()
            env_act[i] = np.append(np.append(pos_action,tq_action),np.append(kp_action,kd_action))
        return env_act.astype(np.float32)

    def get_desired_leg_state(self):
        desired_leg_state = []
        for i in range(self.num_env):
            desired_leg_state.append([desired_leg_state.value for desired_leg_state in self.controller[i]._gait_generator.desired_leg_state])
        return np.array(desired_leg_state)