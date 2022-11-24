from raisimGymTorch.mpc_BC.mpc_controller import com_velocity_estimator
from raisimGymTorch.mpc_BC.mpc_controller import gait_generator as gait_generator_lib
from raisimGymTorch.mpc_BC.mpc_controller import locomotion_controller
from raisimGymTorch.mpc_BC.mpc_controller import openloop_gait_generator
from raisimGymTorch.mpc_BC.mpc_controller import raibert_swing_leg_controller
from raisimGymTorch.mpc_BC.mpc_controller import torque_stance_leg_controller_quadprog as torque_stance_leg_controller
import scipy.interpolate
import numpy as np
from raisimGymTorch.mpc_BC import robot_helper
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


# controller that can be used in runner file.
class Controller(object):
    def __init__(
        self,
        init_obs,
        ctr_time_step,
        control_act_dim,
        num_env
    ):
        self.ctr_time_step = ctr_time_step
        self.num_env = num_env
        self.control_act_dim = control_act_dim
        self.controller = []

        for i in range(num_env):
            joint_pos_init_obs = init_obs[i,8:20] # joint position
            self.controller.append(self._setup_controller())
            self.controller[i].reset(0.0,joint_pos_init_obs)
            
    def _setup_controller(self):
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
            )

        controller = locomotion_controller.LocomotionController(
            gait_generator=gait_generator,
            state_estimator=state_estimator,
            swing_leg_controller=sw_controller,
            stance_leg_controller=st_controller,
            curr_time= 0.0
            )
        return controller
            
    def _update_controller_params(self,controller,lin_speed, ang_speed):
        controller.swing_leg_controller.desired_speed = lin_speed
        controller.swing_leg_controller.desired_twisting_speed = ang_speed
        controller.stance_leg_controller.desired_speed = lin_speed
        controller.stance_leg_controller.desired_twisting_speed = ang_speed
        
    def control_with_action(self,action,control_obs,step):
        simulate_control_act = np.zeros((self.num_env,self.control_act_dim))
        lin_speed, ang_speed, e_stop = self._generate_example_linear_angular_speed(self.ctr_time_step*step)
        for i in range(self.num_env):
            self._update_controller_params(self.controller[i],lin_speed,ang_speed)
            self.controller[i].update(self.ctr_time_step*step,control_obs[i])
            action_tmp = self.controller[i].get_control_action(action[i],control_obs[i])
            pos_action = action_tmp[POSITION_INDEX::MOTOR_COMMAND_DIMENSION].copy()
            tq_action = action_tmp[TORQUE_INDEX::MOTOR_COMMAND_DIMENSION].copy()
            kp_action = action_tmp[POSITION_GAIN_INDEX::MOTOR_COMMAND_DIMENSION].copy()
            kd_action = action_tmp[VELOCITY_GAIN_INDEX::MOTOR_COMMAND_DIMENSION].copy()
            simulate_control_act[i] = np.append(np.append(pos_action,tq_action),np.append(kp_action,kd_action))
        return simulate_control_act

    def control(self,control_obs,step):        
        # multi threads
        contact_force = []
        ctrl_act = []
        for i in range(self.num_env):
            contact_force.append(np.zeros((4,3)))
            ctrl_act.append(np.zeros((self.control_act_dim)))
        threads = []
        for i in range(self.num_env):
            threads.append(threading.Thread(target = self.control_individual,\
                            args=(control_obs,contact_force,ctrl_act,step,i)))    
        for i in range(self.num_env):
            threads[i].start()
        print(threading.active_count())
        for i in range(self.num_env):
            threads[i].join()
        return np.array(contact_force), np.array(ctrl_act)

        # single thread
        contact_force = []
        ctrl_act = np.zeros((self.num_env,self.control_act_dim))
        lin_speed, ang_speed, e_stop = self._generate_example_linear_angular_speed(self.ctr_time_step*step)
        for i in range(self.num_env):
            self._update_controller_params(self.controller[i],lin_speed,ang_speed)
            contact_force.append(self.controller[i].get_contact_force(control_obs[i]))
            self.controller[i].update(self.ctr_time_step*step,control_obs[i])
            action_tmp = self.controller[i].get_control_action(contact_force[i],control_obs[i])
            pos_action = action_tmp[POSITION_INDEX::MOTOR_COMMAND_DIMENSION].copy()
            tq_action = action_tmp[TORQUE_INDEX::MOTOR_COMMAND_DIMENSION].copy()
            kp_action = action_tmp[POSITION_GAIN_INDEX::MOTOR_COMMAND_DIMENSION].copy()
            kd_action = action_tmp[VELOCITY_GAIN_INDEX::MOTOR_COMMAND_DIMENSION].copy()
            ctrl_act[i] = np.append(np.append(pos_action,tq_action),np.append(kp_action,kd_action))

        return np.array(contact_force), ctrl_act
    
    def control_individual(self,ctrl_obs,contact_force,ctrl_act,step,idx):
        lin_speed, ang_speed, e_stop = self._generate_example_linear_angular_speed(self.ctr_time_step*step)
        self._update_controller_params(self.controller[idx],lin_speed,ang_speed)
        contact_force[idx] = self.controller[idx].get_contact_force(ctrl_obs[idx])
        self.controller[idx].update(self.ctr_time_step*step,ctrl_obs[idx])
        action_tmp = self.controller[idx].get_control_action(contact_force[idx],ctrl_obs[idx])
        pos_action = action_tmp[POSITION_INDEX::MOTOR_COMMAND_DIMENSION].copy()
        tq_action = action_tmp[TORQUE_INDEX::MOTOR_COMMAND_DIMENSION].copy()
        kp_action = action_tmp[POSITION_GAIN_INDEX::MOTOR_COMMAND_DIMENSION].copy()
        kd_action = action_tmp[VELOCITY_GAIN_INDEX::MOTOR_COMMAND_DIMENSION].copy()
        ctrl_act[idx] = np.append(np.append(pos_action,tq_action),np.append(kp_action,kd_action))
        # print(idx)

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

    def reset(self,time,init_joint_pos_obs,env_idx):
        self.controller[env_idx].reset(time,init_joint_pos_obs)
