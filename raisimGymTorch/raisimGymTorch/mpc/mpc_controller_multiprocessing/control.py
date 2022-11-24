from raisimGymTorch.mpc.mpc_controller_multiprocessing import com_velocity_estimator
from raisimGymTorch.mpc.mpc_controller_multiprocessing import gait_generator as gait_generator_lib
from raisimGymTorch.mpc.mpc_controller_multiprocessing import locomotion_controller
from raisimGymTorch.mpc.mpc_controller_multiprocessing import openloop_gait_generator
from raisimGymTorch.mpc.mpc_controller_multiprocessing import raibert_swing_leg_controller
from raisimGymTorch.mpc.mpc_controller_multiprocessing import torque_stance_leg_controller_quadprog as torque_stance_leg_controller
from raisimGymTorch.mpc.mpc_controller_multiprocessing import a1
import scipy.interpolate
import numpy as np
from multiprocessing import Process, Pool, Value, Array, Queue
from concurrent.futures import ProcessPoolExecutor
from multiprocessing import Process,shared_memory
import os
import sys
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


    # def control_with_conactForce(self,action,ctrl_obs,step):       
    #     # TODO: implement with process not pool
    #     self.action = action
    #     self.ctrl_obs = ctrl_obs
    #     self.step = step
    #     env_act = np.zeros((self.num_env,self.ctrl_act_dim),dtype=np.float32)
    #     with ProcessPoolExecutor() as executor:
    #         results = executor.map(self.control_indv,list(range(self.num_env)))
    #         i=0
    #         for result in results:
    #             env_act[i], self.controller[i] = result
    #             i+=1
    #     return env_act.astype(np.float32)

    # def control_indv(self,env_num):
    #     lin_speed, ang_speed, e_stop = self._generate_example_linear_angular_speed(self.ctrl_time_step*self.step)
    #     indiv_obs = {}
    #     for key in self.ctrl_obs:
    #         indiv_obs[key] = self.ctrl_obs[key][env_num,:]
    #     self._update_controller_params(self.controller[env_num],lin_speed,ang_speed)
    #     self.controller[env_num].update(self.ctrl_time_step*self.step,indiv_obs)
    #     action_tmp = self.controller[env_num].get_action_with_contactForce(self.action[env_num],indiv_obs)
    #     pos_action = action_tmp[POSITION_INDEX::MOTOR_COMMAND_DIMENSION].copy()
    #     tq_action = action_tmp[TORQUE_INDEX::MOTOR_COMMAND_DIMENSION].copy()
    #     kp_action = action_tmp[POSITION_GAIN_INDEX::MOTOR_COMMAND_DIMENSION].copy()
    #     kd_action = action_tmp[VELOCITY_GAIN_INDEX::MOTOR_COMMAND_DIMENSION].copy()
    #     env_act = np.append(np.append(pos_action,tq_action),np.append(kp_action,kd_action))
    #     return env_act.astype(np.float32), self.controller[env_num]

    def control_with_conactForce(self,action,ctrl_obs,step):       
        shr_q = Queue()
        env_act = np.zeros((self.num_env,self.ctrl_act_dim),dtype=np.float32)
        processes = []
        self.num_processes = os.cpu_count()
        for i in range(self.num_processes):
            processes.append(Process(target=self.control_indv,args=(shr_q,action,ctrl_obs,step,i)))
        for process in processes:
            process.start()
        for process in processes:
            process.join()
        # while True:
        #     item = shr_q.get()
        #     if item is None:
        #         break
        #     process_num = item["process_num"]
        #     st_env = process_num*int(self.num_env/self.num_processes)
        #     end_env = (process_num+1)*int(self.num_env/self.num_processes)
        #     if end_env > self.num_env:
        #         end_env = self.num_env
        #     env_act[st_env:end_env,:] = item["proc_env_act"]
        #     self.controller[st_env:end_env] = item["controller"]                
        return env_act.astype(np.float32)

    def control_indv(self,shr_q,action,ctrl_obs,step,process_num):
        st_env = process_num*int(self.num_env/self.num_processes)
        end_env = (process_num+1)*int(self.num_env/self.num_processes)
        if end_env > self.num_env:
            end_env = self.num_env
        proc_env_act = np.zeros((end_env-st_env,self.ctrl_act_dim),dtype=np.float32)
        lin_speed, ang_speed, e_stop = self._generate_example_linear_angular_speed(self.ctrl_time_step*step)
        for i in range(st_env,end_env):
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
            proc_env_act[i-st_env] = np.append(np.append(pos_action,tq_action),np.append(kp_action,kd_action))
        # ret = {"proc_env_act":proc_env_act,"controller":self.controller[st_env:end_env],"process_num":process_num}
        # shr_q.put(ret)



    def get_desired_leg_state(self):
        desired_leg_state = []
        for i in range(self.num_env):
            desired_leg_state.append([desired_leg_state.value for desired_leg_state in self.controller[i]._gait_generator.desired_leg_state])
        return np.array(desired_leg_state)