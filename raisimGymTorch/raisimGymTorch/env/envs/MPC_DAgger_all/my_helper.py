import numpy as np
import datetime
import wandb
import torch
import torch.nn as nn
import time
import math
from raisimGymTorch.mpc.mpc_controller_DAgger_all import a1
from raisimGymTorch.mpc.mpc_controller_DAgger_all import gait_generator as gait_generator_lib
from raisimGymTorch.mpc.mpc_controller_DAgger_all.openloop_gait_generator import OpenloopGaitGenerator
from raisimGymTorch.mpc.mpc_controller_DAgger_all import control

SWING = 0
STANCE = 1
# A swing leg that collides with the ground.
EARLY_CONTACT = 2
# A stance leg that loses contact.
LOSE_CONTACT = 3
class MyHelper():
    def __init__(self,env,cfg,saver,rl_obs_dim,rl_act_dim,n_steps,\
                stance_duration,duty_factor,initial_leg_phase,initial_leg_state):
        self.env = env
        self.cfg = cfg
        self.num_env = cfg['environment']['num_envs']
        self.saver = saver
        self.rl_obs_dim = rl_obs_dim
        self.rl_act_dim = rl_act_dim
        self.n_steps = n_steps
        self.u_mpc_mean = np.zeros(rl_act_dim)
        self.u_mpc_var = np.ones(rl_act_dim)
        self.count = 0
        self.ctrl_time_step = self.cfg['environment']['control_dt']        
        self.obs_mean = np.array([ 0.117 , 0.742 ,-1.934, -0.14,   0.74,  -1.923, -0.06,   0.787, -1.882,  0.039, 0.778, -1.872])
        self.obs_var = np.array([0.001, 0.012, 0.034, 0.001, 0.013, 0.039, 0.001, 0.013, 0.035, 0.001, 0.013, 0.04 ])
        self.joint_pos_mean = np.array([ 0.037,  0.304, -0.868, -0.048,  0.301, -0.865, -0.019,  0.316, -0.842,  0.01, 0.32,  -0.853])
        self.joint_pos_var = np.array([0.002, 0.144, 1.131, 0.004, 0.142, 1.14,  0.001, 0.158, 1.082, 0.0003,    0.16,  1.094])
        self.ff_torque_mean = np.array([ 1.804,  1.705,  3.791, -1.81,   1.717,  3.718,  1.888,  2.188,  6.759, -2.026, 1.995,  6.527])
        self.ff_torque_var = np.array([ 4.422,  2.901, 13.736,  4.501,  2.834, 12.734,  2.639,  8.499, 41.766,  3.096, 7.678, 40.002])
        
        self._stance_duration = stance_duration
        self._duty_factor = duty_factor
        self._initial_leg_phase = initial_leg_phase
        self._initial_leg_state = initial_leg_state
        
        self.motor_kps = np.array([a1.ABDUCTION_P_GAIN,a1.ABDUCTION_P_GAIN,a1.ABDUCTION_P_GAIN,a1.ABDUCTION_P_GAIN,
                          a1.HIP_P_GAIN,a1.HIP_P_GAIN,a1.HIP_P_GAIN,a1.HIP_P_GAIN,
                          a1.KNEE_P_GAIN,a1.KNEE_P_GAIN,a1.KNEE_P_GAIN,a1.KNEE_P_GAIN])
        self.motor_kds = np.array([a1.ABDUCTION_D_GAIN,a1.ABDUCTION_D_GAIN,a1.ABDUCTION_D_GAIN,a1.ABDUCTION_D_GAIN,
                                    a1.HIP_D_GAIN,a1.HIP_D_GAIN,a1.HIP_D_GAIN,a1.HIP_D_GAIN,
                                    a1.KNEE_D_GAIN,a1.KNEE_D_GAIN,a1.KNEE_D_GAIN,a1.KNEE_D_GAIN])
        self.motor_kps = np.tile(self.motor_kps,(self.num_env,1))
        self.motor_kds = np.tile(self.motor_kds,(self.num_env,1))

    def get_phase_in_full_cycle(self,current_time):
        full_cycle_period = (np.array(self._stance_duration)/ np.array(self._duty_factor)) # (4,)
        dt_init_phase = np.tile(self._initial_leg_phase * full_cycle_period,(self.num_env,1)) # (num_env,4)
        augmented_time = current_time.reshape(self.num_env,1) + dt_init_phase # (num_env,4)
        full_cycle_period = np.tile(full_cycle_period,(self.num_env,1)) #(num_env,4)
        phase_in_full_cycle = np.fmod(augmented_time,full_cycle_period) / full_cycle_period
        return phase_in_full_cycle
            

    @staticmethod    
    def get_ctrl_obs(env_obs):
        bodyOrientation = env_obs[:,0:4]
        footContacts = env_obs[:,4:8]
        jointPos = env_obs[:,8:20]
        baseLinVel = env_obs[:,20:23]
        baseAngVel = env_obs[:,23:26]
        body_height = env_obs[:,26]
        gravity = env_obs[:,27:30]
        return {'bodyOrientation': bodyOrientation, 'footContacts':footContacts,\
                'jointPos':jointPos,'baseLinVel':baseLinVel,'baseAngVel':baseAngVel,"body_height":body_height,"gravity":gravity}
    @staticmethod 
    def get_rl_obs(env_normed_obs,additional_obs,num_env):
        tmp_obs = env_normed_obs[:,4:27]

        gravity = env_normed_obs[:,27:30]
        normed_gravity = gravity/(np.linalg.norm(gravity,axis=1).reshape(num_env,1))
        body_z_dir = env_normed_obs[:,30:33]
        angle_z = np.diag(np.arccos((-normed_gravity)@(body_z_dir.T))).reshape(num_env,-1)
        tmp_obs = np.append(tmp_obs,angle_z,axis=1)
        
        rl_obs = np.append(tmp_obs,additional_obs,axis = 1)
        return rl_obs.astype(np.float32)
    
    @staticmethod
    def change_foot_contact_range(footContacts):
        footContacts[footContacts==0] = -1    
        return footContacts

    def change_normed_obs(self,raw_obs,perturbed_raw_obs=None):
        if perturbed_raw_obs is not None:
            normed_obs = perturbed_raw_obs.copy()
            footContacts = raw_obs[:,4:8]
            footContacts = self.change_foot_contact_range(footContacts)
            normed_obs[:,4:8] = footContacts
            normed_obs[:,8:20] = (normed_obs[:,8:20]-self.obs_mean)/np.sqrt(self.obs_var)
            return normed_obs    
        normed_obs = raw_obs.copy()
        footContacts = raw_obs[:,4:8].copy()
        footContacts = self.change_foot_contact_range(footContacts)
        normed_obs[:,4:8] = footContacts
        normed_obs[:,8:20] = (normed_obs[:,8:20]-self.obs_mean)/np.sqrt(self.obs_var)
        return normed_obs

    def process_rl_action(self,rl_action):
        processed_rl_action = rl_action.copy()
        processed_rl_action[:,:12] = rl_action[:,:12]*np.sqrt(self.joint_pos_var)+self.joint_pos_mean
        processed_rl_action[:,12:24] = rl_action[:,12:24]*np.sqrt(self.ff_torque_var)+self.ff_torque_mean
        return processed_rl_action.astype(np.float32)
        # # joint pos, ff joint torque, mode
        # rl_action_control = np.zeros(self.num_env,48)
        # rl_action_control[:,:12] = rl_action[:,:12]*np.sqrt(self.joint_pos_var)+self.joint_pos_mean
        # rl_action_control[:,12:24] = rl_action[:,12:24]*np.sqrt(self.ff_torque_var)+self.ff_torque_mean
        # # mode = np.argmax(rl_action[:,-16:],axsis=1)
        # # first leg swing
        # rl_action_control[mode<8,24:27] = kp[:2]  
        # rl_action_control[mode<8,36:39] = kd[:2]
        # # second leg swing
        # rl_action_control[(mode<4)+(8<=mode<12),27:30] = kp[:2]  
        # rl_action_control[(mode<4)+(8<=mode<12),39:42] = kd[:2]
        # # third leg swing
        # rl_action_control[(mode<2)+(4<=mode<6)+(8<=mode<10)+(12<=mode<14),30:33] = kp[:2]  
        # rl_action_control[(mode<2)+(4<=mode<6)+(8<=mode<10)+(12<=mode<14),42:45] = kd[:2]
        # # fourth leg swing
        # rl_action_control[(mode%2==0),33:36] = kp[:2]  
        # rl_action_control[(mode%2==0),45:48] = kd[:2]
        

    def save_model(self,actor,learner,iter):
        print("Saving policy at iter:"+str(iter))
        # save trained model
        torch.save({
            'actor_architecture_state_dict': actor.architecture.state_dict(),
            'actor_distribution_state_dict': actor.distribution.state_dict(),
            'optimizer_state_dict': learner.optimizer.state_dict(),
        }, self.saver.data_dir+"/full_"+str(iter)+'.pt')
        wandb.save(self.saver.data_dir+"/full_"+str(iter)+'.pt')

    def on_visualize(self,update,record=False,istest=False,video_flag = None):
        if self.cfg['environment']['render']:
            self.env.turn_on_visualization()
            if record:
                if istest:
                    vid_name = self.cfg['video_header']+"_test_"+datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") +"policy_"+str(update)+'.mp4'
                else:
                    if video_flag is None:
                        vid_name = self.cfg['video_header']+datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4'
                    else:
                        vid_name = self.cfg['video_header']+video_flag+datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4'
                self.env.start_video_recording(vid_name)

    def off_visualize(self,record=False):
        if self.cfg['environment']['render']:
            self.env.turn_off_visualization()
            if record:
                self.env.stop_video_recording()


    def normalize_target(self,action):
        # update normalization
        normed_action = action.copy()
        normed_action[:,:12] = (action[:,:12]-self.joint_pos_mean)/np.sqrt(self.joint_pos_var)
        normed_action[:,12:24] = (action[:,12:24]-self.ff_torque_mean)/np.sqrt(self.ff_torque_var)
        return normed_action



    def simulate_evaluate(self,update,loaded_graph):
        gait_generator = []
        reset_time = np.zeros((self.num_env,1))
        for i in range(self.num_env):
            gait_generator.append(OpenloopGaitGenerator(                            
                                stance_duration=control._STANCE_DURATION_SECONDS,
                                duty_factor=control._DUTY_FACTOR,
                                initial_leg_phase=control._INIT_PHASE_FULL_CYCLE,
                                initial_leg_state=control._INIT_LEG_STATE)
                                )
        simulation_steps = 2*self.n_steps
        #simulate
        for step in range(simulation_steps):
            with torch.no_grad():
                frame_start = time.time()
                raw_obs,_ = self.env.observe(update_statistics=False,isnoise=False)
                normed_obs = self.change_normed_obs(raw_obs)
                phase_in_full_cycle = self.get_phase_in_full_cycle(self.ctrl_time_step*step-reset_time) # (num_env,4)
                rl_obs = self.get_rl_obs(normed_obs,phase_in_full_cycle,self.num_env) # get rl obs
                for i in range(self.num_env):
                    gait_generator[i].update(self.ctrl_time_step*step-reset_time[i],[False,False,False,False])                

                rl_action = loaded_graph.architecture(torch.from_numpy(rl_obs).cpu()).cpu().detach().numpy()
                rl_action_control = self.process_rl_action(rl_action)
                behavior_action = rl_action_control
                env_act = np.zeros((self.num_env,48))
                env_act[:,:24] = behavior_action
                env_act = self.append_gains(env_act,gait_generator)
                env_act = env_act.astype(np.float32)
                reward, dones = self.env.step(env_act,step)
                for i in range(self.num_env):
                    if dones[i]:
                        reset_time[i] = self.ctrl_time_step*(step+1)
                        gait_generator[i].reset(0.0)
                frame_end = time.time()
                wait_time = self.cfg['environment']['control_dt'] - (frame_end-frame_start)
                if wait_time > 0.:
                    time.sleep(wait_time)

    def append_gains(self,action,gait_generator):
        kps = np.zeros((self.num_env,12))
        kds = np.zeros((self.num_env,12))
        for j in range(self.num_env):
            for leg_id in range(4):
                if gait_generator[j].desired_leg_state[leg_id] == gait_generator_lib.LegState.SWING:
                    kps[j,3*leg_id:3*(leg_id+1)] = [a1.ABDUCTION_P_GAIN,a1.HIP_P_GAIN,a1.KNEE_P_GAIN]
                    kds[j,3*leg_id:3*(leg_id+1)] = [a1.ABDUCTION_D_GAIN,a1.HIP_D_GAIN,a1.KNEE_D_GAIN]
        action[:,24:36] = kps
        action[:,36:] = kds
        return action
    
    def append_gains_vec(self,action,gait_generator): # (num_env, num_leg)
        kps = np.zeros((self.num_env,12))
        kds = np.zeros((self.num_env,12))
        desired_leg_state = np.tile(gait_generator.desired_leg_state,(1,3))
        kps[desired_leg_state == SWING] = self.motor_kps[desired_leg_state == SWING]
        kds[desired_leg_state == SWING] = self.motor_kds[desired_leg_state == SWING]
        action[:,24:27] = kps[:,:12:4]
        action[:,27:30] = kps[:,1:12:4]
        action[:,30:33] = kps[:,2:12:4]
        action[:,33:36] = kps[:,3:12:4]
        
        action[:,36:39] = kds[:,:12:4]
        action[:,39:42] = kds[:,1:12:4]
        action[:,42:45] = kds[:,2:12:4]
        action[:,45:48] = kds[:,3:12:4]
        
        return action