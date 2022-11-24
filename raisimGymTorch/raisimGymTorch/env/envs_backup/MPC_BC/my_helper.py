import numpy as np
import datetime
import wandb
import torch
import torch.nn as nn
import raisimGymTorch.algo.ppo_BC.module as ppo_module
import raisimGymTorch.algo.ppo_BC.ppo_BC as PPO_BC
from raisimGymTorch.mpc.mpc_controller_BC.control import Controller
import time

class MyHelper():
    def __init__(self,env,cfg,saver,rl_obs_dim,rl_act_dim,n_steps):
        self.env = env
        self.cfg = cfg
        self.saver = saver
        self.rl_obs_dim = rl_obs_dim
        self.rl_act_dim = rl_act_dim
        self.n_steps = n_steps
        self.u_mpc_mean = np.zeros(rl_act_dim)
        self.u_mpc_var = np.ones(rl_act_dim)
        self.count = 0
        self.obs_mean=np.tile(np.array([0.12380544,0.753141,-1.9323473]),4)
        self.obs_var = np.tile(np.array([0.001999718,0.01403143,0.03615589]),4)
        self.act_mean = np.array([-3.0614530461775438,3.1071043893296597,-19.698664678480903,\
                                    -3.4226090251468557,-3.342510458450658,-18.888233912780667,\
                                    -3.830589579668038,-7.804019324480523,-37.52501856236166,\
                                    -2.712210106944327,7.040482212117329,-37.55473461099767])
        self.act_var = np.array([13.961287514495538,23.320416952562294,390.5204702231471,\
                                16.764116444284397,28.865088484998886,342.7677544550194,\
                                78.2099044333424,111.63711485882727,1235.9530802357692,\
                                50.72154971575431,95.88191676888425,1217.7261942768607])
        self.ctrl_time_step = self.cfg['environment']['control_dt']
        self.ctrl_act_dim = 12*4 # joint pos, joint torque, KP, KD.
        self.num_env = self.cfg['environment']['num_envs']

    @staticmethod    
    def get_ctrl_obs(env_obs):
        bodyOrientation = env_obs[:,0:4]
        footContacts = env_obs[:,4:8]
        jointPos = env_obs[:,8:20]
        baseLinVel = env_obs[:,20:23]
        baseAngVel = env_obs[:,23:26]
        return {'bodyOrientation': bodyOrientation, 'footContacts':footContacts,\
                'jointPos':jointPos,'baseLinVel':baseLinVel,'baseAngVel':baseAngVel}

    @staticmethod
    def get_rl_obs(env_normed_obs,additional_obs):
        rl_obs = np.append(env_normed_obs,additional_obs,axis = 1)
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
        footContacts = raw_obs[:,4:8]
        footContacts = self.change_foot_contact_range(footContacts)
        normed_obs[:,4:8] = footContacts
        normed_obs[:,8:20] = (normed_obs[:,8:20]-self.obs_mean)/np.sqrt(self.obs_var)
        return normed_obs
                
    def save_model(self,actor,critic,ppo,update):
        torch.save({
            'actor_architecture_state_dict': actor.architecture.state_dict(),
            'actor_distribution_state_dict': actor.distribution.state_dict(),
            'critic_architecture_state_dict': critic.architecture.state_dict(),
            'optimizer_state_dict': ppo.optimizer.state_dict(),
        }, self.saver.data_dir+"/full_"+str(update)+'.pt')
        wandb.save(self.saver.data_dir+"/full_"+str(update)+'.pt')


    def on_visualize(self,record,update,istest=True,video_flag = None):
        if self.cfg['environment']['render']:
            self.env.turn_on_visualization()
            if record:
                if istest:
                    vid_name = self.cfg['video_header']+"_test_"+datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") +"policy_"+str(update)+'.mp4'
                else:
                    if video_flag is None:
                        vid_name = self.cfg['video_header']+"_eval_"+datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4'
                    else:
                        vid_name = self.cfg['video_header']+video_flag+datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4'
                self.env.start_video_recording(vid_name)

    def off_visualize(self,record):
        if self.cfg['environment']['render']:
            self.env.turn_off_visualization()
            if record:
                self.env.stop_video_recording()

    def normalize_target(self,u_mpc):
        # update normalization
        return (u_mpc-self.act_mean)/np.sqrt(self.act_var)


    def unnormalize(self,u_rl):
        return u_rl*np.sqrt(self.act_var)+self.act_mean        

    def reset_env_controller(self):
        # reset env and set controller before simulate
        self.env.reset()
        init_raw_obs, _ = self.env.observe(update_statistics=False, isnoise=False)
        init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
        init_joint_angle = init_ctrl_obs['jointPos']
        return Controller(init_joint_angle,self.ctrl_time_step,self.ctrl_act_dim,self.num_env)
    
    def reset_controller(self,i,step,controller):
        init_raw_obs, _ = self.env.observe(update_statistics=False, isnoise=False)
        init_ctrl_obs = MyHelper.get_ctrl_obs(init_raw_obs)
        init_joint_angle = init_ctrl_obs['jointPos']
        controller.reset(self.cfg['environment']['control_dt']*(step+1),init_joint_angle,i)

    def simulate_evaluate(self,update,controller):
        # we create another graph just to demonstrate the save/load method
        loaded_graph = ppo_module.MLP(self.cfg['architecture']['policy_net'], nn.LeakyReLU, self.rl_obs_dim, self.rl_act_dim)
        loaded_graph.load_state_dict(torch.load(self.saver.data_dir+"/full_"+str(update)+'.pt')['actor_architecture_state_dict'])
        #simulate
        for step in range(self.n_steps):
            with torch.no_grad():
                frame_start = time.time()
                raw_obs,_ = self.env.observe(update_statistics=False,isnoise=False)
                normed_obs = self.change_normed_obs(raw_obs)
                desired_leg_state = MyHelper.change_foot_contact_range(controller.get_desired_leg_state()) # get desired leg state for rl obs
                rl_obs = MyHelper.get_rl_obs(normed_obs,desired_leg_state) # get rl obs
                ctrl_obs = MyHelper.get_ctrl_obs(raw_obs)

                u_rl = loaded_graph.architecture(torch.from_numpy(rl_obs).cpu()).cpu().detach().numpy()
                u_rl_unnormalized = self.unnormalize(u_rl)

                u_rl_contact_forces_normed = {"(eval_normed)RL_FRx":u_rl[0][0],"(eval_normed)RL_FRy":u_rl[0][1],"(eval_normed)RL_FRz":u_rl[0][2], \
                    "(eval_normed)RL_FLx":u_rl[0][3],"(eval_normed)RL_FLy":u_rl[0][4],"(eval_normed)RL_FLz":u_rl[0][5],\
                    "(eval_normed)RL_RRx":u_rl[0][6],"(eval_normed)RL_RRy":u_rl[0][7],"(eval_normed)RL_RRz":u_rl[0][8],\
                    "(eval_normed)RL_RLx":u_rl[0][9],"(eval_normed)RL_RLy":u_rl[0][10],"(eval_normed)RL_RLz":u_rl[0][11]}
                u_rl_contact_forces = {"(eval)RL_FRx":u_rl_unnormalized[0][0],"(eval)RL_FRy":u_rl_unnormalized[0][1],"(eval)RL_FRz":u_rl_unnormalized[0][2], \
                    "(eval)RL_FLx":u_rl_unnormalized[0][3],"(eval)RL_FLy":u_rl_unnormalized[0][4],"(eval)RL_FLz":u_rl_unnormalized[0][5],\
                    "(eval)RL_RRx":u_rl_unnormalized[0][6],"(eval)RL_RRy":u_rl_unnormalized[0][7],"(eval)RL_RRz":u_rl_unnormalized[0][8],\
                    "(eval)RL_RLx":u_rl_unnormalized[0][9],"(eval)RL_RLy":u_rl_unnormalized[0][10],"(eval)RL_RLz":u_rl_unnormalized[0][11]}

                
                u_rl_unnormalized = u_rl_unnormalized.reshape(-1,4,3)
                env_act = controller.control_with_conactForce(u_rl_unnormalized,ctrl_obs,step)
                reward, dones = self.env.step(env_act,step)
                for i in range(self.num_env):
                    if dones[i]:
                        init_raw_obs, _ = self.env.observe(update_statistics=False, isnoise=False)
                        init_ctrl_obs = self.get_ctrl_obs(init_raw_obs)
                        init_joint_angle = init_ctrl_obs['jointPos']
                        controller.reset(self.cfg['environment']['control_dt']*(step+1),init_joint_angle,i)
                frame_end = time.time()
                wait_time = self.cfg['environment']['control_dt'] - (frame_end-frame_start)
                if wait_time > 0.:
                    time.sleep(wait_time)
                if step%10 ==0:
                    ## collect performance info
                    step_data = {"step":step+update*self.n_steps}
                    # rewards
                    reward_data = {}
                    rewardinfo = self.env.getRewardinfo()
                    for i in range(len(rewardinfo)):
                        for key in rewardinfo[i]:
                            if key in reward_data:
                                reward_data[key].append(rewardinfo[i][key])
                            else:
                                reward_data[key] = []
                    for key in reward_data:
                        step_data[key+"(reward)"] = np.mean((np.array(reward_data[key])))
                    # velocities
                    step_data["forward_velocity"] = np.mean(ctrl_obs["baseLinVel"][:,0])
                    # foot contact forces
                    for key in u_rl_contact_forces_normed:
                        step_data[key] = u_rl_contact_forces_normed[key]
                    for key in u_rl_contact_forces:
                        step_data[key] = u_rl_contact_forces[key]
                    # log stepwise data
                    wandb.log(step_data)
    
    def compare_with_mpc(self,update,controller):         
        # we create another graph just to demonstrate the save/load method
        loaded_graph = ppo_module.MLP(self.cfg['architecture']['policy_net'], nn.LeakyReLU, self.rl_obs_dim, self.rl_act_dim)
        loaded_graph.load_state_dict(torch.load(self.saver.data_dir+"/full_"+str(update)+'.pt')['actor_architecture_state_dict'])
        #simulate
        for step in range(self.n_steps):
            with torch.no_grad():
                frame_start = time.time()
                raw_obs,_ = self.env.observe(update_statistics=False,isnoise=False)
                normed_obs = self.change_normed_obs(raw_obs)
                desired_leg_state = MyHelper.change_foot_contact_range(controller.get_desired_leg_state()) # get desired leg state for rl obs
                rl_obs = MyHelper.get_rl_obs(normed_obs,desired_leg_state) # get rl obs
                ctrl_obs = MyHelper.get_ctrl_obs(raw_obs)

                u_mpc = controller.get_mpc_contact_force(ctrl_obs)
                env_act = controller.control_with_conactForce(u_mpc,ctrl_obs,step)
                reward, dones = self.env.step(env_act,step)

                u_rl = loaded_graph.architecture(torch.from_numpy(rl_obs).cpu()).cpu().detach().numpy()
                u_mpc_normed = self.normalize_target(u_mpc.reshape(-1,self.rl_act_dim))

                u_rl_contact_forces_normed = {"RL_FRx":u_rl[0][0],"RL_FRy":u_rl[0][1],"RL_FRz":u_rl[0][2], \
                    "RL_FLx":u_rl[0][3],"RL_FLy":u_rl[0][4],"RL_FLz":u_rl[0][5],\
                    "RL_RRx":u_rl[0][6],"RL_RRy":u_rl[0][7],"RL_RRz":u_rl[0][8],\
                    "RL_RLx":u_rl[0][9],"RL_RLy":u_rl[0][10],"RL_RLz":u_rl[0][11]}
                u_mpc_contact_forces = {"FRx":u_mpc_normed[0][0],"FRy":u_mpc_normed[0][1],"FRz":u_mpc_normed[0][2], \
                    "FLx":u_mpc_normed[0][3],"FLy":u_mpc_normed[0][4],"FLz":u_mpc_normed[0][5],\
                    "RRx":u_mpc_normed[0][6],"RRy":u_mpc_normed[0][7],"RRz":u_mpc_normed[0][8],\
                    "RLx":u_mpc_normed[0][9],"RLy":u_mpc_normed[0][10],"RLz":u_mpc_normed[0][11]}

                for i in range(self.num_env):
                    if dones[i]:
                        init_raw_obs, _ = self.env.observe(update_statistics=False, isnoise=False)
                        init_ctrl_obs = self.get_ctrl_obs(init_raw_obs)
                        init_joint_angle = init_ctrl_obs['jointPos']
                        controller.reset(self.cfg['environment']['control_dt']*(step+1),init_joint_angle,i)

                frame_end = time.time()
                wait_time = self.cfg['environment']['control_dt'] - (frame_end-frame_start)
                if wait_time > 0.:
                    time.sleep(wait_time)
                if step%10==0:
                    ## collect performance info
                    step_data = {"step":step+update*self.n_steps}
                    # foot contact forces
                    for key in u_rl_contact_forces_normed:
                        step_data[key] = u_rl_contact_forces_normed[key]
                    for key in u_mpc_contact_forces:
                        step_data[key] = u_mpc_contact_forces[key]
                    step_data["imitation_loss(eval)"] = np.mean((u_mpc_normed-u_rl)**2)
                    # log stepwise data
                    wandb.log(step_data)