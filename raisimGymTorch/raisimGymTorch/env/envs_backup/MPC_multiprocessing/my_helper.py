import numpy as np
import datetime
import wandb
import torch
import torch.nn as nn
from raisimGymTorch.algo.DAgger import module
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
                
    def save_model(self,actor,learner,iter):
        print("Saving policy at iter:"+str(iter))
        # save trained model
        torch.save({
            'actor_architecture_state_dict': actor.architecture.state_dict(),
            'actor_distribution_state_dict': actor.distribution.state_dict(),
            'optimizer_state_dict': learner.optimizer.state_dict(),
        }, self.saver.data_dir+"/full_"+str(iter)+'.pt')
        wandb.save(self.saver.data_dir+"/full_"+str(iter)+'.pt')

    def on_visualize(self,update,istest=True,video_flag = None):
        if self.cfg['environment']['render']:
            self.env.turn_on_visualization()
            if self.cfg['record_video'] == 'yes':
                if istest:
                    vid_name = self.cfg['video_header']+"_test_"+datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") +"policy_"+str(update)+'.mp4'
                else:
                    if video_flag is None:
                        vid_name = self.cfg['video_header']+datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4'
                    else:
                        vid_name = self.cfg['video_header']+video_flag+datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + "policy_"+str(update)+'.mp4'
                self.env.start_video_recording(vid_name)

    def off_visualize(self):
        if self.cfg['environment']['render']:
            self.env.turn_off_visualization()
            if self.cfg['record_video'] == 'yes':
                self.env.stop_video_recording()


    def normalize_target(self,u_mpc):
        # update normalization
        return (u_mpc-self.act_mean)/np.sqrt(self.act_var)


    def unnormalize(self,u_rl):
        return u_rl*np.sqrt(self.act_var)+self.act_mean