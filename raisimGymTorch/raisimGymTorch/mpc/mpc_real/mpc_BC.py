# 1. in for loop, get data
    # 2. process action data(action normalization)
    # 3. stack data(stack normalized action)
# 4. process obs data(obs normalization)  -> or should it be normalized by mean, var csv data?
if MPC_imitation:
    traj_size = 3000
    num_traj = 200
    whole_mpc_obs = [] # whole observation tarjectory. should be (# of traj, size of traj, obs_dim)
    whole_mpc_acts = [] # whole action tarjectory (# of traj, size of traj, act_dim+12(kps))
    for i in range(num_traj):
        # get data
        acts_name = "/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/mpc/mpc_data_traj_frequencyMatch/action_traj" \
                            + str(i)+".csv"
        obs_name = "/home/hyunyoungjung/MPC-RL/raisim/raisim_ws/raisimLib/mpc/mpc_data_traj_frequencyMatch/observation_traj"\
                            + str(i)+".csv"
        act_mpc = np.loadtxt(acts_name,dtype=np.float32)
        obs_mpc = np.loadtxt(obs_name,dtype=np.float32)
        # normalize action
        mpc_kp = act_mpc[:,24:].copy()
        mpc_kp_mask = (mpc_kp==0) 

        initial_pos = np.array([0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8])
        initial_pos = np.tile(initial_pos,(traj_size,1))
        initial_pos[mpc_kp_mask] = 0
        
        act_pos = act_mpc[:,:12].copy()
        act_pos[mpc_kp_mask] = 0
        act_pos = (act_pos - initial_pos)/0.3

        act_torque = act_mpc[:,12:24].copy()
        act_torque = act_torque/5

        act_mpc_norm = np.hstack((act_pos,act_torque)) # (size of traj, action_dimension)
        assert act_mpc_norm.shape[0] == traj_size, "Wrong traj size!"

        # stack data
        whole_mpc_obs.append(obs_mpc)
        whole_mpc_acts.append(act_mpc_norm)
    whole_mpc_obs = np.array(whole_mpc_obs)
    whole_mpc_acts = np.array(whole_mpc_acts)

    # normalize observation
    tmp_whole_obs = whole_mpc_obs.reshape(-1,whole_mpc_obs.shape[-1]).copy()
    obs_mpc_mean = np.mean(tmp_whole_obs,axis = 0)
    obs_mpc_var = np.mean((tmp_whole_obs - obs_mpc_mean)**2,axis = 0)
    obs_cnt = float(traj_size*num_traj)
    env.setObStatistics(obs_mpc_mean, obs_mpc_var,obs_cnt)