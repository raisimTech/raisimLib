import raisimpy as raisim
import numpy as np

ABDUCTION_P_GAIN = 100.0
ABDUCTION_D_GAIN = 1.
HIP_P_GAIN = 100.0
HIP_D_GAIN = 2.0
KNEE_P_GAIN = 100.0
KNEE_D_GAIN = 2.0


class PyEnvironment(object):
    def __init__(self,a1_urdf_file,simulation_dt):
        self.simulation_dt = simulation_dt
        self.motor_kp = np.array([
            ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN, ABDUCTION_P_GAIN,
            HIP_P_GAIN, KNEE_P_GAIN, ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN,
            ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN
        ])
        self.motor_kd = np.array([
            ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN, ABDUCTION_D_GAIN,
            HIP_D_GAIN, KNEE_D_GAIN, ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN,
            ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN
        ])
        self.a1_nominal_joint_config = np.array([0.0, 0.0, 0.27, 1, 0.0, 0.0, 0.0,
                                                0.0, 0.9, -1.8,        0.0, 0.9, -1.8,
                                                0.0, 0.9, -1.8,       0.0, 0.9, -1.8])
        self.num_obs = 26
        self.num_acts = 48

        self.world = raisim.World()
        self.world.setTimeStep(self.simulation_dt)
        self.ground = self.world.addGround()
        self.a1_obj = self.world.addArticulatedSystem(a1_urdf_file)
        self.a1_obj.setName("a1")
        self.a1_obj.setGeneralizedCoordinate(self.a1_nominal_joint_config)
        
        self.gcDim_ = self.a1_obj.getGeneralizedCoordinateDim()
        self.gvDim_ = self.a1_obj.getDOF()
        self.nJoints_ = self.gvDim_ - 6

    def reset(self):
        kp = np.append(np.zeros(6),self.motor_kp)
        kd = np.append(np.zeros(6),self.motor_kd)

        pTarget_ = np.append(np.zeros(7),self.a1_nominal_joint_config)
        vTarget_ = np.append(np.zeros(6),np.full(12, 0))
        additional_torques = np.append(np.zeros(6),np.full(12, 0))

        self.a1_obj.setPdGains(kp, kd)
        self.a1_obj.setPdTarget(pTarget_,vTarget_)
        self.a1_obj.setGeneralizedForce(additional_torques)
        self.a1_obj.setJointDamping(np.zeros(self.gvDim_))

        for _ in range(500):
            self.world.integrate()

    def load_scaling(self, dir_name, iteration, count=1e5):
        mean_file_name = dir_name + "/mean" + str(iteration) + ".csv"
        var_file_name = dir_name + "/var" + str(iteration) + ".csv"
        self.count = count
        self.mean = np.loadtxt(mean_file_name, dtype=np.float32)
        self.var = np.loadtxt(var_file_name, dtype=np.float32)
        self.wrapper.setObStatistics(self.mean, self.var, self.count)