/*
    This file is supposed to contain the enviroment variables:
    * The robot meassurements
    * Gait Parameters

*/

#include <Eigen/Dense>

// Robot Meassurements
#define H_OFF     0.063;
#define V_OFF     0.008;
#define THIGH_LEN 0.11058;
#define SHANK_LEN 0.1265;
#define LEG_SPAN  0.2442;
#define H         0.2; // Max foot height

// Gait Parameters
const Eigen::Vector4d SIGMA_0(0., 3.142, 3.142, 0.);
const Eigen::Vector3d HZ(0., 0., 1.);
#define F0 4.0;
#define SIM_SECONDS_PER_STEP 0.005;
#define FTG_type "base";

#define CARTESIAN_DIR true;
#define ANGULAR_DIR true;