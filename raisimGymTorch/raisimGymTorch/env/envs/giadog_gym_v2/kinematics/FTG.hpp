


#include "FTG_Handler.hpp"
#include <Eigen/Dense>
#include "../__env__.hpp"
#include "TransformationMatrices.hpp"
#include "InverseKinematics.hpp"


FTG_Handler:: FTG_Handler foot_trajectory_geneator = FTG_Handler(FTG_type);

std::tuple<Eigen::VectorXd, Eigen::Vector4d, Eigen::VectorXd> \
    calculate_joint_angles(Eigen::Vector2d command,
                           Eigen::VectorXd NN_output,
                           Eigen::Vector3d base_rpy,
                           double time_step)
    {  
    //Initialize the joint angle vector
    Eigen::VectorXd joint_angles = Eigen::VectorXd::Zero(12);
    // Calculate the transformation matrices for the base and the legs.
    auto T_matrices = transformation_matrices(base_rpy);

    // For each leg, calculate the foot position and the joints angles.
    auto [foot_positions, ftg_freqs, ftg_phases] = foot_trajectory_geneator.gen_trajectories(command, 
                                              NN_output, 
                                              time_step, 
                                              CARTESIAN_DIR,
                                              ANGULAR_DIR);

    for (int i = 0; i < 4; i++)
    {   
        Eigen::Matrix4d T_i = T_matrices[i];

        Eigen::Vector3d foot_position_leg_frame;

        foot_position_leg_frame = T_i.topLeftCorner(3,3) * foot_position[i] + \
                                  T_i.topRightCorner(3,1);
        

        bool right_leg = (i == 1 || i == 3);
        // Calculate the joint angles.
        Eigen::Vector3d joint_angles_leg;
        joint_angles_leg = solve_leg_IK(right_leg, foot_position_base);

        // Add the joint angles to the joint angle vector.
        joint_angles(i*4)   = joint_angles_leg.coeff(0);
        joint_angles(i*4+1) = joint_angles_leg.coeff(1);
        joint_angles(i*4+2) = joint_angles_leg.coeff(2);

    }
    return std::make_tuple(joint_angles, ftg_freqs, ftg_phases);

}