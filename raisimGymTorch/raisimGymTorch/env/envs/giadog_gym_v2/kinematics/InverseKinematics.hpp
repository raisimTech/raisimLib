/*
    Authors: Amin Arriaga, Eduardo Lopez
    Project: Graduation Thesis: GIAdog

    Inverse kinematics class for the giadog robot. (Spot mini mini // Open Quadruped)
    
    References:
    -----------
        * Muhammed Arif Sen, Veli Bakircioglu, Mete Kalyoncu. (Sep, 2017). 
        Inverse Kinematic Analysis Of A Quadruped Robot  
        https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot

        * Some of the code was taken from the sopt_mini_mini implementation 
        of the same paper.
        https://github.com/OpenQuadruped/spot_mini_mini/blob/spot/spotmicro/Kinematics/LegKinematics.py

*/

#define _USE_MATH_DEFINES
#include <Eigen/Dense>
#include <cmath>
#include "utils.hpp" // for clip
#include <math.h> // for atan2
#include "../__env__.hpp"


Eigen::Vector2d get_IK_params(
    double x, 
    double y,
    double z,
){  /*
        Calculates the leg's Inverse kinematicks parameters:
        The leg Domain 'D' (caps it in case of a breach) and the leg's radius.

        Arguments:
        ----------
            x: double  
                hip-to-foot distance in x-axis

            y: double  
                hip-to-foot distance in y-axis

            z: double  
                hip-to-foot distance in z-axis

        Returns:
        -------
            double
                leg's Domain D

            double
                leg's outer radius
    */
    double r_o, D, sqrt_component;
    sqrt_component = max(0, pow(z, 2) + pow(y, 2) - pow(H_OFF, 2));
    r_o = sqrt(sqrt_component) - V_OFF;
    D = clip(
        (pow(r_o, 2) + pow(x, 2) - pow(SHANK_L, 2)  - pow(THIGH_L, 2)) / (2 * SHANK_L * THIGH_L),
        -1.0, 
        1.0
    );

    return std::make_pair(D, r_o);
};

Eigen::Vector3d right_leg_IK(
    double x, 
    double y, 
    double z, 
    double D, 
    double r_o){
    /*
        Right Leg Inverse Kinematics Solver
        
        Arguments:
        ---------_
            x: double  
                hip-to-foot distance in x-axis

            y: double  
                hip-to-foot distance in y-axis

            z: double  
                hip-to-foot distance in z-axis
            
            D: double
                Leg domain
            
            r_o: double
                Radius of the leg

        Return:
        -------
            Eigen::Vector3d 
                Joint Angles required for desired position. 
                The order is: Hip, Thigh, Shank
                Or: (shoulder, elbow, wrist)
    */

    // Declare the variables
    double wrist_angle, shoulder_angle, elbow_angle;
    double second_sqrt_component, q_o;

    wrist_angle    = atan2(-sqrt(1 - pow(D, 2)), D);
    shoulder_angle = - atan2(z, y) - atan2(r_o, - H_OFF);
    second_sqrt_component = max(
        0,
        pow(r_o, 2) + pow(x, 2) - pow((SHANK_L * sin(wrist_angle)), 2)
    );
    q_o = sqrt(second_sqrt_component);
    elbow_angle = atan2(-x, r_o);
    elbow_angle -= atan2(SHANK_L * sin(wrist_angle), q_o);
    Eigen::Vector3d joint_angles(-shoulder_angle, elbow_angle, wrist_angle);

    return joint_angles;
    }

Eigen::Vector3d left_leg_IK(
    double x, 
    double y, 
    double z, 
    double D, 
    double r_o){
    /*
        Left Leg Inverse Kinematics Solver
        
        Arguments:
        ---------_
            x: double  
                hip-to-foot distance in x-axis

            y: double  
                hip-to-foot distance in y-axis

            z: double  
                hip-to-foot distance in z-axis
            
            D: double
                Leg domain
            
            r_o: double
                Radius of the leg

        Return:
        -------
            Eigen::Vector3d 
                Joint Angles required for desired position. 
                The order is: Hip, Thigh, Shank
                Or: (shoulder, elbow, wrist)
    */

    // Declare the variables
    double wrist_angle, shoulder_angle, elbow_angle;
    double second_sqrt_component, q_o;

    wrist_angle    = atan2(-sqrt(1 - pow(D, 2)), D);
    shoulder_angle = - atan2(z, y) - atan2(r_o, H_OFF);
    second_sqrt_component = max(
        0,
        pow(r_o, 2) + pow(x, 2) - pow((SHANK_L * sin(wrist_angle)), 2)
    );
    q_o = sqrt(second_sqrt_component);
    elbow_angle = atan2(-x, r_o);
    elbow_angle -= atan2(SHANK_L * sin(wrist_angle), q_o);
    Eigen::Vector3d joint_angles(-shoulder_angle, elbow_angle, wrist_angle);

    return joint_angles;
    }

Eigen::Vector3d solve_leg_IK(bool right_leg, Eigen::Vector3d r){
    /*
    Calculates the leg's inverse kinematics.
    (joint angles from xyz coordinates).
    
    Arguments:
    ---------_
        right_leg: bool 
            ('l' or 'r') 
            If true, the right leg is solved, otherwise the left leg is solved.
        
        r: Eigen::Vector3d
            Objective foot position in the H_i frame.
            (x,y,z) hip-to-foot distances in each dimension
            The 

    Return:
    -------
        Eigen::Vector3d
            Leg joint angles to reach the objective foot position r. In the 
            order:(Hip, Shoulder, Wrist). The joint angles are expresed in 
            radians.
    */
    auto [D, r_o] = get_IK_params(r(0), r(1), r(2));

    return right_leg ? right_leg_IK(r(0), r(1), r(2), D, r_o)  \
                     : left_leg_IK(r(0), r(1), r(2), D, r_o);
    }