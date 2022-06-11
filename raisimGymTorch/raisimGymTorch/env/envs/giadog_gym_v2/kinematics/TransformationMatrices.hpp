/*
    Authors: Amin Arriaga, Eduardo Lopez
    Project: Graduation Thesis: GIAdog

    Functions to generate the transformation matrices from the roboto hips to 
    the horizontal frames.

*/

#include <Eigen/Dense>
#include <cmath>
#include "../__env__.hpp"

Eigen::Matrix3d rotation_matrix_from_euler(Eigen::Vector3d base_rpy) 
    {
    /*
        Calculates a rotation matrix from the euler angles.

        Arguments:
        ----------
            roll: double
                roll angle
            
            pitch: double
                pitch angle
            
            yaw: double
                yaw angle
        
        Returns:
        -------
            rotationMatrix: Eigen::Matrix3d
                Rotation matrix.
    */
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rotationMatrix = q.matrix();
    
    return rotationMatrix;
    }

std::vector<Eigen::Matrix4d>  transformation_matrices(Eigen::Vector3d base_rpy){, 
    /*
        Returns the transformation matrices from the hip to the leg base.
    
        Arguments:
        ---------
            base_rpy: numpy.array, shape(3,) 
                The hip's (and robot base) euler angles. (roll, pitch, yaw)

        Returns:
        --------
            Eigen::MatrixXd, shape (4,4). 
                A list containing the transformation matrices from the hip to 
                the leg base, for each of the robots legs.
            The order of the matrices is: LF, RF, LB, RB.
    */

    std::vector<Eigen::Matrix4d> T_matrices;
    
    // We transform the base orientation from quaternion to matrix
    // We also get the base euler angles
    
    auto R = rotation_matrix_from_euler(base_rpy(0), base_rpy(1), 0);
        
    for (int i = 0; i < 4; i++) {
        // We calculate the Hi frame position relative to the leg base position 
        // (hip)
        Eigen::Vector3d  p(0, H_OFF * pow(-1, i), -LEG_SPAN);
        
        // Finally we concatenate the rotation matrix and position vector
        // To get the transformation matrix of the Hi horizontal frame expressed
        // in the leg base frame
        Eigen::Matrix4d T_i{
                            {R.coeff(0,0), R.coeff(0,1), R.coeff(0,2), p.coeff(0)},
                            {R.coeff(1,0), R.coeff(1,1), R.coeff(1,2), p.coeff(1)},
                            {R.coeff(2,0), R.coeff(2,1), R.coeff(2,2), p.coeff(2)},
                            {0     , 0     , 0     , 1   }
                           };

        T_matrices.push_back(T_i)
    }
    
    return T_matrices
}