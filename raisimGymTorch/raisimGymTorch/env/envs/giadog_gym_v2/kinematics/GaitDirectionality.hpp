/*
    Authors: Amin Arriaga, Eduardo Lopez
    Project: Graduation Thesis: GIAdog

    This file contains a function that adds directionality to gaits.
*/

#include <Eigen/Dense>
#include <cmath>
#include "../__env__.hpp"

Eigen::MatrixXd direction_deltas(double delta_t, 
                                Eigen::Vector4d ftg_freqs, 
                                Eigen::Vector4d ftg_sine_phases, 
                                double command_dir, 
                                int turn_dir,
                                bool add_cartesian_delta,
                                bool add_angular_delta) 
                                {
    /*
        Function to add directionality to gaits.

        Is based on an heuristic similar to the one used by Raibert.

        Notes:
        The constants might be changed to be more general.


        Arguments:
        ----------
            delta_t: double
                Time step.
            
            ftg_freqs: Eigen::Vector4d
                The frequencies of the four legs.
            
            ftg_sine_phases: Eigen::Vector4d
                The phases of the four legs.

            command_dir: double
                The desired direction of the robot. (It is an angle in radians)
            
            turn_dir: int
                The direction of the turn. 1 for clockwise, -1 for 
                counterclockwise, 0 for no turn.
            
            add_cartesian_delta: bool
                Whether to add the cartesian delta (position delta).
            
            add_angular_delta: bool
                Whether to add the angular delta (rotation delta).
    */
    Eigen::MatrixXd delta(4, 3);

    for (int i = 0; i < 4; i++) {
        Eigen::Vector3d position_delta(0,0,0);
        
        // Position delta
        position_delta(0) =  cos(command_dir) * delta_t * ftg_sine_phases(i) * \
                          ftg_freqs(i) * LEG_SPAN * 1.7;
        position_delta(1) =  sin(command_dir) * delta_t * ftg_sine_phases(i) * \
                          ftg_freqs(i) * LEG_SPAN * 1.02;

        // Rotation delta (Look Mom no branching!!)

        Eigen::Vector3d rotation_delta(0, 0, 0);
        
        double theta = M_PI/4;
        double phi_arc = (i == 0) * -theta + (i == 1) * -(M_PI - theta) + \
                         (i == 2) *  theta + (i == 3) *  (M_PI - theta);
        
        rotation_delta(0) = -cos(phi_arc) * delta_t * ftg_sine_phases(i) * turn_dir \ 
                          * ftg_freqs(i) * LEG_SPAN * 0.68;
        rotation_delta(1) = -sin(phi_arc) * delta_t * ftg_sine_phases(i) * turn_dir \
                          * ftg_freqs(i) * LEG_SPAN * 0.68;
                          
        
        Eigen::Vector3d rotation_delta(x_delta, y_delta, 0);
        
        delta.row(i) =  (position_delta * add_cartesian_delta + \
                         rotation_delta * add_cartesian_delta);
        }

    return delta;
}