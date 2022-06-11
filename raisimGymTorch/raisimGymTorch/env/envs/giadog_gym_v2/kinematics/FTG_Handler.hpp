/*

*/

#include "Gait.hpp"
#include "GaitDirectionality.hpp"
#include <Eigen/Dense>
#include "../__env__.hpp"
class FTG_Handler
{
    public:
        FTG_Handler(std::string FTG_type){    
            if(FTG_type == "base"){
                Gait::BaseGait this->FTG(F0, SIGMA_0, HZ); 
                double this-> dt = SIM_SECONDS_PER_STEP; // * CONTROLLER_LATENCY_STEPS       
            }
            else{
                throw std::runtime_error("The specified FTG: " + FTG_type +\
                                                         " is not supported.");
            }
        };
        
        std::tuple<Eigen::MatrixXd, Eigen::Vector4d, Eigen::MatrixXd> \
        gen_trajectories(Eigen::Vector2d command, 
                         Eigen::VectorXd theta, 
                         double t,
                         bool cartesian_directionality,
                         bool angular_directionality
                        )
            {
            /*
            Generates the foot trajectories for the given theta and t.
            */
            
            Eigen::MatrixXd xyz_residual = {
                {theta.coeff(0), theta.coeff(1), theta.coeff(2)},
                {theta.coeff(3), theta.coeff(4), theta.coeff(5)},
                {theta.coeff(6), theta.coeff(7), theta.coeff(8)},
                {theta.coeff(9), theta.coeff(10), theta.coeff(11)},   
            };
            
            Eigen::Vector4d frequencies = theta.segment(12, 4);

            double command_dir = command[0];
            double turn_dir = command[1];
            auto[foot_target_pos, ftg_freqs, ftg_phases] =  \
               FTG.compute_foot_trajectories(t, frequencies);
            
            auto dir_deltas = GaitDirectionality::direction_deltas(
                                                    this-> dt,
                                                    ftg_freqs,
                                                    ftg_phases.row(0),
                                                    command_dir,
                                                    turn_dir,
                                                    cartesian_directionality,
                                                    angular_directionality
                                                    );

            foot_target_pos +=  xyz_residual + dir_deltas;

            return std::make_tuple(foot_target_pos, ftg_freqs, ftg_phases);
            };
        



}
