/* 
    Authors: Amin Arriaga, Eduardo Lopez
    Project: Graduation Thesis: GIAdog

    The file contains the base gai class for the foot trajectory generation.

*/
#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Dense>
#include "../__env__.hpp"

class BaseGait {
    /*
    
    Class that represent the simple bezier FTG, used in:
    
    https://arxiv.org/pdf/2010.11251.pdf
    
    */
    
public:
    BaseGait( Eigen::Vector4d sigma_0,
              double f0,
              Eigen::Vector3d hz);
    
    std::pair<Eigen::Vector3d, double>  FTG(double sigma_i_0,
                            double t,
                            double f_i)
        {
            /*
            Generates a vector in R^3 representing the desired foot position
            (end efector) in the H_i frame corresponding to the robots i-th leg
            horizontal frame below its hip.
            
            Arguments:
            ----------
                sigma_i_0 : float 
                    Contact phase.

                t  : float 
                    Timestep.

                f_i: float 
                    i-th leg frequency offset (from NN policy).
            */
        double sigma_i =  (sigma_i_0 + t * (this->f0 + f_i)) % (2 * M_PI);
        double k       =  2 * (sigma_i - M_PI) / M_PI;
        double h       =  H * 0.8;
        // Lets do some no branching
        Eigen::Vector3d  position;

        bool condition_1 = (k <= 1 && k >= 0);
        bool condition_2 = (k >= 1 && k <= 2);

        position += h * (-2 * k * k * k + 3 * k * k)  * this->hz * condition_1;
        position += h * (2 * k * k * k - 9 * k * k + 12 * k - 4) * \
                    this->hz * condition_2;
        // See that if no condition is meet the position is the vector 0
        return std::make_pair(position, sigma_i); 
        };

    std::tuple<Eigen::MatrixXd, Eigen::Vector4d, Eigen::MatrixXd> \
    compute_foot_trajectories(float t, Eigen::Vector4d frequencies){
        
        /*
        Compute the foot trajectories for the given frequencies, for all the
        four legs for the current time t.

        Arguments:
        ----------
            t : float
                Current time.

            frequencies : Eigen::Vector4d
                Vector of the four frequencies offsets of the four legs.

        */

        Eigen::MatrixXd target_foot_positions(4, 3);
        Eigen::Vector4d FTG_frequencies;
        Eigen::MatrixXd FTG_phases(4, 2);

        for (int i = 0; i < 4; i++) {
            auto [r, sigma_i]  = FTG(this->sigma_0[i], t, frequencies[i]);
            FTG_frequencies[i] = this->f0 + frequencies[i];
            FTG_phases(i, 0)   = std::sin(sigma_i);
            FTG_phases(i, 1)   = std::cos(sigma_i);
            target_foot_positions.row(i) = r;
        }

        return std::make_tuple(target_foot_positions, FTG_frequencies,\
                                                                    FTG_phases);
    };
};
