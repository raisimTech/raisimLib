//
// Created by jemin on 20. 7. 9..
//

#ifndef RAISIM_RAISIMMATLAB_RAISIM_INTERFACE_MEX_HPP_H_
#define RAISIM_RAISIMMATLAB_RAISIM_INTERFACE_MEX_HPP_H_
#include <iostream>
#include <memory>
#include "mex.h"
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

namespace raisim {
using Mat33 = raisim::Mat<3, 3>;
using Vec3 = raisim::Vec<3>;
};  // namespace raisim

void mexFunction(int nlhs, mxArray* p_output[], int nrhs, const mxArray* p_input[]);

#endif //RAISIM_RAISIMMATLAB_RAISIM_INTERFACE_MEX_CPP_H_
