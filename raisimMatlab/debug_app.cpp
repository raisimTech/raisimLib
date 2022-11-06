//
// Created by jemin on 20. 7. 9..
//

#include "raisim_interface_mex.hpp"
#include "raisim/World.hpp"

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  mxArray* input[2];
  mxArray* output[1];
  input[0] = mxCreateString("init");
  input[1] = mxCreateString((binaryPath.getDirectory() + "../../rsc/laikago.xml").getString().c_str());

  mexFunction(0, &output[0], 2, const_cast<const mxArray**>(&input[0]));

}