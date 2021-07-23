#############################
Math Classes
#############################

RaiSim provides its own math library.
All files related to the math library can be found in `raisim/math`.

There are two reasons that RaiSim uses its own math library and not the Eigen library:

1. Eigen often has issues with AVX2 instructions. I didn't want users struggle with such problems.
2. Eigen is not optimized for some operations in RaiSim.

RaiSim math library is very similar to Eigen.
But it is a minimal version and I am keep adding features to it.

If you do not want to use RaiSim math library, then you can easily convert RaiSim math classes to corresponding Eigen classes.
For example,