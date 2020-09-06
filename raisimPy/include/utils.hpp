/**
 * Utils methods such as transformation between quaternion and rotation matrices.
 *
 * Copyright (c) 2019, Brian Delhaisse <briandelhaisse@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#ifndef RAISIMPY_UTILS_H
#define RAISIMPY_UTILS_H

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>   // numpy types

#include <sstream>           // for ostringstream
#include "raisim/math.hpp"   // contains the definitions of Vec, Mat, VecDyn, MatDyn, etc
#include "Eigen/Geometry"    // for Eigen::Quaterniond

#include "converter.hpp" // to convert between raisim data types and numpy arrays

namespace py = pybind11;


/// \brief: convert from raisim::Vec<4> (quaternion) to raisim::Mat<3,3> (rotation matrix)

/// \brief: convert from raisim::Mat<3,3> (rotation matrix) to raisim::Vec<4> (quaternion)

/// \brief: convert from quaternion (py::array_t<double>[4]) to rotation matrix (py::array_t<double>[3,3])

/// \brief: convert from rotation matrix (py::array_t<double>[3,3]) to quaternion (py::array_t<double>[4])

#endif