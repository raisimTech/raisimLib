/**
 * Python wrappers for raisim.math using pybind11. Have also a look at
 * `converter.hpp` and `converter.cpp` which contains the code to convert between
 * np.array to raisim::Vec, raisim::Mat, raisim::VecDyn, raisim::MatDyn, and
 * raisim::Transformation.
 *
 * Copyright (c) 2019, jhwangbo (C++), Brian Delhaisse <briandelhaisse@gmail.com> (Python wrappers)
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


#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>   // automatic conversion between Eigen data types to Numpy data types


#include "raisim/math.hpp"   // contains the definitions of Vec, Mat, Transformation, etc.

#include "converter.hpp"  // contains code that allows to convert between the Vec, Mat to numpy arrays.

namespace py = pybind11;
using namespace raisim;


void init_math(py::module &m) {
    /******************/
    /* Transformation */
    /******************/
    py::class_<raisim::Transformation>(m, "Transformation", "Raisim homogeneous transformation.")
        .def(py::init<>())  // default constructor
        .def_property("rot",
            [](raisim::Transformation &self) { // getter
                // convert from Mat<3,3> to np.array[3,3]
                return convert_mat_to_np(self.rot);
            }, [](raisim::Transformation &self, py::array_t<double> array) { // setter
                // convert from np.array[3,3] to Mat<3,3>
                Mat<3,3> rot = convert_np_to_mat<3,3>(array);
                self.rot = rot;
            })
        .def_property("pos",
            [](raisim::Transformation &self) { // getter
                // convert from Vec<3> to np.array[3]
                return convert_vec_to_np(self.pos);
            }, [](raisim::Transformation &self, py::array_t<double> array) { // setter
                // convert from np.array[3] to Vec<3>
                Vec<3> pos = convert_np_to_vec<3>(array);
                self.pos = pos;
            });


    /******************/
    /* SparseJacobian */
    /******************/

    // In Python, we will use instead Numpy instead of this class. For this purpose, you will have to get the MatDyn
    // member variable from the SparseJacobian class and use the converters in `converter.hpp` to convert this MatDyn
    // to a valid numpy array.

}
