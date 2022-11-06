/**
 * Type converters used to convert between different data types.
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

#ifndef RAISIMPY_CONVERTER_H
#define RAISIMPY_CONVERTER_H

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>   // numpy types

#include <sstream>           // for ostringstream
#include "raisim/math.hpp"   // contains the definitions of Vec, Mat, VecDyn, MatDyn, etc.
#include "Eigen/Geometry"    // for Eigen::Quaterniond

namespace py = pybind11;


/// \brief: convert from raisim::Vec<n> to np.array
template<size_t n>
py::array_t<double> convert_vec_to_np(const raisim::Vec<n> &vec) {
    const double *ptr = vec.ptr();  // get data pointer

    // return np.array[float64[n]]
    return py::array_t<double>(
        {n},                // shape
        {sizeof(double)},   // C-style contiguous strides for double (double=8 bytes)
        ptr);               // data pointer
//        vec);   // numpy array references this parent
}


/// \brief: convert from np.array[float[n]] to raisim::Vec<n>
template<size_t n>
raisim::Vec<n> convert_np_to_vec(py::array_t<double> &array) {

    // check size
    if (array.size() != n) {
        std::ostringstream s;
        s << "error: expecting the given vector to be of size " << n << " but got instead a size of "
            << array.size() << ".";
        throw std::domain_error(s.str());
    }

    // reshape if necessary
    if (array.ndim() > 1)
        array.resize({n});

    // create raisim vector
    raisim::Vec<n> vec;

    // copy the data
    for(size_t i=0; i<n; i++) {
        vec[i] = *array.data(i);
    }

    // return vector
    return vec;
}


/// \brief: convert from raisim::Mat<n,m> to np.array[float64[n,m]]
template<size_t n, size_t m>
py::array_t<double> convert_mat_to_np(const raisim::Mat<n, m> &mat) {
    const double *ptr = mat.ptr();  // get data pointer

    // return np.array[float64[n,m]]
    return py::array_t<double>(
        {n, m},     // shape
        {sizeof(double), n*sizeof(double)},   // C-style contiguous strides for double (double=8bytes)
        ptr);
//        mat);   // numpy array references this parent
}


/// \brief: convert from np.array[float[n,m]] to raisim::Mat<n,m>
template<size_t n, size_t m>
raisim::Mat<n, m> convert_np_to_mat(const py::array_t<double> &array) {

    // check dimensions and shape
    if (array.ndim() != 2) {
        std::ostringstream s;
        s << "error: expecting the given array to have a dimension of 2, but got instead a dimension of "
            << array.ndim() << ".";
        throw std::domain_error(s.str());
    }
    if ((array.shape(0) != n) || (array.shape(1) != m)) {
        std::ostringstream s;
        s << "error: expecting the given array to have the following shape (" << n << ", " << m
            << "), but got instead the shape ("<< array.shape(0) << ", " << array.shape(1) << ").";
        throw std::domain_error(s.str());
    }

    // create raisim matrix
    raisim::Mat<n, m> mat;

    // copy the data
    for (size_t i=0; i<n; i++)
        for (size_t j=0; j<m; j++)
            mat(i, j) = *array.data(i, j);

    // return matrix
    return mat;
}

/// \brief: convert from raisim::VecDyn to np.array[float64[n]]
py::array_t<double> convert_vecdyn_to_np(const raisim::VecDyn &vec);


/// \brief: convert from np.array[float[n]] to raisim::VecDyn
raisim::VecDyn convert_np_to_vecdyn(py::array_t<double> &array);


/// \brief: convert from raisim::MatDyn to np.array[float64[n,m]]
py::array_t<double> convert_matdyn_to_np(const raisim::MatDyn &mat);


/// \brief: convert from np.array[float[n,m]] to raisim::MatDyn
raisim::MatDyn convert_np_to_matdyn(const py::array_t<double> &array);


/// \brief: convert from raisim::Transformation to np.array[float[4,4]]
py::array_t<double> convert_transformation_to_np(const raisim::Transformation &transfo);


/// \brief: convert from np.array[float[4,4]] to raisim::Transformation
raisim::Transformation convert_np_to_transformation(const py::array_t<double> &array);


/// \brief: convert from Eigen::Quaterniond to np.array[float[4]]
py::array_t<double> convert_quaternion_to_np(const Eigen::Quaterniond &quaternion);


/// \brief: convert from np.array[float[4]] to Eigen::Quaterniond
Eigen::Quaterniond convert_np_to_quaternion(py::array_t<double> &array);



#endif
