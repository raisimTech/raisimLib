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

#include "converter.hpp"

namespace py = pybind11;


/// \brief: convert from raisim::VecDyn to np.array[float64[n]]
py::array_t<double> convert_vecdyn_to_np(const raisim::VecDyn &vec) {
    const double *ptr = vec.ptr();  // get data pointer
    size_t n = vec.n; // get dimension

    // return np.array[float64[n,m]]
    return py::array_t<double>(
        {n},     // shape
        {sizeof(double)},   // C-style contiguous strides for double (double=8bytes)
        ptr);
//        vec);   // numpy array references this parent
}


/// \brief: convert from np.array[float[n]] to raisim::VecDyn
raisim::VecDyn convert_np_to_vecdyn(py::array_t<double> &array) {

    size_t size = array.size();

    // reshape if necessary
    if (array.ndim() > 1)
        array.resize({size});

    // create raisim dynamic vector
    raisim::VecDyn vec(size);

    // copy the data
    for(size_t i=0; i<size; i++) {
        vec[i] = *array.data(i);
    }

    // return vector
    return vec;
}


/// \brief: convert from raisim::MatDyn to np.array[float64[n,m]]
py::array_t<double> convert_matdyn_to_np(const raisim::MatDyn &mat) {
    const double *ptr = mat.ptr();  // get data pointer
    size_t n = mat.n;
    size_t m = mat.m;

    // return np.array[float64[n,m]]
    return py::array_t<double>(
        {n, m},     // shape
        {sizeof(double), n*sizeof(double)},   // C-style contiguous strides for double (double=8bytes)
        ptr);
//        mat);   // numpy array references this parent
}


/// \brief: convert from np.array[float[n,m]] to raisim::MatDyn
raisim::MatDyn convert_np_to_matdyn(const py::array_t<double> &array) {

    // check dimensions and shape
    if (array.ndim() != 2) {
        std::ostringstream s;
        s << "error: expecting the given array to have a dimension of 2, but got instead a dimension of "
            << array.ndim() << ".";
        throw std::domain_error(s.str());
    }

    // get the number of rows and columns
    size_t nrows = array.shape(0);
    size_t ncols = array.shape(1);

    // create raisim matrix
    raisim::MatDyn mat(nrows, ncols);

    // copy the data
    for (size_t i=0; i<nrows; i++)
        for (size_t j=0; j<ncols; j++)
            mat(i, j) = *array.data(i, j);

    // return matrix
    return mat;
}


/// \brief: convert from raisim::Transformation to np.array[float[4,4]]
py::array_t<double> convert_transformation_to_np(const raisim::Transformation &transfo) {

    // convert from vec / mat to np.array
//    auto pos = convert_vec_to_np(transfo.pos);
//    auto rot = convert_mat_to_np(transfo.rot);
    auto pos = transfo.pos;
    auto rot = transfo.rot;

    // create 4x4 matrix
    py::array_t<double> homogeneous({4, 4});

    // fill for rotation matrix
    for (size_t i=0; i<3; i++)
        for (size_t j=0; j<3; j++)
            *homogeneous.mutable_data(i, j) = rot(i, j); // *rot.data(i, j);

    // fill for position vector
    for (size_t i=0; i<3; i++)
        *homogeneous.mutable_data(i, 3) = pos[i];  //*pos.data(i);

    // fill zero for last row
    for (size_t j=0; j<3; j++)
        *homogeneous.mutable_data(3, j) = 0;

    // fill 1 for the last cell (last row and last column)
    *homogeneous.mutable_data(4,4) = 1;

    return homogeneous;
}


/// \brief: convert from np.array[float[4,4]] to raisim::Transformation
raisim::Transformation convert_np_to_transformation(const py::array_t<double> &array) {

    // check dimensions and shape
    if (array.ndim() != 2) {
        std::ostringstream s;
        s << "error: expecting the given array to have a dimension of 2, but got instead a dimension of "
            << array.ndim() << ".";
        throw std::domain_error(s.str());
    }
    if (array.shape(0) != 4 || array.shape(1) != 4) {
        std::ostringstream s;
        s << "error: expecting the given array to have a shape (4,4), but got instead a shape of ("
            << array.shape(0) << "," << array.shape(1) << ").";
        throw std::domain_error(s.str());
    }

    // create transformation
    raisim::Transformation transfo;
    raisim::Mat<3,3> rot = transfo.rot;
    raisim::Vec<3> pos = transfo.pos;

    // fill rotation matrix
    for (size_t i=0; i<3; i++)
        for (size_t j=0; j<3; j++)
            rot(i, j) = *array.data(i, j);

    // fill position vector
    for (size_t i=0; i<3; i++)
        pos[i] = *array.data(i, 3);

    return transfo;
}


/// \brief: convert from Eigen::Quaterniond to np.array[float[4]]
py::array_t<double> convert_quaternion_to_np(const Eigen::Quaterniond &quaternion) {
    // create vector of size 4
    py::array_t<double> array(4);

    // fill quaternion
    *array.mutable_data(0) = quaternion.w();
    *array.mutable_data(1) = quaternion.x();
    *array.mutable_data(2) = quaternion.y();
    *array.mutable_data(3) = quaternion.z();

    // return quaternion
    return array;
}


/// \brief: convert from np.array[float[4]] to Eigen::Quaterniond
Eigen::Quaterniond convert_np_to_quaternion(py::array_t<double> &array) {

    // check dimensions and shape
    if (array.size() != 4) {
        std::ostringstream s;
        s << "error: expecting the given array to have a size of 4, but got instead a size of "
            << array.size() << ".";
        throw std::domain_error(s.str());
    }

    // reshape if necessary
    if (array.ndim() > 1)
        array.resize({array.size()});

    // create quaternion
    Eigen::Quaterniond quaternion(*array.data(0), *array.data(1), *array.data(2), *array.data(3));

    // return quaternion
    return quaternion;
}
