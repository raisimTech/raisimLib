/**
 * Python wrappers for raisim.contact using pybind11.
 *
 * Copyright (c) 2019, kangd and jhwangbo (C++), Brian Delhaisse <briandelhaisse@gmail.com> (Python wrappers)
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
#include <pybind11/stl.h>     // automatic conversion between std::vector, std::list, etc to Python list/tuples/dict
#include <pybind11/eigen.h>   // automatic conversion between Eigen data types to Numpy data types

#include "raisim/math.hpp"   // contains the definitions of Vec, Mat, etc.
#include "raisim/contact/Contact.hpp"
#include "raisim/contact/BisectionContactSolver.hpp"
#include "raisim/contact/Contact.hpp"

#include "converter.hpp"  // contains code that allows to convert between the Vec, Mat to numpy arrays.

namespace py = pybind11;
using namespace raisim;


void init_contact(py::module &m) {


    // create submodule
    py::module contact_module = m.def_submodule("contact", "RaiSim contact submodule.");

    /*****************/
    /* Contact class */
    /*****************/
    py::class_<raisim::Contact>(contact_module, "Contact", "Raisim Contact.")

        .def("get_position", [](raisim::Contact &self) {
            Vec<3> position = self.getPosition();
            return convert_vec_to_np(position);
        }, R"mydelimiter(
	    Get the contact position.

	    Returns:
	        np.array[float[3]]: contact position in the world.
	    )mydelimiter")


        .def("getNormal", [](raisim::Contact &self) {
            Vec<3> normal = self.getNormal();
            return convert_vec_to_np(normal);
        }, R"mydelimiter(
	    Get the contact normal.

	    Returns:
	        np.array[float[3]]: contact normal in the world.
	    )mydelimiter")


        .def("getContactFrame", [](raisim::Contact &self) {
            Mat<3, 3> frame = self.getContactFrame();
            return convert_mat_to_np(frame);
        }, R"mydelimiter(
	    Get the contact frame.

	    Returns:
	        np.array[float[3, 3]]: contact frame.
	    )mydelimiter")


	    .def("getIndexContactProblem", &raisim::Contact::getIndexContactProblem, R"mydelimiter(
	    Get the index contact problem.

	    Returns:
	        int: index.
	    )mydelimiter")


	    .def("getPairObjectIndex", &raisim::Contact::getPairObjectIndex, R"mydelimiter(
	    Get the pair object index.

	    Returns:
	        int: index.
	    )mydelimiter")


	    .def("getPairContactIndexInPairObject", &raisim::Contact::getPairContactIndexInPairObject, R"mydelimiter(
	    Get the pair contact index in pair objects.

	    Returns:
	        int: index.
	    )mydelimiter")


	    .def("getImpulse", [](raisim::Contact &self) {
            Vec<3> impulse = self.getImpulse();
            return convert_vec_to_np(impulse);
        }, R"mydelimiter(
	    Get the impulse.

	    Returns:
	        np.array[float[3]]: impulse.
	    )mydelimiter")


	    .def("isObjectA", &raisim::Contact::isObjectA, R"mydelimiter(
	    Check if it is object A.

	    Returns:
	        bool: True if object A is in contact.
	    )mydelimiter")

	    .def("getPairObjectBodyType", &raisim::Contact::getPairObjectBodyType, R"mydelimiter(
	    Get the pair object body type.

	    Returns:
	        raisim.BodyType: the body type (STATIC, KINEMATIC, DYNAMIC)
	    )mydelimiter")

	    .def("getlocalBodyIndex", &raisim::Contact::getlocalBodyIndex, R"mydelimiter(
	    Get local body index.

	    Returns:
	        int: local body index.
	    )mydelimiter")


        .def("getDepth", &raisim::Contact::getDepth, R"mydelimiter(
	    Get the depth.

	    Returns:
	        float: depth.
	    )mydelimiter")


        .def("isSelfCollision", &raisim::Contact::isSelfCollision, R"mydelimiter(
	    Return True if self-collision is enabled.

	    Returns:
	        bool: True if self-collision is enabled.
	    )mydelimiter")


        .def("skip", &raisim::Contact::skip, R"mydelimiter(
	    Return True if we contact is skipped.

	    Returns:
	        bool: True if the contact is skipped.
	    )mydelimiter");
}