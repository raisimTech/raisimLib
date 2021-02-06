/**
 * Python wrappers for raisim.constraints using pybind11.
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
#include <pybind11/stl.h>     // automatic conversion between std::vector, std::list, etc to Python list/tuples/dict
#include <pybind11/eigen.h>   // automatic conversion between Eigen data types to Numpy data types

#include "raisim/math.hpp"   // contains the definitions of Vec, Mat, etc.
#include "raisim/object/Object.hpp"
#include "raisim/constraints/Constraints.hpp"
#include "raisim/constraints/LengthConstraint.hpp"
#include "raisim/constraints/StiffLengthConstraint.hpp"
#include "raisim/constraints/CompliantLengthConstraint.hpp"
#include "raisim/constraints/CustomLengthConstraint.hpp"

#include "converter.hpp"  // contains code that allows to convert between the Vec, Mat to numpy arrays.

namespace py = pybind11;
using namespace raisim;


void init_constraints(py::module &m) {


    // create submodule
    py::module constraints_module = m.def_submodule("constraints", "RaiSim contact submodule.");

    py::enum_<raisim::LengthConstraint::StretchType>(m, "StretchType", py::arithmetic())
        .value("STRETCH_RESISTANT_ONLY", raisim::LengthConstraint::StretchType::STRETCH_RESISTANT_ONLY)
        .value("COMPRESSION_RESISTANT_ONLY", raisim::LengthConstraint::StretchType::COMPRESSION_RESISTANT_ONLY)
        .value("BOTH", raisim::LengthConstraint::StretchType::BOTH);
    /**************/
    /* Constraint */
    /**************/
    py::class_<raisim::Constraints>(constraints_module, "Constraints", "Raisim Constraints from which all other constraints inherit from.");


    /********/
    /* Wire */
    /********/
    py::class_<raisim::LengthConstraint, raisim::Constraints>(constraints_module, "LengthConstraint", "Raisim LengthConstraint constraint class; it creates a LengthConstraint constraint between 2 bodies.")

        .def("update", &raisim::LengthConstraint::update, "update internal variables (called by `integrate1()`).")


        .def("getLength", &raisim::LengthConstraint::getLength, R"mydelimiter(
	    Get the length of the LengthConstraint constraint.

	    Returns:
	        float: length of the LengthConstraint constraint.
	    )mydelimiter")


        .def("getP1", [](raisim::LengthConstraint &self) {
            Vec<3> p1 = self.getP1();
            return convert_vec_to_np(p1);
        }, R"mydelimiter(
	    Return the first attachment point in the World frame.

	    Returns:
	        np.array[float[3]]: first point position expressed in the world frame.
	    )mydelimiter")


	    .def("getP2", [](raisim::LengthConstraint &self) {
            Vec<3> p2 = self.getP2();
            return convert_vec_to_np(p2);
        }, R"mydelimiter(
	    Return the second attachment point in the World frame.

	    Returns:
	        np.array[float[3]]: second point position expressed in the world frame.
	    )mydelimiter")


        .def("getBody1", &raisim::LengthConstraint::getBody1, R"mydelimiter(
	    Return the first object to which the LengthConstraint is attached.

	    Returns:
	        Object: first object.
	    )mydelimiter")


        .def("getBody2", &raisim::LengthConstraint::getBody2, R"mydelimiter(
	    Return the second object to which the LengthConstraint is attached.

	    Returns:
	        Object: second object.
	    )mydelimiter")


        .def("getNorm", [](raisim::LengthConstraint &self) {
            Vec<3> normal = self.getNorm();
            return convert_vec_to_np(normal);
        }, R"mydelimiter(
	    Return the direction of the normal (i.e., p2-p1 normalized)

	    Returns:
	        np.array[float[3]]: direction of the normal.
	    )mydelimiter")


        .def("getLocalIdx1", &raisim::LengthConstraint::getLocalIdx1, R"mydelimiter(
	    Return the local index of object1.

	    Returns:
	        int: local index of object1.
	    )mydelimiter")


        .def("getLocalIdx2", &raisim::LengthConstraint::getLocalIdx2, R"mydelimiter(
	    Return the local index of object2.

	    Returns:
	        int: local index of object2.
	    )mydelimiter")


        .def("getStretch", &raisim::LengthConstraint::getStretch, R"mydelimiter(
	    Return the stretch length (i.e., constraint violation).

	    Returns:
	        float: stretch length.
	    )mydelimiter")

        .def("setStretchType", &raisim::LengthConstraint::setStretchType, R"mydelimiter(
	    Return the stretch type (i.e., constraint violation).

	    Returns:
	        stretch_type: stretch type.
	    )mydelimiter")

        .def_property("name", &raisim::LengthConstraint::getName, &raisim::LengthConstraint::setName)
	    .def("getName", &raisim::LengthConstraint::getName, "Get the LengthConstraint constraint's name.")
	    .def("setName", &raisim::LengthConstraint::setName, "Set the LengthConstraint constraint's name.", py::arg("name"))
	    .def_readwrite("isActive", &raisim::LengthConstraint::isActive)
    ;


    /*************/
    /* StiffLengthConstraint */
    /*************/

    py::class_<raisim::StiffLengthConstraint, raisim::LengthConstraint>(constraints_module, "StiffLengthConstraint", "Raisim StiffLengthConstraint constraint class; it creates a stiff wire constraint between 2 bodies.");


    /*************/
    /* CustomLengthConstraint */
    /*************/
    py::class_<raisim::CustomLengthConstraint, raisim::LengthConstraint>(constraints_module, "CustomLengthConstraint", "Raisim CustomLengthConstraint class; it creates a stiff wire constraint between 2 bodies.")
        .def("setTension", &raisim::CustomLengthConstraint::setTension, "Set the tension in the wire.\n"
                                                                        "Args:\n"
                                                                        "   tension (float): tension in the wire", py::arg("tension"));

  /*****************/
    /* CompliantLengthConstraint */
    /*****************/

    py::class_<raisim::CompliantLengthConstraint, raisim::LengthConstraint>(constraints_module, "CompliantLengthConstraint", "Raisim Compliant Wire constraint class; it creates a compliant wire constraint between 2 bodies.");

}