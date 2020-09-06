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
#include "raisim/constraints/Wire.hpp"
#include "raisim/constraints/StiffWire.hpp"
#include "raisim/constraints/CompliantWire.hpp"

#include "converter.hpp"  // contains code that allows to convert between the Vec, Mat to numpy arrays.

namespace py = pybind11;
using namespace raisim;


void init_constraints(py::module &m) {


    // create submodule
    py::module constraints_module = m.def_submodule("constraints", "RaiSim contact submodule.");


    /**************/
    /* Constraint */
    /**************/
    py::class_<raisim::Constraints>(constraints_module, "Constraints", "Raisim Constraints from which all other constraints inherit from.");


    /********/
    /* Wire */
    /********/
    py::class_<raisim::Wire, raisim::Constraints>(constraints_module, "Wire", "Raisim Wire constraint class; it creates a wire constraint between 2 bodies.")
        .def(py::init([](raisim::Object &object1, size_t local_idx1, py::array_t<double> pos_body1,
            raisim::Object &object2, size_t local_idx2, py::array_t<double> pos_body2, double length)
            {
                // convert the arrays to Vec<3>
                raisim::Vec<3> pos1 = convert_np_to_vec<3>(pos_body1);
                raisim::Vec<3> pos2 = convert_np_to_vec<3>(pos_body2);

                // instantiate the class
                return new raisim::Wire(&object1, local_idx1, pos1, &object2, local_idx2, pos2, length);
            }),
            "Instantiate the wire constraint class.\n\n"
	        "Args:\n"
	        "    object1 (Object): first object/body instance.\n"
	        "    local_idx1 (int): local index of the first object/body.\n"
	        "    pos_body1 (np.array[float[3]]): position of the constraint on the first body.\n"
            "    object2 (Object): second object/body instance.\n"
	        "    local_idx2 (int): local index of the second object/body.\n"
	        "    pos_body2 (np.array[float[3]]): position of the constraint on the second body.\n"
            "    length (float): length of the wire constraint.",
            py::arg("object1"), py::arg("local_idx1"), py::arg("pos_body1"), py::arg("object2"), py::arg("local_idx2"),
            py::arg("pos_body2"), py::arg("length"))


        .def("update", &raisim::Wire::update, "update internal variables (called by `integrate1()`).")


        .def("get_length", &raisim::Wire::getLength, R"mydelimiter(
	    Get the length of the wire constraint.

	    Returns:
	        float: length of the wire constraint.
	    )mydelimiter")


        .def("get_p1", [](raisim::Wire &self) {
            Vec<3> p1 = self.getP1();
            return convert_vec_to_np(p1);
        }, R"mydelimiter(
	    Return the first attachment point in the World frame.

	    Returns:
	        np.array[float[3]]: first point position expressed in the world frame.
	    )mydelimiter")


	    .def("get_p2", [](raisim::Wire &self) {
            Vec<3> p2 = self.getP2();
            return convert_vec_to_np(p2);
        }, R"mydelimiter(
	    Return the second attachment point in the World frame.

	    Returns:
	        np.array[float[3]]: second point position expressed in the world frame.
	    )mydelimiter")


        .def("get_body1", &raisim::Wire::getBody1, R"mydelimiter(
	    Return the first object to which the wire is attached.

	    Returns:
	        Object: first object.
	    )mydelimiter")


        .def("get_body2", &raisim::Wire::getBody2, R"mydelimiter(
	    Return the second object to which the wire is attached.

	    Returns:
	        Object: second object.
	    )mydelimiter")


        .def("get_normal", [](raisim::Wire &self) {
            Vec<3> normal = self.getNorm();
            return convert_vec_to_np(normal);
        }, R"mydelimiter(
	    Return the direction of the normal (i.e., p2-p1 normalized)

	    Returns:
	        np.array[float[3]]: direction of the normal.
	    )mydelimiter")


        .def("get_local_idx1", &raisim::Wire::getLocalIdx1, R"mydelimiter(
	    Return the local index of object1.

	    Returns:
	        int: local index of object1.
	    )mydelimiter")


        .def("get_local_idx2", &raisim::Wire::getLocalIdx2, R"mydelimiter(
	    Return the local index of object2.

	    Returns:
	        int: local index of object2.
	    )mydelimiter")


        .def("get_stretch", &raisim::Wire::getStretch, R"mydelimiter(
	    Return the stretch length (i.e., constraint violation).

	    Returns:
	        float: stretch length.
	    )mydelimiter")


	    .def_property("name", &raisim::Wire::getName, &raisim::Wire::setName)
	    .def("get_name", &raisim::Wire::getName, "Get the wire constraint's name.")
	    .def("set_name", &raisim::Wire::setName, "Set the wire constraint's name.", py::arg("name"))
	    .def_readwrite("is_active", &raisim::Wire::isActive)
    ;


    /*************/
    /* StiffWire */
    /*************/

    py::class_<raisim::StiffWire, raisim::Wire>(constraints_module, "StiffWire", "Raisim StiffWire constraint class; it creates a stiff wire constraint between 2 bodies.")
        .def(py::init([](raisim::Object &object1, size_t local_idx1, py::array_t<double> pos_body1,
            raisim::Object &object2, size_t local_idx2, py::array_t<double> pos_body2, double length)
            {
                // convert the arrays to Vec<3>
                raisim::Vec<3> pos1 = convert_np_to_vec<3>(pos_body1);
                raisim::Vec<3> pos2 = convert_np_to_vec<3>(pos_body2);

                // instantiate the class
                return new raisim::StiffWire(&object1, local_idx1, pos1, &object2, local_idx2, pos2, length);
            }),
            "Instantiate the stiff wire constraint class.\n\n"
	        "Args:\n"
	        "    object1 (Object): first object/body instance.\n"
	        "    local_idx1 (int): local index of the first object/body.\n"
	        "    pos_body1 (np.array[float[3]]): position of the constraint on the first body.\n"
            "    object2 (Object): second object/body instance.\n"
	        "    local_idx2 (int): local index of the second object/body.\n"
	        "    pos_body2 (np.array[float[3]]): position of the constraint on the second body.\n"
            "    length (float): length of the wire constraint.",
            py::arg("object1"), py::arg("local_idx1"), py::arg("pos_body1"), py::arg("object2"), py::arg("local_idx2"),
            py::arg("pos_body2"), py::arg("length"));


    /*****************/
    /* CompliantWire */
    /*****************/

    py::class_<raisim::CompliantWire, raisim::Wire>(constraints_module, "CompliantWire", "Raisim Compliant Wire constraint class; it creates a compliant wire constraint between 2 bodies.")
        .def(py::init([](raisim::Object &object1, size_t local_idx1, py::array_t<double> pos_body1,
            raisim::Object &object2, size_t local_idx2, py::array_t<double> pos_body2, double length, double stiffness)
            {
                // convert the arrays to Vec<3>
                raisim::Vec<3> pos1 = convert_np_to_vec<3>(pos_body1);
                raisim::Vec<3> pos2 = convert_np_to_vec<3>(pos_body2);

                // instantiate the class
                return new raisim::CompliantWire(&object1, local_idx1, pos1, &object2, local_idx2, pos2, length, stiffness);
            }),
            "Instantiate the compliant wire constraint class.\n\n"
	        "Args:\n"
	        "    object1 (Object): first object/body instance.\n"
	        "    local_idx1 (int): local index of the first object/body.\n"
	        "    pos_body1 (np.array[float[3]]): position of the constraint on the first body.\n"
            "    object2 (Object): second object/body instance.\n"
	        "    local_idx2 (int): local index of the second object/body.\n"
	        "    pos_body2 (np.array[float[3]]): position of the constraint on the second body.\n"
            "    length (float): length of the wire constraint.\n"
            "    stiffness (float): stiffness of the wire.",
            py::arg("object1"), py::arg("local_idx1"), py::arg("pos_body1"), py::arg("object2"), py::arg("local_idx2"),
            py::arg("pos_body2"), py::arg("length"), py::arg("stiffness"))
        .def("apply_tension", &raisim::CompliantWire::applyTension, "Apply a tension in the compliant wire.");

}