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
#include "raisim/contact/PerObjectContactList.hpp"

#include "converter.hpp"  // contains code that allows to convert between the Vec, Mat to numpy arrays.

namespace py = pybind11;
using namespace raisim;


void init_contact(py::module &m) {


    // create submodule
    py::module contact_module = m.def_submodule("contact", "RaiSim contact submodule.");

    /*****************/
    /* Contact class */
    /*****************/
    py::class_<raisim::contact::Contact>(contact_module, "Contact", "Raisim Contact.")
        .def(py::init([](py::array_t<double> position, py::array_t<double> normal, bool objectA,
            size_t contact_problem_index, size_t contact_index_in_object, size_t pair_object_index,
            BodyType pair_object_body_type, size_t pair_contact_index_in_pair_object, size_t local_body_index,
            double depth)
            {
                // convert the arrays to Vec<3>
                raisim::Vec<3> pos = convert_np_to_vec<3>(position);
                raisim::Vec<3> norm = convert_np_to_vec<3>(normal);

                // instantiate the class
                return new raisim::contact::Contact(pos, norm, objectA, contact_problem_index, contact_index_in_object,
                    pair_object_index, pair_object_body_type, pair_contact_index_in_pair_object, local_body_index,
                    depth);
            }),
            "Instantiate the contact class.\n\n"
	        "Args:\n"
	        "    position (np.array[float[3]]): position vector.\n"
	        "    normal (np.array[float[3]]): normal vector.\n"
	        "    objectA (bool): True if object A.\n"
	        "    contact_problem_index (int): contact problem index.\n"
	        "    contact_index_in_object (int): contact index in object (an object can be in contact at multiple points).\n"
	        "    pair_object_index (int): pair object index.\n"
	        "    pair_object_index (BodyType): pair object body type between {STATIC, KINEMATIC, DYNAMIC}.\n"
            "    pair_contact_index_in_pair_object (int): pair contact index in pair object.\n"
            "    local_body_index (int): local body index."
            "    depth (float): depth of the contact.")


        .def("get_position", [](raisim::contact::Contact &self) {
            Vec<3> position = self.getPosition();
            return convert_vec_to_np(position);
        }, R"mydelimiter(
	    Get the contact position.

	    Returns:
	        np.array[float[3]]: contact position in the world.
	    )mydelimiter")


        .def("get_normal", [](raisim::contact::Contact &self) {
            Vec<3> normal = self.getNormal();
            return convert_vec_to_np(normal);
        }, R"mydelimiter(
	    Get the contact normal.

	    Returns:
	        np.array[float[3]]: contact normal in the world.
	    )mydelimiter")


        .def("get_contact_frame", [](raisim::contact::Contact &self) {
            Mat<3, 3> frame = self.getContactFrame();
            return convert_mat_to_np(frame);
        }, R"mydelimiter(
	    Get the contact frame.

	    Returns:
	        np.array[float[3, 3]]: contact frame.
	    )mydelimiter")


	    .def("get_index_contact_problem", &raisim::contact::Contact::getIndexContactProblem, R"mydelimiter(
	    Get the index contact problem.

	    Returns:
	        int: index.
	    )mydelimiter")


	    .def("get_pair_object_index", &raisim::contact::Contact::getPairObjectIndex, R"mydelimiter(
	    Get the pair object index.

	    Returns:
	        int: index.
	    )mydelimiter")


	    .def("get_pair_contact_index_in_pair_object", &raisim::contact::Contact::getPairContactIndexInPairObject, R"mydelimiter(
	    Get the pair contact index in pair objects.

	    Returns:
	        int: index.
	    )mydelimiter")


	    .def("get_impulse", [](raisim::contact::Contact &self) {
            Vec<3> *impulse = self.getImpulse();
            return convert_vec_to_np(*impulse);
        }, R"mydelimiter(
	    Get the impulse.

	    Returns:
	        np.array[float[3]]: impulse.
	    )mydelimiter")


	    .def("is_objectA", &raisim::contact::Contact::isObjectA, R"mydelimiter(
	    Check if it is object A.

	    Returns:
	        bool: True if object A is in contact.
	    )mydelimiter")


	    .def("get_pair_object_body_type", &raisim::contact::Contact::getPairObjectBodyType, R"mydelimiter(
	    Get the pair object body type.

	    Returns:
	        raisim.BodyType: the body type (STATIC, KINEMATIC, DYNAMIC)
	    )mydelimiter")


	    .def("set_impulse", [](raisim::contact::Contact &self, py::array_t<double> impulse) {
	        Vec<3> impulse_ = convert_np_to_vec<3>(impulse);
            self.setImpulse(&impulse_);
        }, R"mydelimiter(
	    Set the impulse.

	    Args:
	        np.array[float[3]]: impulse.
	    )mydelimiter",
	    py::arg("impulse"))


	    .def("set_inverse_inertia", [](raisim::contact::Contact &self, py::array_t<double> inverse_inertia) {
	        Mat<3, 3> I_ = convert_np_to_mat<3, 3>(inverse_inertia);
            self.setInvInertia(&I_);
        }, R"mydelimiter(
	    Set the inverse of the inertia matrix.

	    Args:
	        np.array[float[3,3]]: inverse of the inertia matrix.
	    )mydelimiter",
	    py::arg("inverse_inertia"))


	    .def("get_inverse_inertia", [](raisim::contact::Contact &self) {
	        const Mat<3, 3> *I_ = self.getInvInertia();
	        return convert_mat_to_np(*I_);
        }, R"mydelimiter(
	    Get the inverse inertia matrix.

	    Returns:
	        np.array[float[3,3]]: inverse of the inertia matrix.
	    )mydelimiter")


	    .def("get_local_body_index", &raisim::contact::Contact::getlocalBodyIndex, R"mydelimiter(
	    Get local body index.

	    Returns:
	        int: local body index.
	    )mydelimiter")


        .def("get_depth", &raisim::contact::Contact::getDepth, R"mydelimiter(
	    Get the depth.

	    Returns:
	        float: depth.
	    )mydelimiter")


        .def("is_self_collision", &raisim::contact::Contact::isSelfCollision, R"mydelimiter(
	    Return True if self-collision is enabled.

	    Returns:
	        bool: True if self-collision is enabled.
	    )mydelimiter")


	    .def("set_self_collision", &raisim::contact::Contact::setSelfCollision, "Enable self-collision.")


        .def("skip", &raisim::contact::Contact::skip, R"mydelimiter(
	    Return True if we contact is skipped.

	    Returns:
	        bool: True if the contact is skipped.
	    )mydelimiter");

    /************************/
    /* PerObjectContactList */
    /************************/
    py::class_<raisim::contact::PerObjectContactList>(contact_module, "PerObjectContactList",
        "Raisim PerObjectContactList, where each object has a list of Contacts.")
        .def(py::init<>(), "Initialize the per object contact list.")
        .def("get_num_contacts", &raisim::contact::PerObjectContactList::getNumContacts)
        .def("get_contact_at", &raisim::contact::PerObjectContactList::getContactAt, py::arg("index"))
        .def("get_contacts", py::overload_cast<>(&raisim::contact::PerObjectContactList::getContacts))
        .def("get_contacts", py::overload_cast<>(&raisim::contact::PerObjectContactList::getContacts, py::const_))
        .def("get_impact_velocities", &raisim::contact::PerObjectContactList::getImpactVel);
}