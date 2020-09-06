/**
 * Python wrappers for raisim.Materials using pybind11.
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

#include <iostream>

#include "raisim/Materials.hpp"


namespace py = pybind11;
using namespace raisim;


void init_materials(py::module &m) {


	/**************************/
	/* MaterialPairProperties */
    /**************************/
    py::class_<raisim::MaterialPairProperties>(m, "MaterialPairProperties", "Raisim Material Pair Properties (friction and restitution).")
        .def(py::init<>(), "Initialize the material pair properties.")
        .def(py::init<double, double, double>(),
        "Initialize the material pair properties.\n\n"
        "Args:\n"
        "    friction (float): coefficient of friction.\n"
        "    restitution (float): coefficient of restitution.\n"
        "    threshold (float): restitution threshold.",
        py::arg("friction"), py::arg("restitution"), py::arg("threshold"));


    /*******************/
    /* MaterialManager */
    /*******************/
    py::class_<raisim::MaterialManager>(m, "MaterialManager", "Raisim Material Manager.")
        .def(py::init<>(), "Initialize the material pair manager.")
        .def(py::init<const std::string>(),
        "Initialize the material manager by uploading the material data from a file.\n\n"
        "Args:\n"
        "    xml_file (float): xml file.",
        py::arg("xml_file"))
        .def("set_material_pair_properties", &raisim::MaterialManager::setMaterialPairProp, R"mydelimiter(
        Set the material pair properties (friction and restitution).

        Args:
            material1 (str): first material.
            material2 (str): second material.
            friction (float): coefficient of friction.
            restitution (float): coefficient of restitution.
            threshold (float): restitution threshold.
        )mydelimiter",
        py::arg("material1"), py::arg("material2"), py::arg("friction"), py::arg("restitution"), py::arg("threshold"))
        .def("get_material_pair_properties", &raisim::MaterialManager::getMaterialPairProp, R"mydelimiter(
        Get the material pair properties (friction and restitution).

        Args:
            material1 (str): first material.
            material2 (str): second material.

        Returns:
            MaterialPairProperties: material pair properties (friction, restitution, and restitution threshold).
        )mydelimiter",
        py::arg("material1"), py::arg("material2"))
        .def("set_default_material_properties", &raisim::MaterialManager::setDefaultMaterialProperties, R"mydelimiter(
        Set the default material properties.

        Args:
            friction (float): coefficient of friction.
            restitution (float): coefficient of restitution.
            threshold (float): restitution threshold.
        )mydelimiter",
        py::arg("friction"), py::arg("restitution"), py::arg("threshold"));

}