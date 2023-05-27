/**
 * Python wrappers for RaiSim using pybind11.
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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>     // automatic conversion between std::vector, std::list, etc to Python list/tuples/dict
#include <pybind11/eigen.h>   // automatic conversion between Eigen data types to Numpy data types

#include <iostream>

#include "ode/ode.h"
#include "ode/collision.h"

namespace py = pybind11;


void init_math(py::module &);
void init_materials(py::module &);
void init_object(py::module &);
void init_constraints(py::module &);
void init_contact(py::module &);
void init_world(py::module &);


// The PYBIND11_MODULE() macro creates a function that will be called when an import statement is issued from within
// Python. In the following, "raisim" is the module name, "m" is a variable of type py::module which is the main
// interface for creating bindings. The method module::def() generates binding code that exposes the C++ function
// to Python.
PYBIND11_MODULE(raisimpy, m) {

    // add meta information
	m.doc() = "Python wrappers for the RaiSim library."; // docstring for the module
	m.attr("__license__") = "PROPRIETARY";
	py::list authors;
	authors.append("Jemin Hwangbo (raisim)");
	authors.append("Donho Kang (raisim)");
	authors.append("Joonho Lee (raisim)");
	authors.append("Brian Delhaisse (raisimpy)");
	m.attr("__authors__") = authors;

    /*******/
    /* Ode */
    /*******/
    /* collision space from ode (in ode/common.h), used to define dSpaceID */
    py::class_<dSpaceID>(m, "dSpaceID", "collision space from ode (in ode/common.h).");
    /* geometry collision object from ode (in ode/common.h), used to define dGeomID */
    py::class_<dGeomID>(m, "dGeomID", "geometry collision object from ode (in ode/common.h).");

    /********/
    /* Math */
    /********/
    init_math(m);

	/*************/
	/* Materials */
    /*************/
    init_materials(m);

    /******************/
	/* raisim.contact */
	/******************/
	init_contact(m);

    /*****************/
	/* raisim.object */
	/*****************/
	init_object(m);  // define primitive shapes and articulated systems)

    /*********************/
	/* raisim.constraint */
	/*********************/
  init_constraints(m);

    /*********/
	/* World */
	/*********/
	init_world(m);
}