//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "Environment.hpp"
#include "VectorizedEnvironment.hpp"

namespace py = pybind11;
using namespace raisim;

#ifndef ENVIRONMENT_NAME
  #define ENVIRONMENT_NAME RaisimGymEnv
#endif

PYBIND11_MODULE(RAISIMGYM_TORCH_ENV_NAME, m) {
  py::class_<Space>(m, "Space")
    .def(py::init<>())
    // .def(py::init<const std::tuple<int, int>&>())
    .def_readonly("shape", &Space::shape);

  py::class_<VectorizedEnvironment<ENVIRONMENT>>(m, RSG_MAKE_STR(ENVIRONMENT_NAME))
    .def(py::init<std::string, std::string>())
    .def("init", &VectorizedEnvironment<ENVIRONMENT>::init)
    .def("reset", &VectorizedEnvironment<ENVIRONMENT>::reset)
    .def("observe", &VectorizedEnvironment<ENVIRONMENT>::observe)
    .def("step", &VectorizedEnvironment<ENVIRONMENT>::step)
    .def("setSeed", &VectorizedEnvironment<ENVIRONMENT>::setSeed)
    .def("seed", &VectorizedEnvironment<ENVIRONMENT>::setSeed)  // OpenAI Gym compatibility
    // .def("info", [](VectorizedEnvironment<ENVIRONMENT>* instance) {
    //   std::cout << "ObDim " << instance->getObDim() << std::endl;
    //   std::cout << "ActionDim " << instance->getActionDim() << std::endl;
    //   std::cout << "NumOfEnvs " << instance->getNumOfEnvs() << std::endl;
    // })
    .def_readonly("action_space", &VectorizedEnvironment<ENVIRONMENT>::action_space)            // OpenAI Gym compatibility
    .def_readonly("observation_space", &VectorizedEnvironment<ENVIRONMENT>::observation_space)  // OpenAI Gym compatibility
    .def_readonly("metadata", &VectorizedEnvironment<ENVIRONMENT>::metadata)                    // OpenAI Gym compatibility
    .def_readonly("reward_range", &VectorizedEnvironment<ENVIRONMENT>::reward_range)            // OpenAI Gym compatibility
    .def("close", &VectorizedEnvironment<ENVIRONMENT>::close)
    .def("isTerminalState", &VectorizedEnvironment<ENVIRONMENT>::isTerminalState)
    .def("setSimulationTimeStep", &VectorizedEnvironment<ENVIRONMENT>::setSimulationTimeStep)
    .def("setControlTimeStep", &VectorizedEnvironment<ENVIRONMENT>::setControlTimeStep)
    .def("getObDim", &VectorizedEnvironment<ENVIRONMENT>::getObDim)
    .def("getActionDim", &VectorizedEnvironment<ENVIRONMENT>::getActionDim)
    .def("getNumOfEnvs", &VectorizedEnvironment<ENVIRONMENT>::getNumOfEnvs)
    .def("turnOnVisualization", &VectorizedEnvironment<ENVIRONMENT>::turnOnVisualization)
    .def("turnOffVisualization", &VectorizedEnvironment<ENVIRONMENT>::turnOffVisualization)
    .def("stopRecordingVideo", &VectorizedEnvironment<ENVIRONMENT>::stopRecordingVideo)
    .def("startRecordingVideo", &VectorizedEnvironment<ENVIRONMENT>::startRecordingVideo)
    .def("curriculumUpdate", &VectorizedEnvironment<ENVIRONMENT>::curriculumUpdate)
    .def(py::pickle(
        [](const VectorizedEnvironment<ENVIRONMENT> &p) { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            // std::cout << "PICKLING TO PYTHON" << std::endl;
            return py::make_tuple(p.get_resource_directory(), p.get_cfg_string());
        },
        [](py::tuple t) { // __setstate__
            // std::cout << "PICKLING FROM PYTHON" << std::endl;
            if (t.size() != 2) {
              throw std::runtime_error("Invalid state!");
            }

            /* Create a new C++ instance */
            VectorizedEnvironment<ENVIRONMENT> p(t[0].cast<std::string>(), t[1].cast<std::string>());

            return p;
        }
    ));
}
