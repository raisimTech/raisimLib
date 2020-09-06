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
  py::class_<VectorizedEnvironment<ENVIRONMENT>>(m, RSG_MAKE_STR(ENVIRONMENT_NAME))
    .def(py::init<std::string, std::string>())
    .def("init", &VectorizedEnvironment<ENVIRONMENT>::init)
    .def("reset", &VectorizedEnvironment<ENVIRONMENT>::reset)
    .def("observe", &VectorizedEnvironment<ENVIRONMENT>::observe)
    .def("step", &VectorizedEnvironment<ENVIRONMENT>::step)
    .def("setSeed", &VectorizedEnvironment<ENVIRONMENT>::setSeed)
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
    .def("curriculumUpdate", &VectorizedEnvironment<ENVIRONMENT>::curriculumUpdate);
}
