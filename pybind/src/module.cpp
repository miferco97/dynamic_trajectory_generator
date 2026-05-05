// Copyright 2025 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names
//      of its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file module.cpp
 *
 * @brief Python bindings for the Dynamic Trajectory Generator.
 *
 * @author Rafael Pérez Seguí <r.psegui@upm.es>
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "dynamic_trajectory_generator_pybind/dynamic_trajectory_pybind.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_dynamic_trajectory_generator_cpp, m)
{
  m.doc() = "Python bindings for Dynamic Trajectory Generator";

  py::class_<dynamic_traj_generator::DynamicTrajectoryBind>(m, "_DynamicTrajectory")
    .def(py::init<>())
    .def(
      "generate_trajectory",
      &dynamic_traj_generator::DynamicTrajectoryBind::generateTrajectory,
      py::arg("initial_position"),
      py::arg("initial_yaw"),
      py::arg("waypoints"),
      py::arg("max_velocity"),
      "Generate a polynomial trajectory from the current vehicle position through the given "
      "waypoints at the given maximum velocity. Blocks until the optimizer returns.")
    .def(
      "evaluate_trajectory",
      &dynamic_traj_generator::DynamicTrajectoryBind::evaluateTrajectory,
      py::arg("time"),
      "Evaluate the trajectory at the given time (s). Returns (position, velocity, acceleration, "
      "yaw).")
    .def(
      "set_path_facing",
      &dynamic_traj_generator::DynamicTrajectoryBind::setPathFacing,
      py::arg("enable"),
      "Enable/disable automatic yaw alignment with the trajectory velocity direction.")
    .def(
      "get_max_time",
      &dynamic_traj_generator::DynamicTrajectoryBind::getMaxTime,
      "Return the maximum valid evaluation time of the trajectory (s).")
    .def(
      "get_min_time",
      &dynamic_traj_generator::DynamicTrajectoryBind::getMinTime,
      "Return the minimum valid evaluation time of the trajectory (s).")
    .def(
      "get_was_trajectory_regenerated",
      &dynamic_traj_generator::DynamicTrajectoryBind::getWasTrajectoryRegenerated,
      "Return whether the trajectory has been regenerated since the last query.");
}
