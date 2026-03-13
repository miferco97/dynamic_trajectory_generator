/*!*******************************************************************************************
 *  \file       trajectory_tets.cpp
 *  \brief      Test trajectory generation and store it in a CSV file
 *  \authors    Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2024 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

 #include <yaml-cpp/yaml.h>
 #include <filesystem>
 #include <fstream>
 #include <iostream>
 #include <memory>
 #include <stdexcept>

 #include "dynamic_trajectory_generator/dynamic_trajectory.hpp"
 #include "dynamic_trajectory_generator/dynamic_waypoint.hpp"


class CsvLogger
{
public:
  explicit CsvLogger(const std::string & file_name)
  : file_name_(file_name)
  {
    std::cout << "Saving to file: " << file_name << std::endl;
    file_ = std::ofstream(file_name, std::ofstream::out | std::ofstream::trunc);
    file_ << "time,x,y,z,vx,vy,vz,ax,ay,az" << std::endl;
  }

  ~CsvLogger() {file_.close();}


  void save(const double time, const dynamic_traj_generator::References & references)
  {
    // Save time
    file_ << time << ",";

    for (int i = 0; i < 3; i++) {
      // Save position, velocity and acceleration
      file_ << references.position[i] << ",";
    }

    for (int i = 0; i < 3; i++) {
      // Save velocity
      file_ << references.velocity[i] << ",";
    }

    for (int i = 0; i < 3; i++) {
      // Save acceleration
      file_ << references.acceleration[i];
      if (i < 2) {
        file_ << ",";
      }
    }

    // End line
    file_ << std::endl;
  }

  void close() {file_.close();}

private:
  std::string file_name_;
  std::ofstream file_;
};


dynamic_traj_generator::DynamicWaypoint::Vector eigen_vector_to_dynamic_waypoint_vector(
  const std::vector<std::pair<std::string, Eigen::Vector3d>> & vector_waypoints)
{
  dynamic_traj_generator::DynamicWaypoint::Vector vector_dynamic_waypoints;
  for (auto waypoint : vector_waypoints) {
    dynamic_traj_generator::DynamicWaypoint dynamic_waypoint;
    dynamic_waypoint.resetWaypoint(waypoint.second);
    dynamic_waypoint.setName(waypoint.first);
    vector_dynamic_waypoints.push_back(dynamic_waypoint);
  }
  return vector_dynamic_waypoints;
}


int main(int argc, char ** argv)
{
  // Logger
  CsvLogger logger("trajectory.csv");

  using DynamicTrajectory = dynamic_traj_generator::DynamicTrajectory;
  using DynamicWaypoint = dynamic_traj_generator::DynamicWaypoint;

  Eigen::Vector3d initial_position = Eigen::Vector3d(8.0, 22.0, 1.23);
  double speed = 10.0;

  std::vector<std::pair<std::string, Eigen::Vector3d>> vector_waypoints;

  // First run
  vector_waypoints.push_back({"gate1_0", Eigen::Vector3d(14.0, 25.0, 1.23)});
  vector_waypoints.push_back({"gate2_0", Eigen::Vector3d(30.0, 19.0, 1.23)});
  vector_waypoints.push_back({"gate3_0", Eigen::Vector3d(46.0, 22.0, 1.23)});
  vector_waypoints.push_back({"gate4_0", Eigen::Vector3d(63.0, 20.0, 3.93)});
  vector_waypoints.push_back({"ls_gate5prev_0", Eigen::Vector3d(84.0, 18.0, 3.93)});
  vector_waypoints.push_back({"ls_gate5post_0", Eigen::Vector3d(90.0, 16.0, 2.93)});
  vector_waypoints.push_back({"ls_gate6prev_0", Eigen::Vector3d(86.0, 19.0, 1.5)});
  vector_waypoints.push_back({"ls_gate6post_0", Eigen::Vector3d(84.0, 18.0, 1.23)});
  vector_waypoints.push_back({"gate7_0", Eigen::Vector3d(68.0, 13.0, 1.23)});
  vector_waypoints.push_back({"gate8_0", Eigen::Vector3d(55.0, 7.0, 1.23)});
  vector_waypoints.push_back({"gate9_0", Eigen::Vector3d(37.0, 12.0, 1.23)});
  vector_waypoints.push_back({"gate10_0", Eigen::Vector3d(19.7, 7.0, 1.23)});
  vector_waypoints.push_back({"gate11_0", Eigen::Vector3d(9.0, 14.0, 1.23)});

  // // Second run
  // vector_waypoints.push_back({"gate1_1", Eigen::Vector3d(14.0, 25.0, 1.23)});
  // vector_waypoints.push_back({"gate2_1", Eigen::Vector3d(30.0, 19.0, 1.23)});
  // vector_waypoints.push_back({"gate3_1", Eigen::Vector3d(46.0, 22.0, 1.23)});
  // vector_waypoints.push_back({"gate4_1", Eigen::Vector3d(63.0, 20.0, 3.93)});
  // vector_waypoints.push_back({"ls_gate5prev_1", Eigen::Vector3d(84.0, 18.0, 3.93)});
  // vector_waypoints.push_back({"ls_gate5post_1", Eigen::Vector3d(86.0, 18.0, 3.93)});
  // vector_waypoints.push_back({"ls_gate6prev_1", Eigen::Vector3d(86.0, 18.0, 1.23)});
  // vector_waypoints.push_back({"ls_gate6post_1", Eigen::Vector3d(84.0, 18.0, 1.23)});
  // vector_waypoints.push_back({"gate7_1", Eigen::Vector3d(68.0, 13.0, 1.23)});
  // vector_waypoints.push_back({"gate8_1", Eigen::Vector3d(55.0, 7.0, 1.23)});
  // vector_waypoints.push_back({"gate9_1", Eigen::Vector3d(37.0, 12.0, 1.23)});
  // vector_waypoints.push_back({"gate10_1", Eigen::Vector3d(19.7, 7.0, 1.23)});
  // vector_waypoints.push_back({"gate11_1", Eigen::Vector3d(9.0, 14.0, 1.23)});

  // End point
  vector_waypoints.push_back({"endpoint", Eigen::Vector3d(8.0, 22.0, 1.23)});

  // Initialize dynamic trajectory generator
  std::unique_ptr<DynamicTrajectory> trajectory_generator = std::make_unique<DynamicTrajectory>();

  // Set trajectory generator parameters
  trajectory_generator->updateVehiclePosition(initial_position);
  trajectory_generator->setSpeed(speed);
  DynamicWaypoint::Vector waypoints_to_set = eigen_vector_to_dynamic_waypoint_vector(
    vector_waypoints);

  // Generate trajectory
  trajectory_generator->setWaypoints(waypoints_to_set);
  double max_time = trajectory_generator->getMaxTime();   // Block until trajectory is generated

  // Evaluate trajectory
  dynamic_traj_generator::References references;
  double dt = 0.01;   // seconds
  for (double t = 0.0; t < max_time - dt; t += dt) {
    trajectory_generator->evaluateTrajectory(t, references);
    logger.save(t, references);
  }
  return 0;
}
