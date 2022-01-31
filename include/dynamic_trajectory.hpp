#ifndef __DYNAMIC_TRAJECTORY_HPP__
#define __DYNAMIC_TRAJECTORY_HPP__

#include <chrono>
#include <exception>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include "mav_trajectory_generation/trajectory.h"

#include "matplotlibcpp.h"
#include "thread_safe_trajectory.hpp"
#include "utils/logging_utils.hpp"
#include "utils/traj_modifiers.hpp"

#include "dynamic_waypoint.hpp"

#define MAV_MAX_ACCEL (1 * 9.81f)

namespace dynamic_traj_generator
{

  struct References
  {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;

    Eigen::Vector3d &operator[](int index)
    {
      switch (index)
      {
      case 0:
        return position;
      case 1:
        return velocity;
      case 2:
        return acceleration;
      default:
        throw std::runtime_error("Invalid index");
      }
    }
  };

  class DynamicTrajectory
  {
  private:
    // const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::JERK;
    const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::ACCELERATION;
    // const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::VELOCITY;

    ThreadSafeTrajectory traj_;
    std::future<ThreadSafeTrajectory> future_traj_;

    const int dimension_ = 3;
    std::atomic_bool from_scratch_ = true;
    const double a_max_ = MAV_MAX_ACCEL;

    double last_t_eval_ = 0.0f;
    double t_offset_ = 0.0f;

    std::mutex traj_mutex_;
    std::mutex future_mutex_;
    std::mutex dynamic_waypoints_mutex_;

    dynamic_traj_generator::DynamicWaypoint::Vector dynamic_waypoints_;
    dynamic_traj_generator::DynamicWaypoint::Vector temporal_dynamic_waypoints_;

    /**********************************************************************/
    /************************ NEW FUNCTIONALITIES *************************/
    /**********************************************************************/

  public:
    std::atomic_bool generate_new_traj_ = false;
    std::atomic_bool in_security_zone_ = false;
    std::atomic_bool computing_new_trajectory_ = false;

    void setWaypoints(const DynamicWaypoint::Vector &waypoints)
    {
      const std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
      dynamic_waypoints_ = waypoints;
    };

    bool modifyWaypoint(const std::string &name, const Eigen::Vector3d &position);
    void appendWaypoint(const DynamicWaypoint &waypoint)
    {
      const std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
      dynamic_waypoints_.emplace_back(waypoint);
    };

    void generateTraj(float speed, bool force = false);
    ThreadSafeTrajectory __generatetrajectory(float max_speed);

    /**********************************************************************/
    /************************ END NEW FUNCTIONALITIES *********************/
    /**********************************************************************/

  public:
    DynamicTrajectory(){};

    double getMaxTime();
    double getMinTime();

    inline mav_trajectory_generation::Vertex::Vector getWaypoints() { return traj_.getWaypoints(); }
    inline mav_trajectory_generation::Segment::Vector getSegments() { return traj_.getSegments(); }
    void generateTrajectory(const dynamic_traj_generator::DynamicWaypoint::Vector &waypoints, const float &max_speed);
    bool obtainDynamicWaypoints(const std::string &waypoint_name, DynamicWaypoint &waypoint);

    DynamicWaypoint::Vector getDynamicWaypoints();
    bool evaluateTrajectory(const float &t, dynamic_traj_generator::References &refs, bool only_positions = false);

  private:
    void selectProperTrajectoryGenerationMethod(const mav_trajectory_generation::Vertex::Vector &waypoints, const float &max_speed)
    {
      if (from_scratch_)
        generateTrajectoryFromScratch(waypoints, max_speed);
      else
        throw std::runtime_error("Not implemented");
    };

    void generateTrajectoryFromScratch(const mav_trajectory_generation::Vertex::Vector &vertices,
                                       const float &max_speed);
    bool checkTrajectoryGenerated();
    ThreadSafeTrajectory computeTrajectory(const mav_trajectory_generation::Vertex::Vector &vertices,
                                           const float &max_speed,
                                           const bool &lineal_optimization = false);
    void swapTrajectory();
    void swapDynamicWaypoints(std::vector<dynamic_traj_generator::DynamicWaypoint> &waypoints);
    bool getRefs(const ThreadSafeTrajectory &traj,
                 float t_eval, dynamic_traj_generator::References &refs,
                 const bool only_position = false);
  };

} // namespace mav_planning

#endif // __DYNAMIC_TRAJECTORY_HPP__