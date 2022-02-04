#ifndef __DYNAMIC_TRAJECTORY_HPP__
#define __DYNAMIC_TRAJECTORY_HPP__

#include <chrono>
#include <exception>
#include <functional>
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
#define N_WAYPOINTS_TO_APPEND 2
#define SECURITY_COEF 0.9
#define TIME_CONSTANT 0.9

constexpr float AsyntoticComplexity(int n)
{
  float value = n * n; // TODO: CALCULATE THIS CORRECTLY
  return value;
}

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
    struct NumericParameters
    {
      double last_t_eval = 0.0f;
      double t_offset = 0.0f;
      double speed = 0.0f;
    } parameters_;

    // const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::JERK;
    const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::ACCELERATION;
    // const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::VELOCITY;

    ThreadSafeTrajectory traj_;
    std::future<ThreadSafeTrajectory> future_traj_;

    const int dimension_ = 3;
    std::atomic_bool from_scratch_ = true;
    const double a_max_ = MAV_MAX_ACCEL;

    mutable std::mutex traj_mutex_;
    mutable std::mutex future_mutex_;
    mutable std::mutex dynamic_waypoints_mutex_;
    mutable std::mutex parameters_mutex;
    mutable std::mutex todo_mutex;

    dynamic_traj_generator::DynamicWaypoint::Deque dynamic_waypoints_;        // buffer to store waypoints not updated
    dynamic_traj_generator::DynamicWaypoint::Deque next_trajectory_waypoint_; // buffer to store waypoints not updated

    dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_be_added_;
    dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_be_set_;
    std::vector<std::pair<std::string, Eigen::Vector3d>> waypoints_to_be_modified_;

    std::atomic_bool generate_new_traj_ = false;
    std::atomic_bool in_security_zone_ = false;
    std::atomic_bool computing_new_trajectory_ = false;

    std::thread waitForGeneratingNewTraj;

    void TodoThreadLoop();
    inline bool DynamicTrajectory::checkIfTrajectoryCanBeGenerated()
    {
      return !in_security_zone_ && !computing_new_trajectory_;
    }

  public:
    DynamicWaypoint::Deque generateWaypointsForTheNextTrajectory();
    // Default constructor
    DynamicTrajectory(){};

    // getters
    double getMaxTime();
    double getMinTime();
    inline mav_trajectory_generation::Vertex::Vector getWaypoints() const { return traj_.getWaypoints(); }
    inline mav_trajectory_generation::Segment::Vector getSegments() const { return traj_.getSegments(); }
    DynamicWaypoint::Deque getDynamicWaypoints() const;
    inline double getSpeed() const;

    // setters
    inline void setSpeed(double speed);
    void setWaypoints(const DynamicWaypoint::Vector &waypoints);

    void appendWaypoint(const DynamicWaypoint &waypoint);
    void modifyWaypoint(const std::string &name, const Eigen::Vector3d &position);
    bool evaluateTrajectory(const float &t, dynamic_traj_generator::References &refs, bool only_positions = false);

    void generateTrajectory(const DynamicWaypoint::DynamicWaypoint::Deque &waypoints, bool force);

  private:
    bool applyWaypointModification(const std::string &name, const Eigen::Vector3d &position);
    void calculateIndexDynamicWaypoints(DynamicWaypoint::Deque &dynamic_vector);
    inline void removeWaypoint();
    void modifyWaypointsBuffer();
    bool checkTrajectoryGenerated();

    ThreadSafeTrajectory computeTrajectory(const DynamicWaypoint::Deque &waypoints,
                                           const bool &lineal_optimization = false);
    void swapTrajectory();
    bool getRefs(const ThreadSafeTrajectory &traj,
                 double t_eval, dynamic_traj_generator::References &refs,
                 const bool only_position = false);
    void updateDynamicWaypoints(DynamicWaypoint::Deque &new_dyn_waypoints);

    /**
     * @brief
     *
     * @param current_traj reference to the current trajectory
     * @param last_t_evaluated last T evaluated, will be the beggining point of the new trajectory
     * @param waypoints new dynamic waypoints vector
     * @param TimeConstantAlgorithm Time constant (Ct) of computing trajectory: The complexity of the algoritm is O(f(n)) this means that it will spend at least t = Ct * f(n) sec
     */
    DynamicWaypoint::Deque DynamicTrajectory::stitchActualTrajectoryWithNewWaypoints(double last_t_evaluated,
                                                                                     const DynamicWaypoint::Deque &waypoints,
                                                                                     double TimeConstantAlgorithm);
  };

} // namespace mav_planning

#endif // __DYNAMIC_TRAJECTORY_HPP__
