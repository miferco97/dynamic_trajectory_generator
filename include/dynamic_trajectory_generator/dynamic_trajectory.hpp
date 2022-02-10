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

#define __SCREEN_OUTPUT__

#include "dynamic_waypoint.hpp"
#include "matplotlibcpp.h"
#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include "mav_trajectory_generation/trajectory.h"
#include "thread_safe_trajectory.hpp"
#include "utils/logging_utils.hpp"
#include "utils/traj_modifiers.hpp"

#define MAV_MAX_ACCEL (1 * 9.81f)
#define N_WAYPOINTS_TO_APPEND 1
#define SECURITY_COEF 0.9
#define TIME_CONSTANT 1.0
#define SECURITY_ZONE_MULTIPLIER 0.001  // TODO : This means no security zone

constexpr float AsyntoticComplexity(int n) {
  float value = n * n;  // TODO: CALCULATE THIS CORRECTLY
  return value;
}

namespace dynamic_traj_generator {

struct References {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;

  Eigen::Vector3d &operator[](int index) {
    switch (index) {
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

class DynamicTrajectory {
  private:
  struct NumericParameters {
    double last_t_eval = 0.0f;
    double t_offset = 0.0f;
    double last_evaluation_time_asked_for_ = 0.0f;
    double speed = 0.0f;
    double compensation_time = 0.0f;
    double last_evaluation_time_asked_for_before_generating_traj = 0.0f;
  } parameters_;

  // const int derivative_to_optimize_ =
  // mav_trajectory_generation::derivative_order::JERK;
  const int derivative_to_optimize_ =
      mav_trajectory_generation::derivative_order::ACCELERATION;
  // const int derivative_to_optimize_ =
  // mav_trajectory_generation::derivative_order::VELOCITY;

  ThreadSafeTrajectory traj_;
  std::future<ThreadSafeTrajectory> future_traj_;
  /* double t_compensation_ = 0.0f; */

  const int dimension_ = 3;
  std::atomic_bool from_scratch_ = true;
  const double a_max_ = MAV_MAX_ACCEL;

  mutable std::mutex traj_mutex_;
  mutable std::mutex future_mutex_;
  mutable std::mutex dynamic_waypoints_mutex_;
  mutable std::mutex parameters_mutex_;
  mutable std::mutex todo_mutex;

  dynamic_traj_generator::DynamicWaypoint::Deque
      dynamic_waypoints_;  // buffer to store waypoints not updated
  dynamic_traj_generator::DynamicWaypoint::Deque
      next_trajectory_waypoint_;  // buffer to store waypoints not updated

  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_be_added_;
  dynamic_traj_generator::DynamicWaypoint::Vector waypoints_to_be_set_;
  std::vector<std::pair<std::string, Eigen::Vector3d>>
      waypoints_to_be_modified_;

  std::atomic_bool generate_new_traj_ = false;
  /* std::atomic_bool in_security_zone_ = false; */
  std::atomic_bool computing_new_trajectory_ = false;
  std::atomic_bool stop_process_ = false;
  std::atomic_bool trajectory_regenerated_ = false;

  std::thread waitForGeneratingNewTraj_thread_;

  void todoThreadLoop();
  inline bool checkIfTrajectoryCanBeGenerated() {
    /* DYNAMIC_LOG("Checking if trajectory can be generated"); */
    return !checkInSecurityZone() && !computing_new_trajectory_;
  }
  double computeEvalTime(double t, bool for_plotting = false) {
    std::lock_guard<std::mutex> lock(parameters_mutex_);
    double eval_time = t - parameters_.compensation_time + parameters_.t_offset;
    if (!for_plotting) {
      parameters_.last_evaluation_time_asked_for_ = t;
      parameters_.last_t_eval = eval_time;
    }
    return eval_time;
  };

  bool stitchTrajectory() { return traj_ != nullptr; }
  double getTimeCompensation() {
    std::lock_guard<std::mutex> lock(parameters_mutex_);
    return parameters_.compensation_time + parameters_.t_offset;
  }
  bool checkInSecurityZone() {
    return false;
    const std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
    parameters_mutex_.lock();
    double last_t_eval = parameters_.last_t_eval;
    parameters_mutex_.unlock();

    for (auto &waypoint : dynamic_waypoints_) {
      if (waypoint.getName() != "" && waypoint.getTime() > last_t_eval) {
        const std::lock_guard<std::mutex> lock2(parameters_mutex_);
        if (waypoint.getTime() - parameters_.last_t_eval >
            SECURITY_ZONE_MULTIPLIER *
                computeSecurityTime(dynamic_waypoints_.size(), TIME_CONSTANT)) {
          return false;
          DYNAMIC_LOG("Not in security zone");
        } else {
          return true;
        }
      }
    }
    return false;
  };

  inline double computeSecurityTime(int n, double TimeConstant) {
    return TimeConstant * AsyntoticComplexity(n);
  }

  public:
  bool getWasTrajectoryRegenerated() {
    bool value = trajectory_regenerated_;
    trajectory_regenerated_ = false;
    return value;
  }
  DynamicWaypoint::Deque generateWaypointsForTheNextTrajectory();
  // Default constructor
  DynamicTrajectory() {
    waitForGeneratingNewTraj_thread_ =
        std::thread(&DynamicTrajectory::todoThreadLoop, this);
  }
  ~DynamicTrajectory() {
    stop_process_ = true;
    waitForGeneratingNewTraj_thread_.join();
  }

  // getters
  double getMaxTime();
  double getMinTime();
  inline mav_trajectory_generation::Vertex::Vector getWaypoints() const {
    return traj_.getWaypoints();
  }
  inline mav_trajectory_generation::Segment::Vector getSegments() const {
    return traj_.getSegments();
  }
  DynamicWaypoint::Deque getDynamicWaypoints();
  inline double getSpeed() const;

  // setters
  inline void setSpeed(double speed) {
    const std::lock_guard<std::mutex> lock(parameters_mutex_);
    parameters_.speed = speed;
  };
  void setWaypoints(const DynamicWaypoint::Vector &waypoints);

  void appendWaypoint(const DynamicWaypoint &waypoint);
  void modifyWaypoint(const std::string &name, const Eigen::Vector3d &position);
  bool evaluateForPlotting(const float &t,
                           dynamic_traj_generator::References &refs,
                           bool only_positions = false);
  bool evaluateTrajectory(const float &t,
                          dynamic_traj_generator::References &refs,
                          bool only_positions = false);
  void generateTrajectory(
      const DynamicWaypoint::DynamicWaypoint::Deque &waypoints, bool force);

  private:
  bool applyWaypointModification(const std::string &name,
                                 const Eigen::Vector3d &position);
  void calculateIndexDynamicWaypoints(DynamicWaypoint::Deque &dynamic_vector) {
    for (int i = 0; i < dynamic_vector.size(); i++) {
      dynamic_vector[i].setIndex(i);
    }
  };
  bool checkTrajectoryModifiers();
  bool checkTrajectoryGenerated();
  ThreadSafeTrajectory computeTrajectory(
      const DynamicWaypoint::Deque &waypoints,
      const bool &lineal_optimization = false);
  void swapTrajectory();
  void swapDynamicWaypoints();
  bool getRefs(const ThreadSafeTrajectory &traj, double t_eval,
               dynamic_traj_generator::References &refs,
               const bool only_position = false);
  void filterPassedWaypoints(DynamicWaypoint::Deque &waypoints);
  /* void updateDynamicWaypoints(DynamicWaypoint::Deque &new_dyn_waypoints); */

  /**
   * @brief
   *
   * @param current_traj reference to the current trajectory
   * @param last_t_evaluated last T evaluated, will be the beggining point of
   * the new trajectory
   * @param waypoints new dynamic waypoints vector
   * @param TimeConstantAlgorithm Time constant (Ct) of computing trajectory:
   * The complexity of the algoritm is O(f(n)) this means that it will spend at
   * least t = Ct * f(n) sec
   */
  DynamicWaypoint::Deque stitchActualTrajectoryWithNewWaypoints(
      double last_t_evaluated, const DynamicWaypoint::Deque &waypoints,
      double TimeConstantAlgorithm);
};  // namespace dynamic_traj_generator

}  // namespace dynamic_traj_generator

#endif  // __DYNAMIC_TRAJECTORY_HPP__
