#include "dynamic_trajectory.hpp"

#include <future>
#include <mutex>

#include "utils/logging_utils.hpp"

namespace dynamic_traj_generator {

mav_trajectory_generation::Vertex::Vector extractVerticesFromWaypoints(
    const dynamic_traj_generator::DynamicWaypoint::Deque &waypoints) {
  mav_trajectory_generation::Vertex::Vector vertices;
  vertices.reserve(waypoints.size());
  for (auto &waypoint : waypoints) {
    vertices.emplace_back(waypoint.getVertex());
  }
  return vertices;
}

bool DynamicTrajectory::checkTrajectoryGenerated() {
  while (traj_ == nullptr) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    const std::lock_guard<std::mutex> lock(future_mutex_);
    if (future_traj_.valid()) {
      future_traj_.wait();
      break;
    }
  }
  future_mutex_.lock();
  bool future_traj_valid = future_traj_.valid();
  future_mutex_.unlock();
  if (computing_new_trajectory_ && future_traj_valid) {
    using namespace std::chrono_literals;
    future_mutex_.lock();
    bool traj_ready = future_traj_.wait_for(0ms) == std::future_status::ready;
    future_mutex_.unlock();
    if (traj_ready) {
      swapTrajectory();
      swapDynamicWaypoints();
      computing_new_trajectory_ = false;
      generate_new_traj_ = false;
    }
  }
  /* DYNAMIC_LOG("check Trajectory Generated"); */
  /* else { */
  /*   DYNAMIC_LOG("[!ERROR!] Trajectory not initialized"); */
  /*   return false; */
  /* } */
  return true;
}

double getCummulativeTime(
    const mav_trajectory_generation::Segment::Vector &segments,
    const int &waypoint_index) {
  if (waypoint_index < 0 || waypoint_index > segments.size()) {
    throw std::out_of_range("[GetCumulativeTime]: Waypoint index out of range");
  }
  double value = 0.0f;
  for (int i = waypoint_index; i > 0; i--) {
    value += segments[i - 1].getTime();
  }
  return value;
}

ThreadSafeTrajectory DynamicTrajectory::computeTrajectory(
    const DynamicWaypoint::Deque &waypoints, const bool &lineal_optimization) {
  parameters_mutex_.lock();
  float max_speed = parameters_.speed;
  parameters_mutex_.unlock();

  auto vertices = extractVerticesFromWaypoints(waypoints);
  if (vertices.size() < 2) {
    throw std::runtime_error("Not enough waypoints");
  }
  const int N = 10;
  std::shared_ptr<mav_trajectory_generation::Trajectory> trajectory =
      std::make_shared<mav_trajectory_generation::Trajectory>();
  auto segment_times = mav_trajectory_generation::estimateSegmentTimes(
      vertices, max_speed, this->a_max_);
  // Optimizer
  if (lineal_optimization) {
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension_);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
    opt.solveLinear();
    opt.getTrajectory(
        (mav_trajectory_generation::Trajectory *)trajectory.get());
  } else {
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.max_iterations = 2000;
    parameters.f_rel = 0.05;
    parameters.x_rel = 0.1;
    parameters.time_penalty = 200.0;
    parameters.initial_stepsize_rel = 0.1;
    // parameters.inequality_constraint_tolerance = 0.1;
    parameters.inequality_constraint_tolerance = 0.2;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(
        dimension_, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
    opt.addMaximumMagnitudeConstraint(
        mav_trajectory_generation::derivative_order::VELOCITY, max_speed);
    opt.addMaximumMagnitudeConstraint(
        mav_trajectory_generation::derivative_order::ACCELERATION, a_max_);
    opt.optimize();
    opt.getTrajectory(
        (mav_trajectory_generation::Trajectory *)trajectory.get());
  }
  mav_trajectory_generation::Segment::Vector segments;
  trajectory->getSegments(&segments);
  int index = 0;
  for (auto &waypoint : next_trajectory_waypoint_) {
    waypoint.setIndex(index);
    waypoint.setTime(getCummulativeTime(segments, waypoint.getIndex()));
    index++;
  }
  return ThreadSafeTrajectory(std::move(trajectory));
};

bool DynamicTrajectory::evaluateForPlotting(
    const float &t, dynamic_traj_generator::References &refs,
    bool only_positions) {
  float t_eval = computeEvalTime(t, true);
  return getRefs(traj_, t_eval, refs, only_positions);
};

bool DynamicTrajectory::evaluateTrajectory(
    const float &t, dynamic_traj_generator::References &refs,
    bool only_positions) {
  // const std::lock_guard<std::mutex> lock(traj_mutex_);
  return getRefs(traj_, computeEvalTime(t), refs, only_positions);
}

void DynamicTrajectory::generateTrajectory(
    const DynamicWaypoint::DynamicWaypoint::Deque &waypoints, bool force) {
  computing_new_trajectory_ = true;
  std::lock_guard<std::mutex> lock(future_mutex_);
  future_traj_ =
      std::async(std::launch::async, &DynamicTrajectory::computeTrajectory,
                 this, waypoints, force);
};

void DynamicTrajectory::swapTrajectory() {
  const std::lock_guard<std::mutex> lock(future_mutex_);
  traj_ = std::move(future_traj_.get());
  const std::lock_guard<std::mutex> lock3(parameters_mutex_);
  parameters_.compensation_time = parameters_.last_evaluation_time_asked_for_;
  parameters_.t_offset =
      parameters_.last_evaluation_time_asked_for_ -
      parameters_.last_evaluation_time_asked_for_before_generating_traj + 0.3f; // FIXME: THIS VALUE MUST NOT BE HARDCODED
  parameters_.last_evaluation_time_asked_for_before_generating_traj = 0.0f;
  DYNAMIC_LOG(parameters_.compensation_time);
  parameters_.last_t_eval = 0.0f;
  trajectory_regenerated_ = true;
  DYNAMIC_LOG("Trajectory swapped");
};

void DynamicTrajectory::swapDynamicWaypoints() {
  const std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
  dynamic_waypoints_ = next_trajectory_waypoint_;
  DYNAMIC_LOG("Waypoints swapped");
}

bool DynamicTrajectory::getRefs(const ThreadSafeTrajectory &traj, double t_eval,
                                dynamic_traj_generator::References &refs,
                                const bool only_position) {
  checkTrajectoryGenerated();
  if (t_eval < traj.getMinTime()) {
    parameters_mutex_.lock();
    t_eval = traj.getMinTime();
    parameters_mutex_.unlock();
    DYNAMIC_LOG("[WARN] Time out of bounds");
    return false;
  }
  if (t_eval > traj.getMaxTime()) {
    t_eval = traj.getMaxTime();
  }
  dynamic_waypoints_mutex_.lock();
  short int max_index = 3;
  if (only_position) {
    max_index = 1;
  }
  for (short int i = 0; i < max_index; i++) {
    refs[i] = traj.evaluate(t_eval, i);
    for (auto &waypoint : dynamic_waypoints_) {
      if (waypoint.getName() == "") {
        continue;
      }
      refs[i] += waypoint.trajectoryCompensation(t_eval, i);
    }
  }
  dynamic_waypoints_mutex_.unlock();
  return true;
}

double DynamicTrajectory::getMaxTime() {
  if (!checkTrajectoryGenerated()) return 0.0;
  std::lock_guard<std::mutex> lock(parameters_mutex_);
  return traj_.getMaxTime() + parameters_.compensation_time;
}

double DynamicTrajectory::getMinTime() {
  if (!checkTrajectoryGenerated()) return 0.0;
  return traj_.getMinTime() + parameters_.compensation_time;
}

DynamicWaypoint::Deque DynamicTrajectory::getDynamicWaypoints() {
  checkTrajectoryGenerated();
  std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
  return dynamic_waypoints_;
};

inline double DynamicTrajectory::getSpeed() const {
  const std::lock_guard<std::mutex> lock(parameters_mutex_);
  return parameters_.speed;
};

bool DynamicTrajectory::applyWaypointModification(
    const std::string &name, const Eigen::Vector3d &position) {
  bool modified = false;
  std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
  for (auto &waypoint : dynamic_waypoints_) {
    if (waypoint.getName() == name && waypoint.getName() != "") {
      const std::lock_guard<std::mutex> lock3(parameters_mutex_);
      waypoint.setCurrentPosition(position, parameters_.last_t_eval);
      modified = true;
      trajectory_regenerated_ = true;
    }
  }
  return modified;
};

DynamicWaypoint::Deque
DynamicTrajectory::stitchActualTrajectoryWithNewWaypoints(
    double last_t_evaluated, const DynamicWaypoint::Deque &waypoints,
    double TimeConstantAlgorithm) {
  // First step is to calculate new waypoints.
  int n_waypoints = N_WAYPOINTS_TO_APPEND + waypoints.size();
  DynamicWaypoint::Deque new_waypoints;
  // Append the previous points to generate a smooth stitching
  parameters_mutex_.lock();
  parameters_.last_evaluation_time_asked_for_before_generating_traj =
      parameters_.last_evaluation_time_asked_for_;
  parameters_mutex_.unlock();

  for (int i_waypoints_appended = 0;
       i_waypoints_appended < N_WAYPOINTS_TO_APPEND; i_waypoints_appended++) {
    dynamic_traj_generator::References references;
    mav_trajectory_generation::Vertex vertex(3);

    double eval_t = last_t_evaluated +
                    SECURITY_COEF *
                        (i_waypoints_appended / N_WAYPOINTS_TO_APPEND) *
                        computeSecurityTime(n_waypoints, TimeConstantAlgorithm);
    if (i_waypoints_appended == 0) {
      getRefs(traj_, eval_t, references);
      for (int i = 0; i < 3; i++) vertex.addConstraint(i, references[i]);
    } else {
      getRefs(traj_, eval_t, references, true);
      vertex.addConstraint(
          mav_trajectory_generation::derivative_order::POSITION,
          references.position);
    }
    new_waypoints.emplace_back(vertex);
  }
  // append the rest of the waypoints
  for (auto &waypoint : waypoints) {
    new_waypoints.emplace_back(waypoint);
  }
  return new_waypoints;
};

static void updateDynamicWaypointsPosition(
    DynamicWaypoint::Deque &dynamic_waypoints) {
  for (auto &waypoint : dynamic_waypoints) {
    waypoint.resetWaypointWithCurrentPosition();
  }
}

// void DynamicTrajectory::calculateIndexDynamicWaypoints(DynamicWaypoint::Deque
// &dynamic_vector)
// {
//   for (int i = 0; i < dynamic_waypoints_.size(); i++)
//   {
//     dynamic_waypoints_[i].setIndex(i);
//   }
// };

void DynamicTrajectory::setWaypoints(const DynamicWaypoint::Vector &waypoints) {
  const std::lock_guard<std::mutex> lock(todo_mutex);
  DYNAMIC_LOG("Setting waypoints");
  waypoints_to_be_set_.clear();
  waypoints_to_be_modified_.clear();
  waypoints_to_be_added_.clear();
  waypoints_to_be_set_.reserve(waypoints.size());
  for (auto &waypoint : waypoints) {
    waypoints_to_be_set_.emplace_back(waypoint);
  }
  generate_new_traj_ = true;
};

void DynamicTrajectory::appendWaypoint(const DynamicWaypoint &waypoint) {
  const std::lock_guard<std::mutex> lock(todo_mutex);
  waypoints_to_be_added_.emplace_back(waypoint);
  generate_new_traj_ = true;
}

void DynamicTrajectory::modifyWaypoint(const std::string &name,
                                       const Eigen::Vector3d &position) {
  const std::lock_guard<std::mutex> lock(todo_mutex);
  // check if the waypoint is already in the waypoints to be modified list
  auto iter = std::find_if(
      waypoints_to_be_modified_.begin(), waypoints_to_be_modified_.end(),
      [&name](const std::pair<std::string, Eigen::Vector3d> &waypoint) {
        return waypoint.first == name;
      });
  if (iter == waypoints_to_be_modified_.end()) {
    waypoints_to_be_modified_.emplace_back(name, position);
  } else {
    iter->first = name;
    iter->second = position;
  }
  generate_new_traj_ = true;
};

void resetWaypointThroughDeque(DynamicWaypoint::Deque &waypoints,
                               const std::string &name,
                               const Eigen::Vector3d &position) {
  if (name == "") {
    return;
  }
  for (auto &waypoint : waypoints) {
    if (waypoint.getName() == name) {
      waypoint.resetWaypoint(position);
    }
  }
};

bool DynamicTrajectory::checkTrajectoryModifiers() {
  std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
  for (auto &waypoint : dynamic_waypoints_) {
    if (waypoint.hasModifiers()) return true;
  }
  return false;
};

void DynamicTrajectory::todoThreadLoop() {
  while (!stop_process_) {
    if (generate_new_traj_ && checkIfTrajectoryCanBeGenerated()) {
      next_trajectory_waypoint_ = generateWaypointsForTheNextTrajectory();
      // TODO: FIX WAYPOINTS==1
      if (next_trajectory_waypoint_.size() == 0 ||
          next_trajectory_waypoint_.size() == 1) {
        DYNAMIC_LOG("No waypoints to generate a new trajectory");
        generate_new_traj_ = false;
        continue;
      }
      DYNAMIC_LOG("generate new trajectory");
      if (stitchTrajectory()) {
        DYNAMIC_LOG("stitching new trajectory");
        next_trajectory_waypoint_ = stitchActualTrajectoryWithNewWaypoints(
            parameters_.last_t_eval, next_trajectory_waypoint_,
            computeSecurityTime(next_trajectory_waypoint_.size(),
                                TIME_CONSTANT));
      }
      DYNAMIC_LOG(next_trajectory_waypoint_.size());
      generateTrajectory(next_trajectory_waypoint_, parameters_.speed);
      generate_new_traj_ = false;
    }

    /* else if (waypoints_to_be_modified_.size() && !computing_new_trajectory_)
       { */
    else if (false) {
      /* DYNAMIC_LOG("Modifying waypoints"); */
      const std::lock_guard<std::mutex> lock(todo_mutex);
      for (auto &modification : waypoints_to_be_modified_) {
        bool modified =
            applyWaypointModification(modification.first, modification.second);
        if (modified) {
          /* DYNAMIC_LOG(modification.first); */
        }
      }
      waypoints_to_be_modified_.clear();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  return;
}

void DynamicTrajectory::filterPassedWaypoints(
    DynamicWaypoint::Deque &waypoints) {
  parameters_mutex_.lock();
  double last_t_eval = parameters_.last_t_eval;
  DYNAMIC_LOG(last_t_eval);
  parameters_mutex_.unlock();
  for (auto it = waypoints.begin(); it != waypoints.end();) {
    if (it->getTime() < last_t_eval) {
      DYNAMIC_LOG("Removing waypoint ");
      it = waypoints.erase(it);
    } else {
      ++it;
    }
  }
}

DynamicWaypoint::Deque
DynamicTrajectory::generateWaypointsForTheNextTrajectory() {
  DYNAMIC_LOG("Generating waypoints for the next trajectory");
  const std::lock_guard<std::mutex> lock(todo_mutex);
  DynamicWaypoint::Deque next_trajectory_waypoints;
  if (waypoints_to_be_set_.size() == 0) {
    /* if (waypoints_to_be_added_.size() == 0 && !checkTrajectoryModifiers()) {
     */
    /*   return next_trajectory_waypoints; */
    /* } */
    next_trajectory_waypoints = (dynamic_waypoints_);
    updateDynamicWaypointsPosition(next_trajectory_waypoints);
  } else {
    for (auto &waypoint : waypoints_to_be_set_)
      next_trajectory_waypoints.emplace_back(waypoint);
  }
  for (auto &waypoint : waypoints_to_be_added_) {
    next_trajectory_waypoints.emplace_back(waypoint);
  }
  for (auto &waypoint : waypoints_to_be_modified_) {
    resetWaypointThroughDeque(next_trajectory_waypoints, waypoint.first,
                              waypoint.second);
  }
  waypoints_to_be_set_.clear();
  waypoints_to_be_added_.clear();
  waypoints_to_be_modified_.clear();
  filterPassedWaypoints(next_trajectory_waypoints);
  return next_trajectory_waypoints;
}

}  // namespace dynamic_traj_generator
