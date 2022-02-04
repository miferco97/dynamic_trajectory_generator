#include "dynamic_trajectory.hpp"

namespace dynamic_traj_generator
{

  mav_trajectory_generation::Vertex::Vector extractVerticesFromWaypoints(const dynamic_traj_generator::DynamicWaypoint::Deque &waypoints)
  {
    mav_trajectory_generation::Vertex::Vector vertices;
    vertices.reserve(waypoints.size());
    for (auto &waypoint : waypoints)
    {
      vertices.emplace_back(waypoint.getVertex());
    }
    return vertices;
  }

  bool DynamicTrajectory::checkTrajectoryGenerated()
  {
    if (traj_ == nullptr)
    {
      future_mutex_.lock();
      if (future_traj_.valid())
      {
        future_traj_.wait();
        future_mutex_.unlock();
        swapTrajectory();
      }
      else
      {
        DYNAMIC_LOG("[!ERROR!] Trajectory not initialized");
        future_mutex_.unlock();
        return false;
      }
    }
    return true;
  }

  double getCummulativeTime(const mav_trajectory_generation::Segment::Vector &segments, const int &waypoint_index)
  {
    if (waypoint_index < 0 || waypoint_index > segments.size())
    {
      throw std::out_of_range("[GetCumulativeTime]: Waypoint index out of range");
    }
    double value = 0.0f;
    for (int i = waypoint_index; i > 0; i--)
    {
      value += segments[i - 1].getTime();
    }
    return value;
  }

  ThreadSafeTrajectory DynamicTrajectory::computeTrajectory(const DynamicWaypoint::Deque &waypoints,
                                                            const bool &lineal_optimization)
  {
    parameters_mutex.lock();
    float max_speed = parameters_.speed;
    parameters_mutex.unlock();

    auto vertices = extractVerticesFromWaypoints(waypoints);
    if (vertices.size() < 2)
    {
      throw std::runtime_error("Not enough waypoints");
    }
    const int N = 10;
    std::shared_ptr<mav_trajectory_generation::Trajectory> trajectory = std::make_shared<mav_trajectory_generation::Trajectory>();
    auto segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices, max_speed, this->a_max_);
    // Optimizer
    if (lineal_optimization)
    {
      mav_trajectory_generation::PolynomialOptimization<N> opt(dimension_);
      opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
      opt.solveLinear();
      opt.getTrajectory((mav_trajectory_generation::Trajectory *)trajectory.get());
    }
    else
    {
      mav_trajectory_generation::NonlinearOptimizationParameters parameters;
      parameters.max_iterations = 2000;
      parameters.f_rel = 0.05;
      parameters.x_rel = 0.1;
      parameters.time_penalty = 200.0;
      parameters.initial_stepsize_rel = 0.1;
      // parameters.inequality_constraint_tolerance = 0.1;
      parameters.inequality_constraint_tolerance = 0.2;
      mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension_, parameters);
      opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
      opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_speed);
      opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max_);
      opt.optimize();
      opt.getTrajectory((mav_trajectory_generation::Trajectory *)trajectory.get());
    }
    DYNAMIC_LOG("[!INFO!] Trajectory generated with dynamic waypoints");
    mav_trajectory_generation::Segment::Vector segments;
    trajectory->getSegments(&segments);
    dynamic_waypoints_mutex_.lock();
    for (auto &waypoint : dynamic_waypoints_)
    {
      DYNAMIC_LOG(waypoint.getName());
      waypoint.setTime(getCummulativeTime(segments, waypoint.getIndex()));
    }
    dynamic_waypoints_mutex_.unlock();
    computing_new_trajectory_ = false;
    return ThreadSafeTrajectory(trajectory);
  };

  bool DynamicTrajectory::evaluateTrajectory(const float &t, dynamic_traj_generator::References &refs, bool only_positions)
  {
    const std::lock_guard<std::mutex> lock(traj_mutex_);
    parameters_mutex.lock();
    float t_eval = t + parameters_.t_offset;
    parameters_.last_t_eval = t_eval;
    parameters_mutex.unlock();
    return getRefs(traj_, t_eval, refs, only_positions);
  }

  void DynamicTrajectory::generateTrajectory(const DynamicWaypoint::DynamicWaypoint::Deque &waypoints, bool force)
  {
    std::lock_guard<std::mutex> lock(future_mutex_);
    future_traj_ = std::async(std::launch::async, &DynamicTrajectory::computeTrajectory, this, waypoints, force);
  };

  void DynamicTrajectory::swapTrajectory()
  {
    const std::lock_guard<std::mutex> lock(future_mutex_);
    traj_ = std::move(future_traj_.get());
    const std::lock_guard<std::mutex> lock2(dynamic_waypoints_mutex_);
    const std::lock_guard<std::mutex> lock3(parameters_mutex);
    parameters_.last_t_eval = 0.0f;
    DYNAMIC_LOG("Trajectory swapped");
  };

  bool DynamicTrajectory::getRefs(const ThreadSafeTrajectory &traj,
                                  double t_eval, dynamic_traj_generator::References &refs,
                                  const bool only_position)
  {
    if (t_eval < traj.getMinTime())
    {
      DYNAMIC_LOG("[!ERROR!] Time out of bounds");
      return false;
    }
    if (t_eval > traj.getMaxTime())
    {
      t_eval = traj.getMaxTime();
    }
    dynamic_waypoints_mutex_.lock();
    short int max_index = 3;
    if (only_position)
    {
      max_index = 1;
    }
    for (short int i = 0; i < max_index; i++)
    {
      refs[i] = traj.evaluate(t_eval, i);
      for (auto &waypoint : dynamic_waypoints_)
      {
        refs[i] += waypoint.trajectoryCompensation(t_eval, i);
      }
    }
    dynamic_waypoints_mutex_.unlock();
    return true;
  }

  double DynamicTrajectory::getMaxTime()
  {
    if (!checkTrajectoryGenerated())
      return 0.0;
    return traj_.getMaxTime();
  }

  double DynamicTrajectory::getMinTime()
  {
    if (!checkTrajectoryGenerated())
      return 0.0;
    return traj_.getMinTime();
  }

  DynamicWaypoint::Deque DynamicTrajectory::getDynamicWaypoints() const
  {
    std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
    return dynamic_waypoints_;
  };

  inline double DynamicTrajectory::getSpeed() const
  {
    const std::lock_guard<std::mutex> lock(parameters_mutex);
    return parameters_.speed;
  };

  inline void DynamicTrajectory::setSpeed(double speed)
  {
    const std::lock_guard<std::mutex> lock(parameters_mutex);
    parameters_.speed = speed;
  };

  bool DynamicTrajectory::applyWaypointModification(const std::string &name, const Eigen::Vector3d &position)
  {
    bool modified = false;
    std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
    for (auto &waypoint : dynamic_waypoints_)
    {
      if (waypoint.getName() == name)
      {
        DYNAMIC_LOG("Modifying waypoint");
        DYNAMIC_LOG(waypoint.getName());

        const std::lock_guard<std::mutex> lock3(parameters_mutex);
        waypoint.setCurrentPosition(position, parameters_.last_t_eval);
        modified = true;
      }
    }
    return modified;
  };

  inline double computeSecurityTime(int n, double TimeConstant) { return TimeConstant * AsyntoticComplexity(n); }

  DynamicWaypoint::Deque DynamicTrajectory::stitchActualTrajectoryWithNewWaypoints(double last_t_evaluated,
                                                                                   const DynamicWaypoint::Deque &waypoints,
                                                                                   double TimeConstantAlgorithm)
  {
    // First step is to calculate new waypoints.
    int n_waypoints = N_WAYPOINTS_TO_APPEND + waypoints.size();
    DynamicWaypoint::Deque new_waypoints;
    // Append the previous points to generate a smooth stitching
    for (int i_waypoints_appended = 0; i_waypoints_appended < N_WAYPOINTS_TO_APPEND; i_waypoints_appended++)
    {
      dynamic_traj_generator::References references;
      mav_trajectory_generation::Vertex vertex(3);
      double eval_t = last_t_evaluated + SECURITY_COEF * (i_waypoints_appended / N_WAYPOINTS_TO_APPEND) * computeSecurityTime(n_waypoints, TimeConstantAlgorithm);
      if (i_waypoints_appended == 0)
      {
        getRefs(traj_, eval_t, references);
        for (int i = 0; i < 3; i++)
          vertex.addConstraint(i, references[i]);
      }
      else
      {
        getRefs(traj_, eval_t, references, true);
        vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, references.position);
      }
      new_waypoints.emplace_back(vertex);
    }
    // append the rest of the waypoints
    for (auto &waypoint : waypoints)
    {
      new_waypoints.emplace_back(waypoint);
    }
    return new_waypoints;
  };

  static void updateDynamicWaypointsPosition(DynamicWaypoint::Deque &dynamic_waypoints)
  {
    for (auto &waypoint : dynamic_waypoints)
    {
      waypoint.resetWaypointWithCurrentPosition();
    }
  }

  inline void DynamicTrajectory::removeWaypoint()
  {
    const std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
    dynamic_waypoints_.pop_front();
  }

  // void DynamicTrajectory::calculateIndexDynamicWaypoints(DynamicWaypoint::Deque &dynamic_vector)
  // {
  //   for (int i = 0; i < dynamic_waypoints_.size(); i++)
  //   {
  //     dynamic_waypoints_[i].setIndex(i);
  //   }
  // };

  void DynamicTrajectory::setWaypoints(const DynamicWaypoint::Vector &waypoints)
  {
    const std::lock_guard<std::mutex> lock(todo_mutex);
    waypoints_to_be_set_.clear();
    waypoints_to_be_modified_.clear();
    waypoints_to_be_added_.clear();
    waypoints_to_be_set_.reserve(waypoints.size());
    for (auto &waypoint : waypoints)
    {
      waypoints_to_be_set_.emplace_back(waypoint);
    }
  };

  void DynamicTrajectory::appendWaypoint(const DynamicWaypoint &waypoint)
  {
    const std::lock_guard<std::mutex> lock(todo_mutex);
    waypoints_to_be_added_.emplace_back(waypoint);
  }

  void DynamicTrajectory::modifyWaypoint(const std::string &name, const Eigen::Vector3d &position)
  {
    const std::lock_guard<std::mutex> lock(todo_mutex);
    // check if the waypoint is already in the waypoints to be modified list
    auto iter = std::find_if(waypoints_to_be_modified_.begin(),
                             waypoints_to_be_modified_.end(),
                             [&name](const std::pair<std::string, Eigen::Vector3d> &waypoint)
                             { return waypoint.first == name; });
    if (iter == waypoints_to_be_modified_.end())
    {
      waypoints_to_be_modified_.emplace_back(name, position);
    }
    else
    {
      iter->first = name;
      iter->second = position;
    }
  };

  void resetWaypointThroughDeque(DynamicWaypoint::Deque &waypoints, const std::string &name, const Eigen::Vector3d &position)
  {
    if (name == "")
    {
      return;
    }
    for (auto &waypoint : waypoints)
    {
      if (waypoint.getName() == name)
      {
        waypoint.resetWaypoint(position);
      }
    }
  };

  bool stitchTrajectory() { return true; }
  void DynamicTrajectory::TodoThreadLoop()
  {
    if (generate_new_traj_)
    {
      next_trajectory_waypoint_ = generateWaypointsForTheNextTrajectory();
      if (stitchTrajectory())
      {
        DynamicWaypoint::Deque new_waypoints = stitchActualTrajectoryWithNewWaypoints(parameters_.last_t_eval,
                                                                                      next_trajectory_waypoint_,
                                                                                      computeSecurityTime(next_trajectory_waypoint_.size(), TIME_CONSTANT));
        next_trajectory_waypoint_ = new_waypoints;
      }
      generateTrajectory(next_trajectory_waypoint_, parameters_.speed);
    }
  }

  DynamicWaypoint::Deque DynamicTrajectory::generateWaypointsForTheNextTrajectory()
  {
    const std::lock_guard<std::mutex> lock(todo_mutex);
    DynamicWaypoint::Deque next_trajectory_waypoints;
    if (waypoints_to_be_set_.size() == 0)
    {
      next_trajectory_waypoints = (dynamic_waypoints_);
      updateDynamicWaypointsPosition(next_trajectory_waypoint_);
    }
    else
    {
      for (auto &waypoint : waypoints_to_be_set_)
        next_trajectory_waypoint_.emplace_back(waypoint);
    }
    for (auto &waypoint : waypoints_to_be_added_)
    {
      next_trajectory_waypoints.emplace_back(waypoint);
    }
    for (auto &waypoint : waypoints_to_be_modified_)
    {
      resetWaypointThroughDeque(next_trajectory_waypoints, waypoint.first, waypoint.second);
    }
    waypoints_to_be_set_.clear();
    waypoints_to_be_added_.clear();
    waypoints_to_be_modified_.clear();

    return next_trajectory_waypoints;
  }

} // namespace dynamic_traj_generator
