#include "dynamic_trajectory.hpp"

namespace dynamic_traj_generator
{

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
    if (waypoint_index < 0 || waypoint_index >= segments.size())
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

  ThreadSafeTrajectory DynamicTrajectory::computeTrajectory(const mav_trajectory_generation::Vertex::Vector &vertices,
                                                            const float &max_speed,
                                                            const bool &lineal_optimization)
  {
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
    if (has_dynamic_waypoints_)
    {
      DYNAMIC_LOG("[!INFO!] Trajectory generated with dynamic waypoints");
      mav_trajectory_generation::Segment::Vector segments;
      trajectory->getSegments(&segments);
      for (auto &waypoint : temporal_dynamic_waypoints_)
      {
        DYNAMIC_LOG("AQUI");
        DYNAMIC_LOG(waypoint.getName());
        waypoint.setTime(getCummulativeTime(segments, waypoint.getIndex()));
      }
    }
    return ThreadSafeTrajectory(trajectory);
  };

  void DynamicTrajectory::generateTrajectoryFromScratch(const mav_trajectory_generation::Vertex::Vector &vertices,
                                                        const float &max_speed)
  {
    std::lock_guard<std::mutex> lock(future_mutex_);
    future_traj_ = std::async(std::launch::async, &DynamicTrajectory::computeTrajectory, this, vertices, max_speed, false);
  };

  bool DynamicTrajectory::evaluateTrajectory(const float &t, dynamic_traj_generator::References &refs, bool only_positions)
  {
    const std::lock_guard<std::mutex> lock(traj_mutex_);
    float t_eval = t + t_offset_;
    last_t_eval_ = t_eval;
    return getRefs(traj_, t_eval, refs, only_positions);
  }

  void DynamicTrajectory::swapTrajectory()
  {
    const std::lock_guard<std::mutex> lock(future_mutex_);
    traj_ = std::move(future_traj_.get());
    if (has_dynamic_waypoints_)
    {
      const std::lock_guard<std::mutex> lock2(dynamic_waypoints_mutex_);
      dynamic_waypoints_ = std::move(temporal_dynamic_waypoints_);
    }
    last_t_eval_ = 0.0f;
    DYNAMIC_LOG("Trajectory swapped");
  };

  bool DynamicTrajectory::getRefs(const ThreadSafeTrajectory &traj,
                                  float t_eval, dynamic_traj_generator::References &refs,
                                  const bool only_position)
  {

    if (t_eval < traj.getMinTime())
    {
      DYNAMIC_LOG("[!ERROR!] Time out of bounds");
      return false;
    }

    if (t_eval > traj.getMaxTime() - 0.01)
    {
      t_eval = traj.getMaxTime() - 0.01;
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

  void DynamicTrajectory::generateTrajectory(const mav_trajectory_generation::Vertex::Vector &waypoints, const float &max_speed)
  {
    has_dynamic_waypoints_ = false;
    selectProperTrajectoryGenerationMethod(waypoints, max_speed);
  };

  void DynamicTrajectory::generateTrajectory(const dynamic_traj_generator::DynamicWaypoint::Vector &waypoints, const float &max_speed)
  {
    has_dynamic_waypoints_ = true;
    mav_trajectory_generation::Vertex::Vector vertices(waypoints.size(), dimension_);
    temporal_dynamic_waypoints_.clear();
    temporal_dynamic_waypoints_.reserve(waypoints.size());
    int waypoint_index = 0;
    for (auto waypoint : waypoints)
    {
      if (waypoint.getName() != "")
      {
        waypoint.setIndex(waypoint_index);
        temporal_dynamic_waypoints_.emplace_back(waypoint);
      }
      vertices[waypoint_index] = waypoint.getVertex();
      waypoint_index++;
    }
    selectProperTrajectoryGenerationMethod(vertices, max_speed);
  };

} // namespace dynamic_traj_generator