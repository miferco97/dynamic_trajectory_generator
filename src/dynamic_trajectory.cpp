#include "dynamic_trajectory.hpp"

#include <future>
#include <mutex>

#include "dynamic_waypoint.hpp"
#include "utils/logging_utils.hpp"

namespace dynamic_traj_generator
{

  /******************************************************************************/
  /***************************** STATIC FUNCTIONS *******************************/
  /******************************************************************************/

  static mav_trajectory_generation::Vertex::Vector extractVerticesFromWaypoints(
      const dynamic_traj_generator::DynamicWaypoint::Deque &waypoints)
  {
    mav_trajectory_generation::Vertex::Vector vertices;
    vertices.reserve(waypoints.size());
    for (auto &waypoint : waypoints)
    {
      vertices.emplace_back(waypoint.getVertex());
    }
    vertices[vertices.size() - 1].addConstraint(1, Eigen::Vector3d(0, 0, 0));
    vertices[vertices.size() - 1].addConstraint(2, Eigen::Vector3d(0, 0, 0));
    return vertices;
  }

  static double getCummulativeTime(const mav_trajectory_generation::Segment::Vector &segments,
                                   const int &waypoint_index)
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

  static void calculateIndexDynamicWaypoints(DynamicWaypoint::Deque &dynamic_vector)
  {
    for (int i = 0; i < dynamic_vector.size(); i++)
    {
      dynamic_vector[i].setIndex(i);
    }
  };

  /******************************************************************************/
  /***************************** PUBLIC INTERFACE *******************************/
  /******************************************************************************/

  void DynamicTrajectory::setWaypoints(const DynamicWaypoint::Vector &waypoints)
  {
    const std::lock_guard<std::mutex> lock(todo_mutex);
    DYNAMIC_LOG("Setting waypoints");
    waypoints_to_be_set_.clear();
    waypoints_to_be_modified_.clear();
    waypoints_to_be_added_.clear();
    waypoints_to_be_set_.reserve(waypoints.size());
    for (auto &waypoint : waypoints)
    {
      waypoints_to_be_set_.emplace_back(waypoint);
    }
    generate_new_traj_ = true;
  };

  void DynamicTrajectory::appendWaypoint(const DynamicWaypoint &waypoint)
  {
    const std::lock_guard<std::mutex> lock(todo_mutex);
    waypoints_to_be_added_.emplace_back(waypoint);
    generate_new_traj_ = true;
  }

  void DynamicTrajectory::modifyWaypoint(const std::string &name, const Eigen::Vector3d &position)
  {
    const std::lock_guard<std::mutex> lock(todo_mutex);
    DYNAMIC_LOG("Modifying waypoint: ");
    // check if the waypoint is already in the waypoints to be modified list
    auto iter = std::find_if(waypoints_to_be_modified_.begin(), waypoints_to_be_modified_.end(),
                             [&name](const std::pair<std::string, Eigen::Vector3d> &waypoint)
                             {
                               return waypoint.first == name;
                             });
    if (iter == waypoints_to_be_modified_.end())
    {
      waypoints_to_be_modified_.emplace_back(name, position);
    }
    else
    {
      iter->first = name;
      iter->second = position;
    }
    generate_new_traj_ = true;
  };

  bool DynamicTrajectory::evaluateTrajectory(const float &t, dynamic_traj_generator::References &refs,
                                             bool only_positions, bool for_plotting)
  {
    /* if (!checkIfTrajectoryIsAlreadyGenerated()) return false; */
    checkTrajectoryGenerated();

    double global_time = t;
    double local_time = convertFromGlobalTime(t);
    if (!for_plotting)
    {
      parameters_mutex_.lock();
      parameters_.last_global_time_evaluated = global_time;
      parameters_.last_local_time_evaluated = local_time;
      parameters_mutex_.unlock();
    }
    refs = getReferences(traj_, global_time, local_time, only_positions);
    // DYNAMIC_LOG("returning evaluate trajectory");
    return true;
  }

  References DynamicTrajectory::getReferences(const ThreadSafeTrajectory &traj, double global_time,
                                              double local_time, const bool only_positions)
  {
    References refs;
    for (short int i = 0; i < 3; i++)
    {
      refs[i] = evaluateModifiedTrajectory(traj, global_time, local_time, i);
      if (only_positions)
      {
        break;
      }
    }
    return refs;
  };

  /******************************************************************************/
  /**************************** GETTERS & SETTERS *******************************/
  /******************************************************************************/

  double DynamicTrajectory::getMaxTime()
  {
    if (!checkIfTrajectoryIsAlreadyGenerated())
    {
      waitUntilTrajectoryIsGenerated();
    }
    return convertIntoGlobalTime(traj_.getMaxTime());
  }

  double DynamicTrajectory::getMinTime()
  {
    if (!checkIfTrajectoryIsAlreadyGenerated())
    {
      waitUntilTrajectoryIsGenerated();
    }
    return convertIntoGlobalTime(traj_.getMinTime());
  }

  DynamicWaypoint::Deque DynamicTrajectory::getDynamicWaypoints()
  {
    checkTrajectoryGenerated();
    std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
    return dynamic_waypoints_;
  };

  double DynamicTrajectory::getSpeed() const
  {
    const std::lock_guard<std::mutex> lock(parameters_mutex_);
    return parameters_.speed;
  };

  /* mav_trajectory_generation::Vertex::Vector DynamicTrajectory::getWaypoints() const { */
  /*   return traj_.getWaypoints(); */
  /* } */
  /* mav_trajectory_generation::Segment::Vector DynamicTrajectory::getSegments() const { */
  /* return traj_.getSegments(); */
  /* } */

  void DynamicTrajectory::setSpeed(double speed)
  {
    const std::lock_guard<std::mutex> lock(parameters_mutex_);
    parameters_.speed = speed;
  };

  double DynamicTrajectory::getTimeCompensation()
  {
    std::lock_guard<std::mutex> lock(parameters_mutex_);
    return parameters_.t_offset;
  }

  bool DynamicTrajectory::getWasTrajectoryRegenerated()
  {
    bool value = trajectory_regenerated_;
    trajectory_regenerated_ = false;
    return value;
  }
  /******************************************************************************/
  /***************************** PRIVATE INTERFACE ******************************/
  /******************************************************************************/

  double DynamicTrajectory::convertFromGlobalTime(double t)
  {
    std::lock_guard<std::mutex> lock(parameters_mutex_);
    return t - parameters_.global_time_last_trajectory_generated;
  };

  double DynamicTrajectory::convertIntoGlobalTime(double t)
  {
    std::lock_guard<std::mutex> lock(parameters_mutex_);
    return t + parameters_.global_time_last_trajectory_generated;
  };

  void DynamicTrajectory::todoThreadLoop()
  {
    while (!stop_process_)
    {
      if (generate_new_traj_ && checkIfTrajectoryCanBeGenerated())
      {
        next_trajectory_waypoint_ = generateWaypointsForTheNextTrajectory();
        if (next_trajectory_waypoint_.size() == 0)
        // ||
        //   (next_trajectory_waypoint_.size() == 1 && !checkStitchTrajectory()))
        {
          DYNAMIC_LOG("No waypoints to generate a new trajectory");
          generate_new_traj_ = false;
          continue;
        }
        parameters_mutex_.lock();
        new_parameters_ = parameters_;
        new_parameters_.global_time_last_trajectory_generated =
            parameters_.last_global_time_evaluated;
        parameters_mutex_.unlock();

        DYNAMIC_LOG("generate new trajectory");
        if (checkStitchTrajectory())
        {
          DYNAMIC_LOG("stitching new trajectory");
          next_trajectory_waypoint_ = stitchActualTrajectoryWithNewWaypoints(
              parameters_.last_local_time_evaluated, next_trajectory_waypoint_);
        }
        else
        {
          DYNAMIC_LOG("Trajectory by scratch");
          appendDronePositionWaypoint(next_trajectory_waypoint_);
        }

        generateTrajectory(next_trajectory_waypoint_, parameters_.speed);
        generate_new_traj_ = false;
      }

      else if (waypoints_to_be_modified_.size() && !computing_new_trajectory_)
      {
        // DYNAMIC_LOG("Modifying waypoints");
        const std::lock_guard<std::mutex> lock(todo_mutex);
        for (auto &modification : waypoints_to_be_modified_)
        {
          bool modified = applyWaypointModification(modification.first, modification.second);
        }
        waypoints_to_be_modified_.clear();
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    return;
  }

  ThreadSafeTrajectory DynamicTrajectory::computeTrajectory(const DynamicWaypoint::Deque &waypoints,
                                                            const bool &lineal_optimization)
  {
    float max_speed = new_parameters_.speed;

    /* auto vertices = extractVerticesFromWaypoints(waypoints); */
    auto vertices = extractVerticesFromWaypoints(next_trajectory_waypoint_);
    if (vertices.size() < 2)
    {
      throw std::runtime_error("Not enough waypoints");
    }
    const int N = 10;
    std::shared_ptr<mav_trajectory_generation::Trajectory> trajectory =
        std::make_shared<mav_trajectory_generation::Trajectory>();
    auto segment_times =
        mav_trajectory_generation::estimateSegmentTimes(vertices, max_speed, this->a_max_);
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
      parameters.max_iterations = 2000; // 2000
      parameters.f_rel = 0.05;
      parameters.x_rel = 0.1;
      parameters.time_penalty = 1000; // 200.0 con 500 va bien
      parameters.initial_stepsize_rel = 0.1;
      // parameters.inequality_constraint_tolerance = 0.1;
      parameters.inequality_constraint_tolerance = 0.2;
      mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension_, parameters);
      opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
      opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                                        max_speed);
      opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION,
                                        a_max_);
      opt.optimize();
      opt.getTrajectory((mav_trajectory_generation::Trajectory *)trajectory.get());
    }
    mav_trajectory_generation::Segment::Vector segments;
    trajectory->getSegments(&segments);
    int index = 0;
    for (auto &waypoint : next_trajectory_waypoint_)
    {
      waypoint.setIndex(index);
      waypoint.setTime(getCummulativeTime(segments, waypoint.getIndex()) +
                       new_parameters_.global_time_last_trajectory_generated);
      index++;
    }
    return ThreadSafeTrajectory(std::move(trajectory));
  };

  void DynamicTrajectory::generateTrajectory(const DynamicWaypoint::DynamicWaypoint::Deque &waypoints,
                                             bool force)
  {
    computing_new_trajectory_ = true;
    std::lock_guard<std::mutex> lock(future_mutex_);
    future_traj_ =
        std::async(std::launch::async, &DynamicTrajectory::computeTrajectory, this, waypoints, false);
  };

  void DynamicTrajectory::swapTrajectory()
  {
    const std::lock_guard<std::mutex> lock(future_mutex_);
    traj_ = std::move(future_traj_.get());
    const std::lock_guard<std::mutex> lock3(parameters_mutex_);
    parameters_ = new_parameters_;
    trajectory_regenerated_ = true;
    DYNAMIC_LOG("Trajectory swapped");
  };

  void DynamicTrajectory::swapDynamicWaypoints()
  {
    const std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
    dynamic_waypoints_ = next_trajectory_waypoint_;
    DYNAMIC_LOG("Waypoints swapped");
  }

  Eigen::Vector3d DynamicTrajectory::evaluateModifiedTrajectory(const ThreadSafeTrajectory &traj,
                                                                double global_time, double local_time,
                                                                const int order)
  {
    Eigen::Vector3d refs;
    if (local_time < 0.0f)
    {
      local_time = 0.0f;
      DYNAMIC_LOG("[WARN] Time out of bounds");
    }
    if (local_time > traj.getMaxTime())
    {
      local_time = traj.getMaxTime();
      if (order > 0)
        return Eigen::Vector3d::Zero();
    }
    refs = traj.evaluate(local_time, order);
    dynamic_waypoints_mutex_.lock();
    for (auto &waypoint : dynamic_waypoints_)
    {
      if (waypoint.getName() == "")
      {
        continue;
      }
      refs += waypoint.trajectoryCompensation(global_time, order);
    }

    dynamic_waypoints_mutex_.unlock();
    return refs;
  }

  bool DynamicTrajectory::applyWaypointModification(const std::string &name,
                                                    const Eigen::Vector3d &position)
  {
    bool modified = false;
    std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
    for (auto &waypoint : dynamic_waypoints_)
    {
      if (waypoint.getName() == name && waypoint.getName() != "")
      {
        const std::lock_guard<std::mutex> lock3(parameters_mutex_);
        waypoint.setCurrentPosition(position, parameters_.last_global_time_evaluated);
        modified = true;
        trajectory_regenerated_ = true;
      }
    }
    return modified;
  };

  DynamicWaypoint::Deque DynamicTrajectory::stitchActualTrajectoryWithNewWaypoints(
      double last_t_evaluated, const DynamicWaypoint::Deque &waypoints)
  {
    int n_waypoints = N_WAYPOINTS_TO_APPEND + waypoints.size();
    DynamicWaypoint::Deque new_waypoints;
    // Append the previous points to generate a smooth stitching

    double local_eval_t = new_parameters_.last_local_time_evaluated;

    for (int i_waypoints_appended = 0; i_waypoints_appended < N_WAYPOINTS_TO_APPEND;
         i_waypoints_appended++)
    {
      dynamic_traj_generator::References references;
      mav_trajectory_generation::Vertex vertex(3);
      if (i_waypoints_appended == 0)
      {
        references = getReferences(traj_, convertIntoGlobalTime(local_eval_t), local_eval_t);
        for (int i = 0; i < 3; i++)
        {
          vertex.addConstraint(i, references[i]);
        }
        // std::cout << "waypoint position : " << references.position.transpose() << std::endl;
        // std::cout << "waypoint velocity : " << references.velocity.transpose() << std::endl;
        // std::cout << "waypoint acceleration : " << references.acceleration.transpose() << std::endl;
      }
      else
      {
        references = getReferences(traj_, convertIntoGlobalTime(local_eval_t), local_eval_t, true);
        vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                             references.position);
      }
      new_waypoints.emplace_back(vertex);
      // local_eval_t += (TIME_STITCHING_SECURITY_COEF / N_WAYPOINTS_TO_APPEND) *
      //                 computeSecurityTime(n_waypoints, new_parameters_.algorithm_time_constant);
      local_eval_t += 0.5f; // FIXME: hardcoded
    }
    // append the rest of the waypoints

    for (auto &waypoint : waypoints)
    {
      new_waypoints.emplace_back(waypoint);
    }
    /* std::this_thread::sleep_for(std::chrono::milliseconds(1000)); */
    return new_waypoints;
  };

  static void updateDynamicWaypointsPosition(DynamicWaypoint::Deque &dynamic_waypoints)
  {
    for (auto &waypoint : dynamic_waypoints)
    {
      waypoint.resetWaypointWithCurrentPosition();
    }
  }

  void resetWaypointThroughDeque(DynamicWaypoint::Deque &waypoints, const std::string &name,
                                 const Eigen::Vector3d &position)
  {
    if (name == "")
    {
      return;
    }
    for (auto &waypoint : waypoints)
    {
      if (waypoint.getName() == name)
      {
        DYNAMIC_LOG("reseting waypoint ");
        DYNAMIC_LOG(name);
        waypoint.resetWaypoint(position);
      }
    }
  };

  void DynamicTrajectory::filterPassedWaypoints(DynamicWaypoint::Deque &waypoints)
  {
    parameters_mutex_.lock();
    double last_t_eval = parameters_.last_global_time_evaluated;
    parameters_mutex_.unlock();
    for (auto it = waypoints.begin(); it != waypoints.end();)
    {
      if (it->getTime() < last_t_eval && it->getTime() != 0.0f)
      {
        DYNAMIC_LOG("Removing waypoint ");
        it = waypoints.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  DynamicWaypoint::Deque DynamicTrajectory::generateWaypointsForTheNextTrajectory()
  {
    DYNAMIC_LOG("Generating waypoints for the next trajectory");
    const std::lock_guard<std::mutex> lock(todo_mutex);
    DynamicWaypoint::Deque next_trajectory_waypoints;
    if (waypoints_to_be_set_.size() == 0)
    {
      /* if (waypoints_to_be_added_.size() == 0 && !checkTrajectoryModifiers()) {
       */
      /*   return next_trajectory_waypoints; */

      /* } */
      next_trajectory_waypoints = (dynamic_waypoints_);
      filterPassedWaypoints(next_trajectory_waypoints);
      updateDynamicWaypointsPosition(next_trajectory_waypoints);
    }
    else
    {
      for (auto &waypoint : waypoints_to_be_set_)
        next_trajectory_waypoints.emplace_back(waypoint);
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

  void DynamicTrajectory::appendDronePositionWaypoint(DynamicWaypoint::Deque &waypoints)
  {
    DynamicWaypoint waypoint;
    waypoint.resetWaypoint(getVehiclePosition());
    waypoints.emplace_front(waypoint);
  }

  /******************************************************************************/
  /****************************** CHECK FUNCTIONS *******************************/
  /******************************************************************************/

  bool DynamicTrajectory::checkStitchTrajectory()
  {
    if (traj_ == nullptr)
      return false;
    bool security_time = true;

    DYNAMIC_LOG("loading parameters");
    parameters_mutex_.lock();
    double last_global_t_eval = parameters_.last_global_time_evaluated;
    double previous_t_global = parameters_.global_time_last_trajectory_generated;
    parameters_mutex_.unlock();
    DYNAMIC_LOG("parameters loaded");

    if ((traj_.getMaxTime() + previous_t_global) - last_global_t_eval < 0.5f)
    {
      DYNAMIC_LOG("NOT STITCHING TRAJECTORY");
      security_time = false;
    }
    DYNAMIC_LOG("trajectory_time evaluated");
    return security_time;
  }

  bool DynamicTrajectory::checkTrajectoryGenerated()
  {
    while (traj_ == nullptr)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      const std::lock_guard<std::mutex> lock(future_mutex_);
      if (future_traj_.valid())
      {
        future_traj_.wait();
        break;
      }
    }
    future_mutex_.lock();
    bool future_traj_valid = future_traj_.valid();
    future_mutex_.unlock();
    if (computing_new_trajectory_ && future_traj_valid)
    {
      using namespace std::chrono_literals;
      future_mutex_.lock();
      bool traj_ready = future_traj_.wait_for(0ms) == std::future_status::ready;
      future_mutex_.unlock();
      if (traj_ready)
      {
        swapTrajectory();
        swapDynamicWaypoints();
        computing_new_trajectory_ = false;
        generate_new_traj_ = false;
      }
    }
    return true;
  }

  bool DynamicTrajectory::checkTrajectoryModifiers()
  {
    std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
    for (auto &waypoint : dynamic_waypoints_)
    {
      if (waypoint.hasModifiers())
        return true;
    }
    return false;
  };

  bool DynamicTrajectory::checkInSecurityZone()
  {
    if (traj_ == nullptr)
      return false;
    const std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
    parameters_mutex_.lock();
    double last_t_eval = parameters_.last_global_time_evaluated;
    parameters_mutex_.unlock();
    for (auto &waypoint : dynamic_waypoints_)
    {
      if (waypoint.getName() != "" && waypoint.getTime() > last_t_eval)
      {
        if (waypoint.getTime() - last_t_eval > SECURITY_TIME_BEFORE_WAYPOINT)
        {
          return false;
        }
        else
        {
          DYNAMIC_LOG("[DEBUG] in security zone");
          return true;
        }
      }
    }
    return false;
  };

  bool DynamicTrajectory::checkIfTrajectoryCanBeGenerated()
  {
    return !checkInSecurityZone() && !computing_new_trajectory_;
  }

} // namespace dynamic_traj_generator
