#include "dynamic_trajectory.hpp"

#include <chrono>
#include <future>
#include <memory>
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

  void DynamicTrajectory::modifyWaypoint(
    const std::string & name, const Eigen::Vector3d & position,
    bool generate_new_traj)
  {
    const std::lock_guard<std::mutex> lock(todo_mutex);
    DYNAMIC_LOG("Modifying waypoint: ");
    // check if the waypoint is already in the waypoints to be modified list
    auto iter = std::find_if(
      waypoints_to_be_modified_.begin(), waypoints_to_be_modified_.end(),
      [&name](const std::pair<std::string, Eigen::Vector3d> & waypoint)
      {
        return waypoint.first == name;
      });
    if (iter == waypoints_to_be_modified_.end()) {
      waypoints_to_be_modified_.emplace_back(name, position);
    } else {
      iter->first = name;
      iter->second = position;
    }
    generate_new_traj_ = generate_new_traj;
  }
  
  void DynamicTrajectory::modifyWaypoints(
    const std::vector<std::pair<std::string,
    Eigen::Vector3d>> & waypoints_to_modified)
  {
    // check if the waypoint is already in the waypoints to be modified list
    for (const auto & waypoint : waypoints_to_modified) {
      modifyWaypoint(waypoint.first, waypoint.second, false);
    }
    generate_new_traj_ = true;
  }

  bool DynamicTrajectory::evaluateTrajectory(const float &t, dynamic_traj_generator::References &refs,
                                             bool only_positions, bool for_plotting)
  {
    // if (!checkIfTrajectoryIsAlreadyGenerated()) return false;
    // DYNAMIC_LOG("Evaluating trajectory");
    checkTrajectoryGenerated();

    double global_time = t;
    // double global_time = t - time_constant_;
    double local_time = convertFromGlobalTime(t);
    if (!for_plotting)
    {
      parameters_mutex_.lock();
      parameters_.last_global_time_evaluated = global_time;
      parameters_.last_local_time_evaluated = local_time;
      parameters_mutex_.unlock();
    }

    global_time += parameters_.t_offset;
    local_time += parameters_.t_offset;

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
    new_parameters_.speed = speed;
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
  bool DynamicTrajectory::getGenerateNewTraj() const
  {
    return generate_new_traj_;
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
        time_measure_mutex_.lock();
        inital_time_traj_generation_ = std::chrono::steady_clock::now();
        time_measure_mutex_.unlock();

        if (checkStitchTrajectory())
        {
          from_scratch_ = false;
          DYNAMIC_LOG("stitching new trajectory");
          next_trajectory_waypoint_ = stitchActualTrajectoryWithNewWaypoints(
              parameters_.last_local_time_evaluated, next_trajectory_waypoint_);
        }
        else
        {
          DYNAMIC_LOG("Trajectory by scratch");
          from_scratch_ = true;
          appendDronePositionWaypoint(next_trajectory_waypoint_);
        }
        generateTrajectory(next_trajectory_waypoint_, false);
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
    parameters_mutex_.lock();
    float max_speed = parameters_.speed;
    parameters_mutex_.unlock();

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

    // std::this_thread::sleep_for(std::chrono::milliseconds(501));

    time_measure_mutex_.lock();

    time_constant_ = std::chrono::duration_cast<std::chrono::microseconds>(
                         std::chrono::steady_clock::now() - inital_time_traj_generation_)
                         .count() /
                     1.0e6;

    DYNAMIC_LOG(time_constant_);
    time_measure_mutex_.unlock();

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

    future_mutex_.lock();
    auto new_trajectory = future_traj_.get();
    future_mutex_.unlock();

    if (from_scratch_)
    {
      vehicle_position_mutex_.lock();
      auto arg1 = vehicle_position_;
      vehicle_position_mutex_.unlock();
      timeFittingWithVehiclePosition(new_trajectory,arg1);
      from_scratch_ = false;
    }
    else
    {
      parameters_mutex_.lock();
      auto arg1 = parameters_.last_global_time_evaluated + parameters_.t_offset;
      auto arg2 = parameters_.last_local_time_evaluated + parameters_.t_offset;
      parameters_mutex_.unlock();
      timeFittingWithVehiclePosition(new_trajectory,evaluateModifiedTrajectory(
          traj_, arg1, arg2));
    }

    parameters_mutex_.lock();
    parameters_ = new_parameters_;
    parameters_mutex_.unlock();

    traj_ = std::move(new_trajectory);

    trajectory_regenerated_ = true;
    DYNAMIC_LOG("Trajectory swapped");
  };

  void DynamicTrajectory::swapDynamicWaypoints()
  {
    dynamic_waypoints_mutex_.lock();
    dynamic_waypoints_ = next_trajectory_waypoint_;
    dynamic_waypoints_mutex_.unlock();
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
      // if (order > 0) return Eigen::Vector3d::Zero();
    }

    refs = traj.evaluate(local_time, order);

    if (global_time > 0)
    {
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
    }
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

    double local_eval_t = new_parameters_.last_local_time_evaluated + parameters_.t_offset;

    for (int i_waypoints_appended = 0; i_waypoints_appended < N_WAYPOINTS_TO_APPEND;
         i_waypoints_appended++)
    {
      dynamic_traj_generator::References references;
      mav_trajectory_generation::Vertex vertex(3);
      if (i_waypoints_appended == 0)
      {
        references = getReferences(traj_, convertIntoGlobalTime(local_eval_t), local_eval_t);
        // for (int i = 0; i < 3; i++)
        for (int i = 0; i < 1; i++)
        {
          vertex.addConstraint(i, references[i]);
        }
        // std::cout << "waypoint position : " << references.position.transpose() << std::endl;
        // std::cout << "waypoint velocity : " << references.velocity.transpose() << std::endl;
        // std::cout << "waypoint acceleration : " << references.acceleration.transpose() <<
        // std::endl;
      }
      else
      {
        // references = getReferences(traj_, convertIntoGlobalTime(local_eval_t), local_eval_t, true);
        references = getReferences(traj_, convertIntoGlobalTime(local_eval_t), local_eval_t);
        for (int i = 0; i < 1; i++)
        {
          vertex.addConstraint(i, references[i]);
        }
        // vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
        //                      references.position);
      }
      new_waypoints.emplace_back(vertex);
      // local_eval_t += (TIME_STITCHING_SECURITY_COEF / N_WAYPOINTS_TO_APPEND) *
      //                 computeSecurityTime(n_waypoints, new_parameters_.algorithm_time_constant);
      //
      // local_eval_t += 0.5f;  // FIXME: hardcoded
      local_eval_t += 5*TRAJECTORY_COMPUTATION_TIME;
    }
    // append the rest of the waypoints

    double global_eval_t = convertIntoGlobalTime(local_eval_t);
    for (auto &waypoint : waypoints)
    {
      if (waypoint.getTime() < global_eval_t && waypoint.getTime() > 0.0f)
      {
        continue;
      }
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
      if (it->getTime() < last_t_eval) // && it->getTime() != 0.0f)
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
    waypoint.setConstraint(1, Eigen::Vector3d::Zero());
    waypoint.setConstraint(2, Eigen::Vector3d::Zero());

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

    if ((traj_.getMaxTime() + previous_t_global) - last_global_t_eval <
        2 * TRAJECTORY_COMPUTATION_TIME)
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

    // DYNAMIC_LOG("Checking trajectory generated");
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
          // DYNAMIC_LOG("[DEBUG] in security zone");
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

  void DynamicTrajectory::timeFittingWithVehiclePosition(ThreadSafeTrajectory &traj, const Eigen::Vector3d vehicle_position)
  {
    if (traj == nullptr)
      return;

    const double step = 0.1f;
    const double max_eval_time = 1.5 * TRAJECTORY_COMPUTATION_TIME;

    double min_time = 0.0f;
    double global_time = min_time + new_parameters_.global_time_last_trajectory_generated;
    auto pos = evaluateModifiedTrajectory(traj, global_time, min_time, 0);

    double min_distance = (pos - vehicle_position).norm();

    for (double t = step; t < max_eval_time; t += step)
    {
    double global_t = t + new_parameters_.global_time_last_trajectory_generated;
    auto pos = evaluateModifiedTrajectory(traj, global_t, t, 0);
      // pos = evaluateModifiedTrajectory(traj, global_t, t, 0);
      double distance = (pos - vehicle_position).norm();
      if (distance <= min_distance)
      {
        min_distance = distance;
        min_time = t;
      }
    }

    parameters_mutex_.lock();
    double mid_time = parameters_.last_global_time_evaluated -
                      new_parameters_.global_time_last_trajectory_generated;

    DYNAMIC_LOG("NEW_TIME_FITTING");
    DYNAMIC_LOG(parameters_.last_global_time_evaluated);
    DYNAMIC_LOG(new_parameters_.global_time_last_trajectory_generated);
    DYNAMIC_LOG(min_time);
    // parameters_.t_offset = min_time - mid_time + step;
    new_parameters_.t_offset = min_time - mid_time + step;
    parameters_mutex_.unlock();
    // new_parameters_.t_offset = min_time - mid_time + step;

    // new_parameters_.t_offset = std::abs(min_time - mid_time) + step;
    // new_parameters_.t_offset = (min_time - mid_time);
    DYNAMIC_LOG(new_parameters_.t_offset);
  };

} // namespace dynamic_traj_generator
