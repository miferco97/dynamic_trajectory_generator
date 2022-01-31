#ifndef __DYNAMIC_TRAJECTORY_V2_HPP__
#define __DYNAMIC_TRAJECTORY_V2_HPP__

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

  struct references
  {
    eigen::vector3d position;
    eigen::vector3d velocity;
    eigen::vector3d acceleration;

    eigen::vector3d &operator[](int index)
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
        throw std::runtime_error("invalid index");
      }
    }
  };

  class dynamictrajectory
  {
  private:
    dynamicwaypoint::vector waypoints_;
    dynamic_traj_generator::threadsafetrajectory traj_;
    std::future<threadsafetrajectory> future_traj_;

    std::atomic_bool generate_new_traj_ = false;
    std::atomic_bool in_security_zone_ = false;
    std::atomic_bool computing_new_trajectory_ = false;

    std::mutex future_mutex_;

    const int derivative_order_;

  public:
    dynamicwaypoint(int derivative_order = mav_trajectory_generation::derivative_order::acceleration)
    {
      derivative_order_ = derivative_order;
    }

    void setwaypoints(dynamicwaypoint::vector waypoints)
    {
      waypoints_ = waypoints;
    };

    bool modifywaypoint(const std::string &name, eigen::vector3d position)
    {
      bool modified = false;
      for (auto &waypoint : waypoints_)
      {
        if (waypoint.name == name)
        {
          waypoint.setactualposition(position);
          modified = true;
        }
      }
      return modified;
    }

    void appendwaypoint(dynamicwaypoint waypoint)
    {
      waypoints_.emplace_back(waypoint);
    }

    bool generatetrajectory(bool force = false, float max_speed)
    {
      std::lock_guard<std::mutex> lock(future_mutex_);
      future_traj_ = std::async(std::launch::async, &dynamictrajectory::__generatetrajectory, this, vertices, max_speed);
    }

    void dynamictrajectory::generatetrajectoryfromscratch(const mav_trajectory_generation::vertex::vector &vertices,
                                                          const float &max_speed)
    {
      std::lock_guard<std::mutex> lock(future_mutex_);
      future_traj_ = std::async(std::launch::async, &dynamictrajectory::__generatetrajectory, this, max_speed);
    };

  private:
    threadsafetrajectory computetrajectory(const mav_trajectory_generation::vertex::vector &vertices,
                                           const float &max_speed,
                                           const bool &lineal_optimization = false);

    // function to be run in a different thread
    threadsafetrajectory __generatetrajectory(const mav_trajectory_generation::vertex::vector &vertices, float max_speed)
    {
      while (in_security_zone_)
      {
        using std::chrono_literals;
        std::this_thread::sleep_for(10ms);
      }
      computing_new_trajectory_ = true;
      return computetrajectory(vertices, max_speed);
    }

    threadsafetrajectory computetrajectory(const mav_trajectory_generation::vertex::vector &vertices,
                                           const float &max_speed,
                                           const bool &lineal_optimization)
    {
      if (vertices.size() < 2)
      {
        throw std::runtime_error("not enough waypoints");
      }

      const int n = 10;
      std::shared_ptr<mav_trajectory_generation::trajectory> trajectory = std::make_shared<mav_trajectory_generation::trajectory>();
      auto segment_times = mav_trajectory_generation::estimatesegmenttimes(vertices, max_speed, this->a_max_);

      // optimizer
      if (lineal_optimization)
      {
        mav_trajectory_generation::polynomialoptimization<n> opt(dimension_);
        opt.setupfromvertices(vertices, segment_times, derivative_to_optimize_);
        opt.solvelinear();
        opt.gettrajectory((mav_trajectory_generation::trajectory *)trajectory.get());
      }
      else
      {
        mav_trajectory_generation::nonlinearoptimizationparameters parameters;
        parameters.max_iterations = 2000;
        parameters.f_rel = 0.05;
        parameters.x_rel = 0.1;
        parameters.time_penalty = 200.0;
        parameters.initial_stepsize_rel = 0.1;

        // parameters.inequality_constraint_tolerance = 0.1;
        mav_trajectory_generation::polynomialoptimizationnonlinear<n> opt(dimension_, parameters);
        opt.setupfromvertices(vertices, segment_times, derivative_to_optimize_);
        opt.addmaximummagnitudeconstraint(mav_trajectory_generation::derivative_order::velocity, max_speed);
        opt.addmaximummagnitudeconstraint(mav_trajectory_generation::derivative_order::acceleration, a_max_);
        opt.optimize();
        opt.gettrajectory((mav_trajectory_generation::trajectory *)trajectory.get());
      }

      trajectory->getsegments(&segments);
      for (auto &waypoint : temporal_dynamic_waypoints_)
      {
        dynamic_log(waypoint.getname());
        waypoint.settime(getcummulativetime(segments, waypoint.getindex()));
      }
      return threadsafetrajectory(trajectory);
    };
  };

} // namespace mav_planning

#endif // __DYNAMIC_TRAJECTORY_HPP__