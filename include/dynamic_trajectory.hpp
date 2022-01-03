#ifndef __DYNAMIC_TRAJECTORY_HPP__
#define __DYNAMIC_TRAJECTORY_HPP__

#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include "mav_trajectory_generation/trajectory.h"

#include "logging_utils.hpp"
#include "matplotlibcpp.h"
#include "thread_safe_trajectory.hpp"

#define MAV_MAX_ACCEL (1 * 9.81f)

namespace dynamic_traj_generator
{

  struct References
  {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
  };

  class DynamicTrajectory
  {
  public:
    DynamicTrajectory(){};

  private:
    // const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::JERK;
    const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::ACCELERATION;
    // const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::VELOCITY;

    ThreadSafeTrajectory traj_;
    std::future<ThreadSafeTrajectory> future_traj_;

    const int dimension_ = 3;
    bool from_scratch_ = true;
    const double a_max_ = MAV_MAX_ACCEL;

    double last_t_eval_ = 0.0f;
    double t_offset_ = 0.0f;

    std::mutex traj_mutex_;
    std::mutex future_mutex_;

  private:
    bool checkTrajectoryGenerated()
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

  private:
    ThreadSafeTrajectory computeTrajectory(const mav_trajectory_generation::Vertex::Vector &vertices,
                                           const float &max_speed,
                                           const bool &lineal_optimization = false)
    {
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
      return ThreadSafeTrajectory(trajectory);
    };

  public:
    void generateTrajectoryFromScratch(const mav_trajectory_generation::Vertex::Vector &vertices,
                                       const float &max_speed)
    {
      std::lock_guard<std::mutex> lock(future_mutex_);
      future_traj_ = std::async(std::launch::async, &DynamicTrajectory::computeTrajectory, this, vertices, max_speed, false);
    };

  public:
    bool evaluateTrajectory(const float &t, dynamic_traj_generator::References &refs, bool only_positions = false)
    {
      const std::lock_guard<std::mutex> lock(traj_mutex_);
      float t_eval = t + t_offset_;
      last_t_eval_ = t_eval;
      return getRefs(traj_, t_eval, refs, only_positions);
    }

  private:
    void swapTrajectory()
    {
      const std::lock_guard<std::mutex> lock(future_mutex_);
      traj_ = std::move(future_traj_.get());
      last_t_eval_ = 0.0f;
      // waiting_for_traj_ = false;
      DYNAMIC_LOG("Trajectory swapped");
    };

    bool getRefs(const ThreadSafeTrajectory &traj,
                 float t_eval, dynamic_traj_generator::References &refs,
                 const bool only_position = false)
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

      refs.position = traj.evaluate(t_eval, mav_trajectory_generation::derivative_order::POSITION);
      if (only_position)
        return true;
      refs.velocity = traj.evaluate(t_eval, mav_trajectory_generation::derivative_order::VELOCITY);
      refs.acceleration = traj.evaluate(t_eval, mav_trajectory_generation::derivative_order::ACCELERATION);

      return true;
    }

  public:
    void printTrajectory()
    {
      if (!checkTrajectoryGenerated())
        return;
      double t_start = traj_.getMinTime();
      double t_end = traj_.getMaxTime();
      double dt = 0.1;
      for (double t_eval = t_start; t_eval < t_end; t_eval += dt)
      {
        Eigen::VectorXd x_eval;
        x_eval = traj_.evaluate(t_eval, mav_trajectory_generation::derivative_order::POSITION);
        std::cout << "t: " << t_eval << " x: " << x_eval.transpose() << std::endl;
      }
    }
    double getMaxTime()
    {
      if (!checkTrajectoryGenerated())
        return 0.0;
      return traj_.getMaxTime();
    }
    double getMinTime()
    {
      if (!checkTrajectoryGenerated())
        return 0.0;
      return traj_.getMinTime();
    }

    void generate2Dplot();
    void generate3DPlot();
    void showPlots();

    mav_trajectory_generation::Vertex::Vector getWaypoints()
    {
      return traj_.getWaypoints();
    }
  };

} // namespace mav_planning

#endif // __DYNAMIC_TRAJECTORY_HPP__