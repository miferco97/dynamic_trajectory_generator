#ifndef __TEST_TRAJ_EVALUATOR_HPP__
#define __TEST_TRAJ_EVALUATOR_HPP__

#include "dynamic_trajectory.hpp"
#include "utils/plotting_utils.hpp"

#include <chrono>
#include <iostream>
#include <map>
#include <random>
#include <thread>

namespace plt = matplotlibcpp;

#define STEP_SIZE 10ms

class TrajEvaluator
{
private:
public:
  void runEvaluation(dynamic_traj_generator::DynamicTrajectory &traj, double end_time = -1.0f)
  {
    TrajectoryPlotter figure;
    figure.plotTraj(traj);
    if (end_time < 0.0f)
    {
      end_time = traj.getMaxTime();
      DYNAMIC_LOG(end_time);
    }

    dynamic_traj_generator::References refs;
    auto start = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed;
    double t = 0.0f;

    do
    {
      auto end = std::chrono::high_resolution_clock::now();
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(STEP_SIZE);
      std::chrono::duration<double, std::milli> elapsed = end - start;
      t = elapsed.count() / 1000.0f;
      traj.evaluateTrajectory(t, refs, true);
      std::cout << "time [" << t << "]:" << refs.position.transpose() << std::endl;
      figure.setUAVposition(refs, t);
    } while (t < end_time);
  };
};

#endif // __TEST_TRAJ_EVALUATOR_HPP__
