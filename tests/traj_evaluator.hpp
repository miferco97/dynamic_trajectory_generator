#ifndef __TEST_TRAJ_EVALUATOR_HPP__
#define __TEST_TRAJ_EVALUATOR_HPP__

#include "dynamic_trajectory.hpp"

#include <chrono>
#include <iostream>
#include <map>
#include <thread>

namespace plt = matplotlibcpp;

#define STEP_SIZE 10ms

class Figure3D
{
  // auto traj_ptr = getTrajectoryPtr();
private:
  long number_;

public:
  Figure3D(long number = 1)
      : number_(number)
  {
    uav_pose_x_ = std::vector<double>(1);
    uav_pose_y_ = std::vector<double>(1);
    uav_pose_z_ = std::vector<double>(1);
    static_plots_3d_.clear();
    plot_thread_ = std::thread(&Figure3D::plot, this);
    ended_ = false;
  }
  ~Figure3D()
  {
    ended_ = true;
    plot_thread_.join();
  }

  std::vector<double> uav_pose_x_;
  std::vector<double> uav_pose_y_;
  std::vector<double> uav_pose_z_;

  std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::map<std::string, std::string>, long>> static_plots_3d_;

  std::atomic_bool ended_;
  std::atomic_bool update_plot_;

  std::thread plot_thread_;

  std::mutex mutex_;

  void plotWholeTraj(dynamic_traj_generator::DynamicTrajectory &traj)
  {
    double t_start = traj.getMinTime();
    double t_end = traj.getMaxTime();
    double dt = 0.1;
    int n_samples = (t_end - t_start) / dt;
    mutex_.lock();
    auto plot_x = std::vector<double>(n_samples);
    auto ploy_y = std::vector<double>(n_samples);
    auto plot_z = std::vector<double>(n_samples);

    dynamic_traj_generator::References refs;
    for (int i = 0; i < n_samples; i++)
    {
      double t_eval = t_start + i * dt;
      traj.evaluateTrajectory(t_eval, refs, true);
      plot_x[i] = refs.position(0);
      ploy_y[i] = refs.position(1);
      plot_z[i] = refs.position(2);
    }

    static_plots_3d_.emplace_back(plot_x, ploy_y, plot_z, std::map<std::string, std::string>(), number_);

    auto waypoints = traj.getWaypoints();

    auto waypoints_x = std::vector<double>(waypoints.size());
    auto waypoints_y = std::vector<double>(waypoints.size());
    auto waypoints_z = std::vector<double>(waypoints.size());

    for (int i = 0; i < waypoints.size(); i++)
    {
      Eigen::VectorXd pos;
      waypoints[i].getConstraint(0, &pos);
      waypoints_x[i] = pos(0);
      waypoints_y[i] = pos(1);
      waypoints_z[i] = pos(2);
    }
    static std::map<std::string, std::string> style;
    if (style.empty())
    {
      style["color"] = "b";
      style["marker"] = "x";
      style["markersize"] = "7";
      style["linestyle"] = "none";
    }
    static_plots_3d_.emplace_back(waypoints_x, waypoints_y, waypoints_z, style, number_);

    mutex_.unlock();
    update_plot_ = true;
  };

  void setUAVposition(const dynamic_traj_generator::References &refs)
  {
    mutex_.lock();
    uav_pose_x_[0] = refs.position(0);
    uav_pose_y_[0] = refs.position(1);
    uav_pose_z_[0] = refs.position(2);
    mutex_.unlock();
    update_plot_ = true;
  };

  void plot()
  {
    plt::figure(number_);
    // plt::axis("equal");
    plt::grid(true);
    // plt::ion();

    static std::map<std::string, std::string> style;
    if (style.empty())
    {
      style["color"] = "r";
      style["marker"] = "o";
      style["markersize"] = "5";
      style["linestyle"] = "none";
    }

    while (!ended_)
    {
      if (!update_plot_)
      {
        plt::pause(0.01);
        continue;
      }

      plt::cla();
      mutex_.lock();
      for (auto &kwargs : static_plots_3d_)
      {
        plt::plot3(std::get<0>(kwargs), std::get<1>(kwargs), std::get<2>(kwargs), std::get<3>(kwargs), std::get<4>(kwargs));
      }
      plt::plot3(uav_pose_x_, uav_pose_y_, uav_pose_z_, style, number_);

      plt::show(false);
      update_plot_ = false;
      mutex_.unlock();
    }
    DYNAMIC_LOG("Close figure to continue");
    plt::show(true);
    plt::close();
  };
};

class Figure2D
{
};

class TrajEvaluator
{
private:
public:
  void runEvaluation(dynamic_traj_generator::DynamicTrajectory &traj, double end_time = -1.0f)
  {
    Figure3D figure;
    figure.plotWholeTraj(traj);
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
      figure.setUAVposition(refs);
    } while (t < end_time);
  };
};

#endif // __TEST_TRAJ_EVALUATOR_HPP__
