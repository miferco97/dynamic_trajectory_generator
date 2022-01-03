#ifndef __PLOTTING_UTILS_HPP__
#define __PLOTTING_UTILS_HPP__

#include <chrono>
#include <iostream>
#include <map>
#include <random>
#include <thread>

#include "dynamic_trajectory.hpp"

namespace plt = matplotlibcpp;

#define STEP_SIZE 10ms

static char dist[] = {'r', 'g', 'b', 'c', 'm', 'y', 'k'};

class TrajectoryPlotter
{
  // auto traj_ptr = getTrajectoryPtr();
private:
  long number_;
  long number2d_;

public:
  TrajectoryPlotter(long number = 1)
      : number_(number * 2), number2d_(number * 2 + 1)
  {
    uav_pose_x_ = std::vector<double>(1);
    uav_pose_y_ = std::vector<double>(1);
    uav_pose_z_ = std::vector<double>(1);
    uav_time_ = std::vector<double>(1);
    static_plots_3d_.clear();
    plot_thread_ = std::thread(&TrajectoryPlotter::plot, this);
    ended_ = false;
  }
  ~TrajectoryPlotter()
  {
    ended_ = true;
    plot_thread_.join();
  }

  std::vector<double> uav_pose_x_;
  std::vector<double> uav_pose_y_;
  std::vector<double> uav_pose_z_;
  std::vector<double> uav_time_;

  std::vector<std::tuple<std::vector<double>,                // x
                         std::vector<double>,                // y
                         std::vector<double>,                // z
                         std::map<std::string, std::string>, // options
                         long>                               // number
              >
      static_plots_3d_;

  std::vector<std::tuple<std::vector<double>, // time
                         std::vector<double>, // x
                         std::vector<double>, // y
                         std::vector<double>, // z
                         bool,                // only_waypoints
                         long>                // number
              >
      static_plots_2d_;

  std::atomic_bool ended_;
  std::atomic_bool update_plot_;

  std::thread plot_thread_;

  std::mutex mutex_;

  void plotTraj(dynamic_traj_generator::DynamicTrajectory &traj)
  {
    double t_start = traj.getMinTime();
    double t_end = traj.getMaxTime();
    double dt = 0.1;
    int n_samples = (t_end - t_start) / dt;
    mutex_.lock();
    auto plot_x = std::vector<double>(n_samples);
    auto ploy_y = std::vector<double>(n_samples);
    auto plot_z = std::vector<double>(n_samples);
    auto plot_time = std::vector<double>(n_samples);

    srand(time(NULL));
    std::string color(1, dist[rand() % 7]);

    dynamic_traj_generator::References refs;
    for (int i = 0; i < n_samples; i++)
    {
      double t_eval = t_start + i * dt;
      traj.evaluateTrajectory(t_eval, refs, true);
      plot_x[i] = refs.position(0);
      ploy_y[i] = refs.position(1);
      plot_z[i] = refs.position(2);
      plot_time[i] = t_eval;
    }

    std::map<std::string, std::string> style;
    style["color"] = color;

    static_plots_3d_.emplace_back(plot_x, ploy_y, plot_z, style, number_);
    static_plots_2d_.emplace_back(plot_time, plot_x, ploy_y, plot_z, false, number2d_);

    auto waypoints = traj.getWaypoints();
    auto segments = traj.getSegments();

    auto waypoints_x = std::vector<double>(waypoints.size());
    auto waypoints_y = std::vector<double>(waypoints.size());
    auto waypoints_z = std::vector<double>(waypoints.size());
    auto segments_time = std::vector<double>(waypoints.size(), 0);

    DYNAMIC_LOG(segments.size());
    DYNAMIC_LOG(waypoints.size());

    for (int i = 0; i < waypoints.size(); i++)
    {
      Eigen::VectorXd pos;
      waypoints[i].getConstraint(0, &pos);
      waypoints_x[i] = pos(0);
      waypoints_y[i] = pos(1);
      waypoints_z[i] = pos(2);
      if (i < segments.size())
      {
        segments_time[i + 1] = segments_time[i] + segments[i].getTime();
      }
    }
    std::map<std::string, std::string> style2;
    style2["color"] = color;
    style2["marker"] = "x";
    style2["markersize"] = "7";
    style2["linestyle"] = "none";

    static_plots_3d_.emplace_back(waypoints_x, waypoints_y, waypoints_z, style2, number_);
    static_plots_2d_.emplace_back(segments_time, waypoints_x, waypoints_y, waypoints_z, true, number2d_);

    mutex_.unlock();
    update_plot_ = true;
  };

  void setUAVposition(const dynamic_traj_generator::References &refs, const double &time)
  {
    mutex_.lock();
    uav_pose_x_[0] = refs.position(0);
    uav_pose_y_[0] = refs.position(1);
    uav_pose_z_[0] = refs.position(2);
    uav_time_[0] = time;
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
      for (auto &kwargs : static_plots_2d_)
      {
        if (std::get<4>(kwargs))
        {
          plt::figure(std::get<5>(kwargs));
          plt::subplot(3, 1, 1);
          plt::plot(std::get<0>(kwargs), std::get<1>(kwargs), "rx");
          plt::subplot(3, 1, 2);
          plt::plot(std::get<0>(kwargs), std::get<2>(kwargs), "gx");
          plt::subplot(3, 1, 3);
          plt::plot(std::get<0>(kwargs), std::get<3>(kwargs), "bx");
        }
        else
        {
          plt::figure(std::get<5>(kwargs));
          plt::subplot(3, 1, 1);
          plt::cla();
          plt::plot(std::get<0>(kwargs), std::get<1>(kwargs), "r-");
          plt::subplot(3, 1, 2);
          plt::cla();
          plt::plot(std::get<0>(kwargs), std::get<2>(kwargs), "g-");
          plt::subplot(3, 1, 3);
          plt::cla();
          plt::plot(std::get<0>(kwargs), std::get<3>(kwargs), "b-");
        }
      }

      plt::plot3(uav_pose_x_, uav_pose_y_, uav_pose_z_, style, number_);
      plt::figure(number2d_);
      plt::subplot(3, 1, 1);
      plt::plot(uav_time_, uav_pose_x_, "ro");
      plt::subplot(3, 1, 2);
      plt::plot(uav_time_, uav_pose_y_, "go");
      plt::subplot(3, 1, 3);
      plt::plot(uav_time_, uav_pose_z_, "bo");

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

#endif // DYNAMIC_TRAJECTORY_PLOTTER_HPP
