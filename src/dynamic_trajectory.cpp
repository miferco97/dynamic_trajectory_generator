#include "dynamic_trajectory.hpp"

namespace dynamic_traj_generator
{

  void DynamicTrajectory::generate2Dplot()
  {
    // auto traj_ptr = getTrajectoryPtr();
    if (!checkTrajectoryGenerated())
      return;

    namespace plt = matplotlibcpp;
    double t_start = traj_.getMinTime();
    double t_end = traj_.getMaxTime();
    double dt = 0.1;
    int n_samples = (t_end - t_start) / dt;

    plt::figure();
    std::vector<double> x_vec(n_samples), y_vec(n_samples), z_vec(n_samples);
    std::vector<double> time(n_samples);

    References refs;
    for (int i = 0; i < n_samples; i++)
    {
      double t_eval = t_start + i * dt;
      getRefs(traj_, t_eval, refs, true);
      x_vec[i] = refs.position(0);
      y_vec[i] = refs.position(1);
      z_vec[i] = refs.position(2);
      time[i] = t_eval;
    }
    plt::plot(time, x_vec, "r-", time, y_vec, "g-", time, z_vec, "b-");
    plt::show(true);
  }

  void DynamicTrajectory::generate3DPlot()
  {
    if (!checkTrajectoryGenerated())
      return;

    // auto traj_ptr = getTrajectoryPtr();

    namespace plt = matplotlibcpp;
    double t_start = traj_.getMinTime();
    double t_end = traj_.getMaxTime();
    double dt = 0.1;
    int n_samples = (t_end - t_start) / dt;

    plt::figure();
    std::vector<double> x_vec(n_samples), y_vec(n_samples), z_vec(n_samples);
    std::vector<double> time(n_samples);
    References refs;
    for (int i = 0; i < n_samples; i++)
    {
      double t_eval = t_start + i * dt;
      getRefs(traj_, t_eval, refs, true);
      x_vec[i] = refs.position(0);
      y_vec[i] = refs.position(1);
      z_vec[i] = refs.position(2);
      time[i] = t_eval;
    }
    plt::plot3(x_vec, y_vec, z_vec);
    plt::show(true);
  }

  void DynamicTrajectory::showPlots()
  {
    namespace plt = matplotlibcpp;
    // plt::show();
  }

} // namespace dynamic_traj_generator