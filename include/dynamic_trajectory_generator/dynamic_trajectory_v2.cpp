#ifndef __DYNAMIC_TRAJECTORY_V2_HPP__
#define __DYNAMIC_TRAJECTORY_V2_HPP__
// this is only a template for structuring the new class
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
  dynamicwaypoint(int derivative_order = mav_trajectory_generation::derivative_order::acceleration);
  void setwaypoints(dynamicwaypoint::vector waypoints);
  bool modifywaypoint(const std::string &name, eigen::vector3d position);
  void appendwaypoint(dynamicwaypoint waypoint);

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
};

#endif // __DYNAMIC_TRAJECTORY_HPP__