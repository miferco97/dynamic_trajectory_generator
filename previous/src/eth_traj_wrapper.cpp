#include "eth_traj_wrapper.hpp"

ETHSplineGenerator::ETHSplineGenerator(TrajGeneratorOptimizator type, as2::Node *node_ptr)
    : type_(type), node_ptr_(node_ptr)
{

  /* * Constraints:
   *  Acc       = [g_acc_min*9.8, g_acc_max*9.8] (m/s/s)
   *  Vel       = [0 ,  Vel_max] (m/s)
   *  Omega_pr  = [0 , Omega_pr_max] (rad/s)
   *  Omega_yaw = [0 , Omega_yaw_max] (rad/s)
   *  acc_yaw   = [0 , acc_yaw_max] (rad/s/s)
   * */

  constraints_.g_acc_min = 0.25f;
  constraints_.g_acc_max = 9.8f;
  // constraints_.g_acc_max     = 2.0f;
  // constraints_.vel_max       = 5.0f;
  constraints_.vel_max = 10.0f;
  constraints_.omega_pr_max = M_PI / 2.0f;
  constraints_.omega_yaw_max = M_PI / 2.0f;
  constraints_.acc_yaw_max = M_PI;
  begin_time_ = rclcpp::Clock().now();

  sub_pose_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
      node_ptr_->generate_global_name("self_localization/odom"), 1,
      std::bind(&ETHSplineGenerator::odomCallback, this, std::placeholders::_1));
};

ETHSplineGenerator::~ETHSplineGenerator()
{
  gen_traj_thread_.join();
};

void ETHSplineGenerator::logTrajectoryWaypoints(const std::vector<mav_trajectory_generation::Vertex> &vertices)
{
  Eigen::VectorXd position;
  uint8_t index = 0;
  RCLCPP_INFO(node_ptr_->get_logger(), "LOGGING TRAJECTORY GENERATED with %d waypoints", vertices.size());

  for (auto &vertex : vertices)
  {
    vertex.getConstraint(mav_trajectory_generation::derivative_order::POSITION, &position);
    Eigen::VectorXd velocity;
    vertex.getConstraint(mav_trajectory_generation::derivative_order::VELOCITY, &velocity);

    // RCLCPP_INFO(node_ptr_->get_logger(),"position size %d",position.size());
    RCLCPP_INFO(node_ptr_->get_logger(), "Waypoint[%d] = [%6.4f, %6.4f, %6.4f ]", index, position[0], position[1], position[2]);
    if (velocity.size() == 3)
    {
      RCLCPP_INFO(node_ptr_->get_logger(), "Velocity[%d] = [%6.4f, %6.4f, %6.4f ]", index, velocity[0], velocity[1], velocity[2]);
    }
    else
    {
      RCLCPP_INFO(node_ptr_->get_logger(), "Velocity[%d] is not constrained", index);
    }
    index++;
  }
}

void ETHSplineGenerator::genTraj(const std::vector<std::vector<float>> &waypoints, float speed, const std::vector<float> &actual_speed_acc)
{
  // Waypoints[i][j] => i: (x,y,z,yaw) j:(waypoint_j)

  trajectory_swapped_ = false;
  bool trajectory_from_scratch = true;
  uint8_t n_points_added = 1;
  rclcpp::Time t_i = rclcpp::Clock().now();
  std::vector<int> vertices_to_remove;
  std::vector<mav_trajectory_generation::Vertex> vertices;

  int n_points = waypoints[0].size(); // number of waypoints we check it on x axis

  // Check if you must generate trajectory from a previous one or build one from scratch
  // if ((atom_end_time_ - atom_last_t_evaluated_) < (n_points_added)*(2*atom_average_trajectory_generation_elapsed_time_) || atom_end_time_ == 0.0f)

  // FIXME: WARN: this is a hack to avoid the problem of the first trajectory generation (the additional points are not added to the spline)

  if (atom_end_time_ == 0.0f || atom_last_t_evaluated_ >= atom_end_time_)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Generating new trajectory from scratch");
    // built it from scratch

    vertices = std::vector<mav_trajectory_generation::Vertex>(n_points, dimension_);
    for (int i = 0; i < vertices.size(); i++)
    {
      vertices[i].addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                                Eigen::Vector3d(waypoints[0][i],
                                                waypoints[1][i],
                                                waypoints[2][i]));
    }
    vertices[0].addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0, 0, 0));
    vertices[0].addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(0, 0, 0));
  }

  /**********************************************************************/
  /************************** CUARENTENA ********************************/
  /**********************************************************************/

  else
  {
    vertices = std::vector<mav_trajectory_generation::Vertex>(n_points, dimension_);
    trajectory_mutex_.lock();
    auto refs = last_sended_refs_;
    trajectory_mutex_.unlock();

    vertices[0].addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(refs[0][0],
                                                                                                     refs[1][0],
                                                                                                     refs[2][0]));
    vertices[0].addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(refs[0][1],
                                                                                                     refs[1][1],
                                                                                                     refs[2][1]));
    vertices[0].addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(refs[0][2],
                                                                                                         refs[1][2],
                                                                                                         refs[2][2]));

    for (int i = 1; i < vertices.size(); i++)
    {
      vertices[i].addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                                Eigen::Vector3d(waypoints[0][i],
                                                waypoints[1][i],
                                                waypoints[2][i]));
    }
    vertices[0].addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0, 0, 0));
    vertices[0].addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(0, 0, 0));

    // begin from previous trajectory references for the first 3 points
    RCLCPP_INFO(node_ptr_->get_logger(), "Generating new trajectory from previous trajectory");
    trajectory_from_scratch = false;
  }
  /*

  static std::array<std::array<float, 3>, 4> refs;

  n_points = n_points + n_points_added - 1; // -1 for the previous point that is not added
  vertices = std::vector<mav_trajectory_generation::Vertex>(n_points, dimension_);

  uint16_t final_n_points = 0;
  uint16_t index_waypoint = 1;

  float t_evaluate;
  Eigen::Vector3d last_position;
  Eigen::Vector3d actual_position;
  Eigen::Vector3d first_position;

  // at the beggining we consider that the first waypoint is included in previous trajectory
  bool waypoints_included_in_previous_traj = true;

  for (uint8_t i = 0; i < vertices.size(); i++)
  {
    // Add waypoints according with previous trajectory
    if (i >= 0 && i < n_points_added)
    {
      t_evaluate = atom_last_t_evaluated_ + (2.0f * i) * atom_average_trajectory_generation_elapsed_time_;
      RCLCPP_INFO(node_ptr_->get_logger(), "last t_evaluated = %6.4f with final time = %6.4f", t_evaluate, (float)atom_end_time_);
      evaluateTrajectory(t_evaluate, refs);
      actual_position = Eigen::Vector3d(refs[0][0], refs[1][0], refs[2][0]);

      // // check if added waypoints have the same position
      // if (i != 0 && (last_position - actual_position).norm() < 0.01)
      // {
      //   RCLCPP_WARN(node_ptr_->get_logger(), "Waypoint[%d]  wont be added removed because it is too close to previous waypoint", i);
      //   // vertices_to_remove.emplace_back(i);
      // }
      // else
      // {
      if (i == 0)
      {
        trajectory_mutex_.lock();
        refs = last_sended_refs_;
        trajectory_mutex_.unlock();


        vertices[final_n_points].addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(refs[0][0],
                                                                                                                      refs[1][0],
                                                                                                                      refs[2][0]));
        vertices[final_n_points].addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(refs[0][1],
                                                                                                                      refs[1][1],
                                                                                                                      refs[2][1]));
        vertices[final_n_points].addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(refs[0][2],
                                                                                                                          refs[1][2],
                                                                                                                          refs[2][2]));
        first_position = actual_position;
        final_n_points++;
      }
      // }
      last_position = actual_position;
    }
    else
    {
      // simplify the trajectory by removing the points that are too close to the previous one
      auto &zero = first_position; // TODO: rename
      auto &hero = last_position;
      auto waypoint_position = Eigen::Vector3d(waypoints[0][index_waypoint],
                                               waypoints[1][index_waypoint],
                                               waypoints[2][index_waypoint]) -
                               zero;

      auto from_cero_to_hero = hero - zero; // TODO: change name

      if (waypoints_included_in_previous_traj && (from_cero_to_hero.norm()*2.0f > waypoint_position.norm()))
        {

          RCLCPP_WARN(node_ptr_->get_logger(), "Waypoint[%d]  wont be added removed because it is too close to previous waypoint", i);
        }
      else
      {
        waypoints_included_in_previous_traj = false;
        vertices[final_n_points].addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                                               Eigen::Vector3d(waypoints[0][index_waypoint],
                                                               waypoints[1][index_waypoint],
                                                               waypoints[2][index_waypoint]));
        final_n_points++;
      }
      index_waypoint++;
    }
  }
  // Remove vertices that are not added
  if (final_n_points > n_points)
  {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Final_n_points = %d is higher than n_points = %d", final_n_points, n_points);
  }
  vertices.resize(final_n_points, dimension_);
  n_points = final_n_points;

  vertices[vertices.size() - 1].addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(0, 0, 0));
  vertices[vertices.size() - 1].addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d(0, 0, 0));
}
*/

  /**********************************************************************/
  /************************** FIN DE CUARENTENA ********************************/
  /**********************************************************************/

  // last point has velocity and acceleration zero

  logTrajectoryWaypoints(vertices);

  // -------------- Calculate time slots -------------
  // RCLCPP_INFO(node_ptr_->get_logger(), "Simulating delay");
  // std::this_thread::sleep_for(5s);

  double v_max = speed;
  auto segment_times = mav_trajectory_generation::estimateSegmentTimes(vertices, v_max, a_max_);
  const int N = 10;
  mav_trajectory_generation::Segment::Vector segments;
  std::unique_ptr<mav_trajectory_generation::Trajectory> trajectory = std::make_unique<mav_trajectory_generation::Trajectory>();

  // Optimizer
  if (type_ == TrajGeneratorOptimizator::LINEAR)
  {
    mav_trajectory_generation::PolynomialOptimization<N> opt(dimension_);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
    opt.solveLinear();
    opt.getSegments(&segments);
    opt.getTrajectory((mav_trajectory_generation::Trajectory *)trajectory.get());
  }

  else if (type_ == TrajGeneratorOptimizator::NONLINEAR)
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
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max_);
    opt.optimize();
    opt.getPolynomialOptimizationRef().getSegments(&segments);
    opt.getTrajectory((mav_trajectory_generation::Trajectory *)trajectory.get());
  }

  // const float time_alpha = 0.2;
  // atom_average_trajectory_generation_elapsed_time_ =  atom_average_trajectory_generation_elapsed_time_*(1-time_alpha)+ time_alpha * (rclcpp::Clock().now()-t_i).seconds();
  atom_average_trajectory_generation_elapsed_time_ = (rclcpp::Clock().now() - t_i).seconds();
  RCLCPP_INFO(node_ptr_->get_logger(), "Trajectory generation time: %f", (float)atom_average_trajectory_generation_elapsed_time_);

  if (!trajectory_from_scratch)
  {
    atom_max_evaluation_time_ = atom_average_trajectory_generation_elapsed_time_ * n_points_added * 1.5;
  }
  else
  {
    atom_max_evaluation_time_ = -1.0f;
  }

  next_trajectory_mutex_.lock();
  next_traj_ptr_ = std::move(trajectory);
  next_trajectory_mutex_.unlock();

  swapOldTrajectoryWithNewTrajectory();

  // return checkTrajectoryFeasibility();
}
void ETHSplineGenerator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr _msg)
{
  auto &msg = _msg->pose;

  pose_mutex_.lock();
  actual_pos_[0] = msg.pose.position.x;
  actual_pos_[1] = msg.pose.position.y;
  actual_pos_[2] = msg.pose.position.z;
  pose_mutex_.unlock();

  // TODO: IMPROVE THIS

  /* static std::thread t;
  if (atom_new_traj_generated_){
      if (t.joinable()){
          t.join();
      }
      t = std::thread(&ETHSplineGenerator::swapOldTrajectoryWithNewTrajectory,this);
  } */
}

float locateDroneInTraj(const Eigen::Vector3d &actual_pos,
                        const mav_trajectory_generation::Trajectory *trajectory,
                        float max_evaluation_time,
                        float offset = 0.0f)
{
  if (max_evaluation_time < 0.0f)
  {
    // RCLCPP_WARN(rclcpp::get_logger("locateDroneInTraj"), "max_evaluation_time = %f is negative", max_evaluation_time);
    return 0.0f;
  }

  const float step = 0.1f;
  float dist = 0.0f;
  float min_dist = (actual_pos - trajectory->evaluate(0.5f)).norm();
  float delay_time = 0.50f;

  // float evaluation_time = (trajectory->getMaxTime() < max_evaluation_time) ? trajectory->getMaxTime() : max_evaluation_time;
  float evaluation_time = trajectory->getMaxTime();
  for (float t = step; t < evaluation_time; t += step)
  {
    dist = (actual_pos - trajectory->evaluate(t)).norm();
    if (dist <= min_dist)
    {
      min_dist = dist;
      delay_time = t;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("test"), "Delay time: %.3f, max_evaluation_time = %.3f", delay_time, max_evaluation_time);
  return delay_time + step;
}

bool ETHSplineGenerator::generateTrajectory(const std::vector<std::vector<float>> &waypoints, float speed, const std::vector<float> &actual_speed_acc)
{
  trajectory_generated_ = false;
  while (!trajectory_swapped_)
  {
    RCLCPP_WARN(rclcpp::get_logger("generateTrajectory"), "Trajectory not swapped");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (gen_traj_thread_.joinable())
  {
    gen_traj_thread_.join();
  }
  gen_traj_thread_ = std::thread(&ETHSplineGenerator::genTraj, this, waypoints, speed, actual_speed_acc);
  // gen_traj_thread_.detach();
  return true;
}

void ETHSplineGenerator::swapOldTrajectoryWithNewTrajectory()
{

  // indent to ease lock understanding
  next_trajectory_mutex_.lock();

  pose_mutex_.lock();
  auto actual_pos = actual_pos_;
  pose_mutex_.unlock();

  float delay = locateDroneInTraj(actual_pos, next_traj_ptr_.get(), atom_max_evaluation_time_);

  next_trajectory_mutex_.unlock();
  trajectory_mutex_.lock();
  next_trajectory_mutex_.lock();
  traj_ptr_ = std::move(next_traj_ptr_);
  atom_delay_t_ = delay;
  next_traj_ptr_ = nullptr;
  next_trajectory_mutex_.unlock();

  atom_end_time_ = traj_ptr_->getMaxTime();
  RCLCPP_INFO(node_ptr_->get_logger(), "Trajectory swaped");
  time_mutex_.lock();
  begin_time_ = rclcpp::Clock().now();
  time_mutex_.unlock();

  trajectory_generated_ = true;
  trajectory_mutex_.unlock();

  trajectory_swapped_ = true;
}

bool ETHSplineGenerator::evaluateTrajectory(float t, std::array<std::array<float, 3>, 4> &refs, bool for_plot)
{
  trajectory_mutex_.lock();

  if (traj_ptr_ == nullptr)
  {
    // std::cout << "no traj jet" << std::endl;
    trajectory_mutex_.unlock();
    return false;
  }
  // static auto last_refs = refs;
  t += atom_delay_t_;

  if (for_plot)
  {

    if (t < atom_end_time_)
    {
      auto derivative_order = mav_trajectory_generation::derivative_order::POSITION;
      Eigen::VectorXd sample = traj_ptr_->evaluate(t, derivative_order);
      for (int i = 0; i < sample.size(); i++)
        refs[i][0] = sample[i];
    }
    trajectory_mutex_.unlock();
    return true;
  }

  atom_last_t_evaluated_ = t;

  if (t > atom_end_time_)
  {
    if (t < atom_end_time_ + 0.5)
    {

      // set velocities and accels to 0.0f
      for (int i = 0; i < refs.size(); i++)
      {
        refs[i][1] = 0.0f;
        refs[i][2] = 0.0f;
      }
    }
  }
  else if (t < 0)
  {
    t = 0.0f;
    std::cerr << "trajectory evaluated in t<0!!" << std::endl;
  }
  else
  {
    const float alpha = 1;
    int derivative_order;
    derivative_order = mav_trajectory_generation::derivative_order::POSITION;
    Eigen::VectorXd sample = traj_ptr_->evaluate(t, derivative_order);
    if (sample.size() > 4)
      throw std::out_of_range("sample size is higher than 4\n");

    for (int i = 0; i < sample.size(); i++)
      refs[i][0] = sample[i];

    derivative_order = mav_trajectory_generation::derivative_order::VELOCITY;
    sample = traj_ptr_->evaluate(t, derivative_order);
    for (int i = 0; i < sample.size(); i++)
      refs[i][1] = sample[i];

    derivative_order = mav_trajectory_generation::derivative_order::ACCELERATION;
    sample = traj_ptr_->evaluate(t, derivative_order);
    for (int i = 0; i < sample.size(); i++)
      refs[i][2] = sample[i];

#ifdef AUTOYAW
    // refs[3][0] = -atan2f((double)refs[0][1],(double)refs[1][1])+M_PI/2.0f;
    refs[3][0] = 0;
#endif
  }

  last_sended_refs_ = refs;
  trajectory_mutex_.unlock();
  return true;
}

bool ETHSplineGenerator::checkTrajectoryFeasibility()
{

  /*
   * Constraints:
   *  Acc       = [Acc_min, Acc_max]
   *  Vel       = [0 ,  Vel_max]
   *  Omega_pr  = [0 , Omega_pr_max]
   *  Omega_yaw = [0 , Omega_yaw_max]
   *  acc_yaw   = [0 , acc_yaw_max]
   * */

  // Create input constraints.

  /*
  typedef mav_trajectory_generation::InputConstraintType ICT;
  mav_trajectory_generation::InputConstraints input_constraints;

  input_constraints.addConstraint(ICT::kFMin,         constraints_.g_acc_min * 9.81);     // minimum acceleration in [m/s/s].
  input_constraints.addConstraint(ICT::kFMax,         constraints_.g_acc_max  * 9.81);    // maximum acceleration in [m/s/s].
  input_constraints.addConstraint(ICT::kVMax,         constraints_.vel_max );             // maximum velocity in [m/s].
  input_constraints.addConstraint(ICT::kOmegaXYMax,   constraints_.omega_pr_max );        // maximum roll/pitch rates in [rad/s].
  input_constraints.addConstraint(ICT::kOmegaZMax,    constraints_.omega_yaw_max);        // maximum yaw rates in [rad/s].
  input_constraints.addConstraint(ICT::kOmegaZDotMax, constraints_.acc_yaw_max);          // maximum yaw acceleration in [rad/s/s].

  // Create feasibility object of choice (FeasibilityAnalytic,FeasibilitySampling, FeasibilityRecursive).
  int feasibility_value = 0;

  // Check dynamic feasibility
  mav_trajectory_generation::FeasibilityAnalytic feasibility_check(input_constraints);
  feasibility_check.settings_.setMinSectionTimeS(0.01);
  int i = 0;

  // Check feasibility of each segment.
  for (auto segment:segments){
      mav_trajectory_generation::InputFeasibilityResult result = feasibility_check.checkInputFeasibility(segment);
      if ((int)result != 0){
          std::cout << "The segment input(" << i << ") is " << getInputFeasibilityResultName(result) << "." << std::endl;
      }
      feasibility_value += (int) result;
      i++;
  }

  // Check ground plane feasibility
  // mav_trajectory_generation::FeasibilityBase feasibility_check;

  // Create ground plane.
  Eigen::Vector3d point(0.0, 0.0, 0.5);
  Eigen::Vector3d normal(0.0, 0.0, 1.0);
  feasibility_check.half_plane_constraints_.emplace_back(point, normal);

  // Check feasibility.
  for (auto segment:segments){
      if(!feasibility_check.checkHalfPlaneFeasibility(segment)){
          std::cout << "The segment is in collision with the ground plane." << std::endl;
      feasibility_value++;
      }
  }

  if (feasibility_value == 0) return true;
  else{
      std::cout << "Trajectory generated is not feasible\n";
      return false;
  }
  */
  return true;
}
