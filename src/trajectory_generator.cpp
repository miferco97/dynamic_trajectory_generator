#include "trajectory_generator.hpp"

/****************** * CIRCLE GENERATOR ******************/

bool CircleGenerator::generateTrajectory(const vector<vector<float>> & waypoints, float speed)
{
  if (waypoints[0].size() != 2) {
    std::cout << "waypoints size = " << waypoints.size() << "!!!" << std::endl;
    return false;
  }

  for (auto & j : waypoints[0]) std::cout << j << "  ";

  center_.emplace_back(waypoints[0][0]);
  center_.emplace_back(waypoints[1][0]);
  center_.emplace_back(waypoints[2][0]);
  center_.emplace_back(waypoints[3][0]);

  for (auto & x : center_) std::cout << x << "  ";
  std::cout << std::endl;

  std::array<float, 2> point = {waypoints[1][0], waypoints[1][1]};

  height_ = waypoints[2][1];
  std::cout << "height_ " << height_ << std::endl;
  yaw_ = waypoints[3][1];

  float expr = 0.0;

  for (int i = 0; i < 2; i++) {
    expr += pow(center_[i] - point[i], 2);
  }
  radius_ = sqrt(expr) / 2.0f;
  std::cout << "radius " << radius_ << std::endl;
  omega_ = speed / radius_;
  end_time_ = 100000;
  beginTime_ = rclcpp::Clock().now();
  trajectory_generated_ = true;
  return true;
}

bool CircleGenerator::evaluateTrajectory(
  float t, std::array<std::array<float, 3>, 4> & refs, bool for_plot)
{
  refs[0][0] = radius_ * sin(omega_ * t);
  refs[1][0] = radius_ * cos(omega_ * t);
  refs[2][0] = height_;
  // refs[3][0] = 0.0f;

  refs[0][1] = omega_ * radius_ * cos(omega_ * t);
  refs[1][1] = -omega_ * radius_ * sin(omega_ * t);
  refs[2][1] = 0.0f;
  refs[3][1] = 0.0f;

  refs[0][2] = -radius_ * omega_ * omega_ * sin(omega_ * t);
  refs[1][2] = -radius_ * omega_ * omega_ * cos(omega_ * t);
  refs[2][2] = 0.0f;
  refs[3][2] = 0.0f;

#ifdef AUTOYAW
  refs[3][0] = -atan2f((double)refs[0][1], (double)refs[1][1]) + M_PI / 2.0f;
  // refs[3][0] = 0;
#endif
  return true;
}

/******************* LEMNSICATE GENERATOR ********************************/

bool LemniscateGenerator::generateTrajectory(const vector<vector<float>> & waypoints, float speed)
{
  if (waypoints[0].size() != 2) {
    std::cout << "waypoints size = " << waypoints.size() << "!!!" << std::endl;
    return false;
  }

  center_.emplace_back(waypoints[0][0]);
  center_.emplace_back(waypoints[1][0]);
  center_.emplace_back(waypoints[2][0]);
  center_.emplace_back(waypoints[3][0]);

  for (auto x : center_) std::cout << x << "  ";
  std::cout << std::endl;

  std::array<float, 2> point = {waypoints[1][0], waypoints[1][1]};

  height_ = waypoints[2][1];
  std::cout << "height_ " << height_ << std::endl;
  yaw_ = waypoints[3][1];

  float expr = 0.0;

  for (int i = 0; i < 2; i++) {
    expr += pow(center_[i] - point[i], 2);
  }
  radius_ = sqrt(expr) / 2.0f;
  std::cout << "radius " << radius_ << std::endl;
  omega_ = speed / radius_;
  end_time_ = 10000;
  trajectory_generated_ = true;

  return true;
};

bool LemniscateGenerator::evaluateTrajectory(
  float time, std::array<std::array<float, 3>, 4> & refs, bool for_plot)
{
  auto t = omega_ * time;
  auto a = radius_ * sqrt(2);

  refs[0][0] = a * cos(t) / (pow(sin(t), 2) + 1);
  refs[1][0] = a * cos(t) * sin(t) / (pow(sin(t), 2) + 1);
  refs[2][0] = height_;
  refs[3][0] = yaw_;

  refs[0][1] =
    -a * sin(t) * ((pow(sin(t), 2) + 2 * (pow(cos(t), 2) + 1)) / pow((pow(sin(t), 2) + 1), 2));
  refs[1][1] = -a * (pow(sin(t), 4) * +pow(sin(t), 2) + (pow(sin(t), 2) - 1) * pow(cos(t), 2)) /
               pow(pow(sin(t), 2) + 1, 2);
  refs[2][1] = 0.0f;
  refs[3][1] = 0.0f;

  refs[0][2] =
    a * cos(t) *
    (5 * pow(sin(t), 4) + 7 * pow(sin(t), 2) + (6 * pow(sin(t), 2) - 2) * pow(cos(t), 2) - 2) /
    pow(pow(sin(t), 2) + 1, 3);
  refs[1][2] =
    (4 * a * sin(t) * cos(t) * (pow(sin(t), 6) + (pow(sin(t), 2) - 1) * pow(cos(t), 2))) /
      pow((pow(sin(t), 2) + 1), 3) -
    (a * (2 * sin(t) * pow(cos(t), 3) + 6 * pow(sin(t), 5) * cos(t) -
          2 * (pow(sin(t), 2) - 1) * sin(t) * cos(t))) /
      pow(pow(sin(t), 2) + 1, 2);
  refs[2][2] = 0.0f;
  refs[3][2] = 0.0f;

  return true;
};