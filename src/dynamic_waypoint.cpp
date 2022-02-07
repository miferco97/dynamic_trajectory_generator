#include "dynamic_waypoint.hpp"

namespace dynamic_traj_generator
{
  DynamicWaypoint::DynamicWaypoint()
      : vertex_(mav_trajectory_generation::Vertex(3)){};

  DynamicWaypoint::DynamicWaypoint(const mav_trajectory_generation::Vertex &vertex, int index, const std::string &name)
      : DynamicWaypoint(vertex, name)
  {
    index_ = index;
  }

  DynamicWaypoint::DynamicWaypoint(const mav_trajectory_generation::Vertex &vertex, const std::string &name)
      : name_(name), vertex_(vertex)
  {
    Eigen::VectorXd pos;
    vertex_.getConstraint(0, &pos);
    original_position_[0] = pos[0];
    original_position_[1] = pos[1];
    original_position_[2] = pos[2];
    current_position_ = original_position_;
  };

  // copy constructor
  DynamicWaypoint::DynamicWaypoint(const DynamicWaypoint &other)
      : vertex_(other.vertex_), original_position_(other.original_position_), current_position_(other.current_position_),
        t_assigned_(other.t_assigned_), name_(other.name_), index_(other.index_)
  {
    modifiers_ = other.modifiers_;
  }

  void DynamicWaypoint::setCurrentPosition(Eigen::Vector3d position, double actual_time)
  {

    std::array<GaussianModifier, 3> modifiers;
    for (int i = 0; i < 3; i++)
    {
      modifiers[i].setDifference(position[i] - current_position_[i]);
      modifiers[i].setModifierTime(t_assigned_);

      double sigma;

      if (actual_time > 0.0f)
      {
        // 99.7 % of data is between +- 3*sigma
        sigma = (t_assigned_ - actual_time) / 3.0f;
      }
      else
      {
        sigma = last_sigma_ * SIGMA_COEFFICIENT;
        last_sigma_ = sigma;
      }
      modifiers[i].setSigma(sigma);
    }

    current_position_ = position;
    modifiers_.emplace_back(modifiers);
  }

  Eigen::Vector3d DynamicWaypoint::trajectoryCompensation(double t, int derivative_order)
  {
    Eigen::Vector3d compensation = Eigen::Vector3d::Zero();
    for (auto &modifier : modifiers_)
    {
      for (int i = 0; i < 3; i++)
      {
        compensation[i] += modifier[i](t, derivative_order);
      }
    }
    return compensation;
  };
} // namespace dynamic_traj_generator
