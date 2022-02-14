#ifndef __DYNAMIC_WAYPOINT_HPP__
#define __DYNAMIC_WAYPOINT_HPP__

#include "mav_trajectory_generation/trajectory.h"
#include <deque>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <string>
#include <vector>

#include "utils/traj_modifiers.hpp"

namespace dynamic_traj_generator
{
  class DynamicWaypoint
  {
    mav_trajectory_generation::Vertex vertex_;
    Eigen::Vector3d original_position_;
    Eigen::Vector3d current_position_;
    double t_assigned_;
    std::string name_ = "";
    int index_ = -1;

    std::vector<std::array<GaussianModifier, 3>> modifiers_;

  public:
    DynamicWaypoint();
    DynamicWaypoint(const mav_trajectory_generation::Vertex &vertex, int index, const std::string &name = "");
    DynamicWaypoint(const mav_trajectory_generation::Vertex &vertex, const std::string &name = "");
    DynamicWaypoint(const DynamicWaypoint &other); // copy constructor

    typedef std::vector<DynamicWaypoint> Vector;
    typedef std::deque<DynamicWaypoint> Deque;

    double last_sigma_ = SIGMA_FIXED;

    inline void setIndex(int index) { index_ = index; };
    inline void setTime(const double &t)
    {
      for (auto &modifier : modifiers_)
      {
        for (int i = 0; i < 3; i++)
        {
          modifier[i].setModifierTime(t);
        }
      }
      t_assigned_ = t;
    };

    inline bool hasModifiers() const { return !modifiers_.empty(); };
    inline void setName(const std::string &name) { name_ = name; };
    inline void displaceIndex(int displacement) { index_ += displacement; };
    void resetWaypointWithCurrentPosition()
    {
      original_position_ = current_position_;
      vertex_.addConstraint(0, current_position_);
      modifiers_.clear();
    };
    void resetWaypoint(const Eigen::Vector3d &position)
    {
      original_position_ = position;
      current_position_ = position;
      vertex_.addConstraint(0, current_position_);

      modifiers_.clear();
    };

    void setCurrentPosition(Eigen::Vector3d position, double actual_time = 0.0f);
    Eigen::Vector3d trajectoryCompensation(double t, int derivative_order = 0);

    inline mav_trajectory_generation::Vertex getVertex() const
    {
      return vertex_;
    };

    inline std::string getName() const { return name_; };
    inline int getIndex() const { return index_; };
    inline Eigen::Vector3d getOriginalPosition() const { return original_position_; };
    inline Eigen::Vector3d getCurrentPosition() const { return current_position_; };
    inline double getTime() const { return t_assigned_; };
  };

} // namespace dynamic_traj_generator
#endif
