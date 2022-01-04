#ifndef __DYNAMIC_TRAJECTORY_HPP__
#define __DYNAMIC_TRAJECTORY_HPP__

#include <chrono>
#include <exception>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include "mav_trajectory_generation/polynomial_optimization_nonlinear.h"
#include "mav_trajectory_generation/trajectory.h"

#include "matplotlibcpp.h"
#include "thread_safe_trajectory.hpp"
#include "utils/logging_utils.hpp"
#include "utils/traj_modifiers.hpp"

#define MAV_MAX_ACCEL (1 * 9.81f)

namespace dynamic_traj_generator
{

  struct References
  {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;

    Eigen::Vector3d &operator[](int index)
    {
      switch (index)
      {
      case 0:
        return position;
      case 1:
        return velocity;
      case 2:
        return acceleration;
      default:
        throw std::runtime_error("Invalid index");
      }
    }
  };

  class DynamicWaypoint
  {
    mav_trajectory_generation::Vertex vertex_;
    Eigen::Vector3d original_position_;
    Eigen::Vector3d actual_position_;
    double t_assigned_;
    std::string name_ = "";
    int index_ = -1;

    // GaussianModifier modifiers_[3];

    std::vector<std::array<GaussianModifier, 3>> modifiers_;

  public:
    DynamicWaypoint()
        : vertex_(mav_trajectory_generation::Vertex(3)){};

    DynamicWaypoint(const mav_trajectory_generation::Vertex &vertex, int index, const std::string &name = "")
        : DynamicWaypoint(vertex, name)
    {
      index_ = index;
    }

    DynamicWaypoint(const mav_trajectory_generation::Vertex &vertex, const std::string &name = "")
        : name_(name), vertex_(vertex)
    // : name_(name), vertex_(3)
    {
      Eigen::VectorXd pos;
      vertex_.getConstraint(0, &pos);
      original_position_[0] = pos[0];
      original_position_[1] = pos[1];
      original_position_[2] = pos[2];

      DYNAMIC_LOG(original_position_.transpose());
      // vertex_.addConstraint(mav_trajectory_generation::derivative_order::POSITION, original_position_);
      actual_position_ = original_position_;
    };

    // copy constructor
    DynamicWaypoint(const DynamicWaypoint &other)
        : vertex_(other.vertex_), original_position_(other.original_position_), actual_position_(other.actual_position_),
          t_assigned_(other.t_assigned_), name_(other.name_), index_(other.index_)
    {
      modifiers_ = other.modifiers_;
    }

    typedef std::vector<DynamicWaypoint> Vector;
    double last_sigma_ = SIGMA_FIXED;

    inline void setIndex(const int &index) { index_ = index; };
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

    inline void setName(const std::string &name) { name_ = name; };
    inline void displaceIndex(const int &displacement) { index_ += displacement; };

    void setActualPosition(Eigen::Vector3d position)
    {

      std::array<GaussianModifier, 3> modifiers;
      for (int i = 0; i < 3; i++)
      {
        modifiers[i].setDifference(position[i] - actual_position_[i]);
        modifiers[i].setModifierTime(t_assigned_);
        modifiers[i].setSigma(last_sigma_ * SIGMA_COEFFICIENT);
      }
      last_sigma_ *= SIGMA_COEFFICIENT;

      actual_position_ = position;
      modifiers_.emplace_back(modifiers);
      DYNAMIC_LOG(modifiers_.size());
    }
    Eigen::Vector3d trajectoryCompensation(double t, int derivative_order = 0)
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
    // get vertex
    inline mav_trajectory_generation::Vertex getVertex() const { return vertex_; };
    inline std::string getName() const { return name_; };
    inline int getIndex() const { return index_; };
    inline Eigen::Vector3d getOriginalPosition() const { return original_position_; };
    inline Eigen::Vector3d getActualPosition() const { return actual_position_; };
    inline double getTime() const { return t_assigned_; };
  };
  class DynamicTrajectory
  {

  private:
    // const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::JERK;
    const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::ACCELERATION;
    // const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::VELOCITY;

    ThreadSafeTrajectory traj_;
    std::future<ThreadSafeTrajectory> future_traj_;

    const int dimension_ = 3;
    std::atomic_bool from_scratch_ = true;
    const double a_max_ = MAV_MAX_ACCEL;

    double last_t_eval_ = 0.0f;
    double t_offset_ = 0.0f;

    std::mutex traj_mutex_;
    std::mutex future_mutex_;
    std::atomic_bool has_dynamic_waypoints_ = false;

    std::mutex dynamic_waypoints_mutex_;
    dynamic_traj_generator::DynamicWaypoint::Vector dynamic_waypoints_;
    dynamic_traj_generator::DynamicWaypoint::Vector temporal_dynamic_waypoints_;

    // std::atomic_bool has_traj_ = false;

  public:
    DynamicTrajectory(){};

    double getMaxTime();
    double getMinTime();

    inline mav_trajectory_generation::Vertex::Vector getWaypoints() { return traj_.getWaypoints(); }
    inline mav_trajectory_generation::Segment::Vector getSegments() { return traj_.getSegments(); }

    void generateTrajectory(const mav_trajectory_generation::Vertex::Vector &waypoints, const float &max_speed);
    void generateTrajectory(const dynamic_traj_generator::DynamicWaypoint::Vector &waypoints, const float &max_speed);

    void modifyWaypoint(const std::string &name, const Eigen::Vector3d &position)
    {
      std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
      for (auto &waypoint : dynamic_waypoints_)
      {
        if (waypoint.getName() == name)
        {
          DYNAMIC_LOG("Modifying waypoint");
          DYNAMIC_LOG(waypoint.getName());

          waypoint.setActualPosition(position);
          break;
        }
      }
    };

    bool obtainDynamicWaypoints(const std::string &waypoint_name, DynamicWaypoint &waypoint)
    {
      std::lock_guard<std::mutex> lock(dynamic_waypoints_mutex_);
      for (auto &waypoint_ : dynamic_waypoints_)
      {
        if (waypoint_.getName() == waypoint_name)
        {
          waypoint = waypoint_;
          return true;
        }
      }
      return false;
    };

    bool evaluateTrajectory(const float &t, dynamic_traj_generator::References &refs, bool only_positions = false);

  private:
    void selectProperTrajectoryGenerationMethod(const mav_trajectory_generation::Vertex::Vector &waypoints, const float &max_speed)
    {
      if (from_scratch_)
      {
        generateTrajectoryFromScratch(waypoints, max_speed);
      }
      else
      {
        // throw not implemented exception
        throw std::runtime_error("Not implemented");
      }
    };

    void generateTrajectoryFromScratch(const mav_trajectory_generation::Vertex::Vector &vertices,
                                       const float &max_speed);
    bool checkTrajectoryGenerated();
    ThreadSafeTrajectory computeTrajectory(const mav_trajectory_generation::Vertex::Vector &vertices,
                                           const float &max_speed,
                                           const bool &lineal_optimization = false);
    void swapTrajectory();
    bool getRefs(const ThreadSafeTrajectory &traj,
                 float t_eval, dynamic_traj_generator::References &refs,
                 const bool only_position = false);
  };

} // namespace mav_planning

#endif // __DYNAMIC_TRAJECTORY_HPP__