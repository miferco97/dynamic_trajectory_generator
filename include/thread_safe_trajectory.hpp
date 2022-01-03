#ifndef __THREAD_SAFE_TRAJECTORY_HPP__
#define __THREAD_SAFE_TRAJECTORY_HPP__

#include "mav_trajectory_generation/trajectory.h"
#include <chrono>
#include <memory>

class ThreadSafeTrajectory
{

  mutable std::mutex evaluate_mutex_;
  mutable std::mutex time_mutex_;
  std::shared_ptr<mav_trajectory_generation::Trajectory> trajectory_ptr_ = nullptr;

private:
  void reset(const std::shared_ptr<mav_trajectory_generation::Trajectory> &trajectory_)
  {
    std::lock_guard<std::mutex> lock(evaluate_mutex_);
    std::lock_guard<std::mutex> lock_time(time_mutex_);
    trajectory_ptr_ = std::move(trajectory_);
  }

public:
  ThreadSafeTrajectory(){};
  ThreadSafeTrajectory(const ThreadSafeTrajectory &other)
  {
    this->reset(other.trajectory_ptr_);
  };

  ThreadSafeTrajectory(ThreadSafeTrajectory &&other)
  {
    this->reset(other.trajectory_ptr_);
  };

  ThreadSafeTrajectory &operator=(const ThreadSafeTrajectory &other)
  {
    this->reset(other.trajectory_ptr_);
    return *this;
  };

  ThreadSafeTrajectory(std::shared_ptr<mav_trajectory_generation::Trajectory> trajectory_ptr)
  {
    this->reset(trajectory_ptr);
  };

  mav_trajectory_generation::Vertex::Vector getWaypoints() const
  {
    mav_trajectory_generation::Vertex::Vector waypoints;
    std::lock_guard<std::mutex> lock(evaluate_mutex_);
    trajectory_ptr_->getVertices(0, &waypoints);

    return waypoints;
  };
  mav_trajectory_generation::Segment::Vector getSegments() const
  {
    mav_trajectory_generation::Segment::Vector segments;
    std::lock_guard<std::mutex> lock(evaluate_mutex_);
    trajectory_ptr_->getSegments(&segments);
    return segments;
  };

  double getMaxTime() const
  {
    std::lock_guard<std::mutex> lock(time_mutex_);
    return trajectory_ptr_->getMaxTime();
  }
  double getMinTime() const
  {
    std::lock_guard<std::mutex> lock(time_mutex_);
    return trajectory_ptr_->getMinTime();
  }

  bool operator==(const ThreadSafeTrajectory &other) noexcept
  {
    return other.trajectory_ptr_ == trajectory_ptr_;
  }
  bool operator!=(const ThreadSafeTrajectory &other) noexcept
  {
    return other.trajectory_ptr_ != trajectory_ptr_;
  }
  bool operator==(const nullptr_t &other) noexcept
  {
    return trajectory_ptr_ == nullptr;
  }
  bool operator!=(const nullptr_t &other) noexcept
  {
    return trajectory_ptr_ != nullptr;
  }

  Eigen::VectorXd evaluate(double t, int derivative_order = 0) const
  {
    std::lock_guard<std::mutex> lock(evaluate_mutex_);
    return trajectory_ptr_->evaluate(t, derivative_order);
  }

private:
  void lock()
  {
    evaluate_mutex_.lock();
  }
  void unlock()
  {
    evaluate_mutex_.unlock();
  }
};

#endif //__DYNAMIC_TRAJECTORY_HPP__