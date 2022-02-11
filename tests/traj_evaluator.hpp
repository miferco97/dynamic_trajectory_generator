#ifndef __TEST_TRAJ_EVALUATOR_HPP__
#define __TEST_TRAJ_EVALUATOR_HPP__

#include <chrono>
#include <iostream>
#include <map>
#include <random>
#include <thread>

#include "dynamic_trajectory.hpp"
#include "utils/logging_utils.hpp"
#include "utils/plotting_utils.hpp"

/* namespace plt = matplotlibcpp; */

#define MAX_POINT_MOVEMENT_DISTANCE 2.5
#define MAX_POINT_MOVEMENT_TIME 5

static Eigen::Vector3d poseRandomizer(const Eigen::Vector3d &v,
                                      double movement_distance = MAX_POINT_MOVEMENT_DISTANCE) {
  Eigen::Vector3d ret = v;
  for (int i = 0; i < 3; i++) {
    ret[i] += movement_distance * (1 - (std::rand() % 1000) / 500.0f);
  }
  return ret;
}

class DynamicWaypointModifier {
  std::string name_;
  dynamic_traj_generator::DynamicWaypoint waypoint_modified_;
  Eigen::Vector3d modified_position_;
  bool has_waypoint_ = false;

  public:
  DynamicWaypointModifier(const char *name) : name_(name){};

  DynamicWaypointModifier(const std::string &name) : name_(name){};
  std::string getName() const { return name_; }
  bool findWaypoint(const std::string &name, dynamic_traj_generator::DynamicWaypoint &waypoint,
                    dynamic_traj_generator::DynamicTrajectory &traj) {
    auto waypoints = traj.getDynamicWaypoints();
    for (auto &elem : waypoints) {
      if (name == elem.getName()) {
        waypoint = dynamic_traj_generator::DynamicWaypoint(elem);
        return true;
      }
    }
    return false;
  }
  void loadWaypointFromTraj(dynamic_traj_generator::DynamicTrajectory &traj) {
    has_waypoint_ = findWaypoint(name_, waypoint_modified_, traj);
    modified_position_ = waypoint_modified_.getCurrentPosition();
  };

  bool modifyWaypointInTraj(dynamic_traj_generator::DynamicTrajectory &traj, double t) {
    if (!has_waypoint_ || waypoint_modified_.getTime() < 0.1f) {
      loadWaypointFromTraj(traj);
    }

    if (has_waypoint_ && triggerModification(t)) {
      // std::cout << "Modifying waypoint " << name_ << " at time " << t << std::endl;
      updateModifiedPosition(t);
      traj.modifyWaypoint(name_, modified_position_);
      return true;
    }

    return false;
  };

  bool triggerModification(double t) {
    static double last_trigger_time = t;
    if (t < waypoint_modified_.getTime() - 0.1 &&
        t > waypoint_modified_.getTime() - MAX_POINT_MOVEMENT_TIME &&
        (t - last_trigger_time) > 1.0) {
      last_trigger_time = t;
      return true;
    }
    return false;
  };

  void updateModifiedPosition(double t) {
    static double multiplier_reduced = 1.0;
    // modified_position_ = poseRandomizer(modified_position_, MAX_POINT_MOVEMENT_DISTANCE *
    // multiplier_reduced); multiplier_reduced *= 0.75j
    /* modified_position_ = modified_position_ + Eigen::Vector3d(0.2, 0.2, 0.2); */
    modified_position_ = modified_position_ + Eigen::Vector3d(1.0, 1.0, 1.0);
  };
};

#define STEP_SIZE 10ms
class TrajEvaluator {
  private:
  std::vector<DynamicWaypointModifier> dynamic_waypoint_modifiers_;

  public:
  void addWaypointModifiers(const DynamicWaypointModifier &elem) {
    dynamic_waypoint_modifiers_.emplace_back(elem);
  };

  void runEvaluation(dynamic_traj_generator::DynamicTrajectory &traj, double end_time = -1.0f) {
    TrajectoryPlotter figure;
    figure.plotTraj(traj);
    if (end_time < 0.0f) {
      end_time = traj.getMaxTime();
      DYNAMIC_LOG(end_time);
    }

    dynamic_traj_generator::References refs;
    auto start = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed;
    double t = 0.0f;
    bool change_traj = false;
    do {
      /* auto end = std::chrono::high_resolution_clock::now(); */
      /* using namespace std::chrono_literals; */
      /* std::this_thread::sleep_for(STEP_SIZE); */
      /* std::chrono::duration<double, std::milli> elapsed = end - start; */
      /* t = elapsed.count() / 1000.0f; */
      using namespace std::chrono_literals;
      std::this_thread::sleep_for(STEP_SIZE);
      t += 0.01;

      traj.evaluateTrajectory(t, refs, false);

      for (auto &elem : dynamic_waypoint_modifiers_) {
        static int counter = 0;
        if (counter > 100) {
          continue;
        }
        if (elem.modifyWaypointInTraj(traj, t)) counter++;
      }
      if (traj.getWasTrajectoryRegenerated()) {
        /* DYNAMIC_LOG("Trajectory was regenerated"); */
        figure.plotTraj(traj);
      }

      figure.setUAVposition(refs, t);
    } while (t < end_time);
  };
};

#endif  // __TEST_TRAJ_EVALUATOR_HPP__
