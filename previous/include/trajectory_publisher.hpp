#ifndef __TRAJECTORY_PUBLISHER_H__
#define __TRAJECTORY_PUBLISHER_H__

// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "as2_core/node.hpp"
#include "as2_msgs/msg/trajectory_waypoints.hpp"

#include <iostream>
#include <math.h>
#include <string>
#include <vector>

#include "eth_traj_wrapper.hpp"
#include "trajectory_generator.hpp"

#define LOG_(x) std::cout << x << std::endl

enum Trajectory_type
{
    spline_basic,
    circle,
    lemniscate,
    eth_spline_linear,
    eth_spline_non_linear
};

class TrajectoryPublisher : public as2::Node
{
private:
    // Ros stuff
    std::string n_space_;
    std::string self_localization_pose_topic_;
    std::string self_localization_speed_topic_;
    std::string motion_reference_traj_topic_;
    std::string motion_reference_waypoints_path_topic_;
    std::string debug_traj_generated_topic_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<as2_msgs::msg::TrajectoryWaypoints>::SharedPtr waypoints_sub_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    TrajectoryGenerator *traj_gen_;

    bool is_trajectory_generated_ = false;
    rclcpp::Time begin_time_;
    std::string frame_id_ = "odom";
    Trajectory_type type_;
    float actual_pose_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    std::vector<float> actual_vel_acc_;
    rclcpp::Time last_time_;

    int yaw_mode_ = 0;
    float begin_traj_yaw_ = 0.0f;

public:
    TrajectoryPublisher(Trajectory_type type);
    ~TrajectoryPublisher();

    void setup();
    void run();

private:
    void plotTrajectory(float period);
    void plotTrajectory__(float period);
    std::thread plot_thread_;

    void publishTrajectory();

    void CallbackWaypointsTopic(const as2_msgs::msg::TrajectoryWaypoints::SharedPtr msg);
    void CallbackOdomTopic(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif