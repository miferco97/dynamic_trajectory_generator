#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_publisher.hpp"
#include "as2_core/core_functions.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TrajectoryPublisher>(Trajectory_type::eth_spline_non_linear);
  node->preset_loop_frequency(100);
  as2::spinLoop(node);

  return 0;
}
