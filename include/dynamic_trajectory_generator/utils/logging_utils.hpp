#ifndef __LOGGING_UTILS_HPP__
#define __LOGGING_UTILS_HPP__

#include <chrono>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>

#define COUNT_TIME(_function_)                                                                                                                                     \
  {                                                                                                                                                                \
    auto start = std::chrono::steady_clock::now();                                                                                                                 \
    _function_;                                                                                                                                                    \
    auto end = std::chrono::steady_clock::now();                                                                                                                   \
    std::cout << "[" << #_function_ << " ] takes :  " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0f << "s" << std::endl; \
  }

namespace dynamic_traj_generator {

using LogSink = std::function<void(const std::string &)>;

// Enable or disable runtime logging. Default: ON (preserves prior behavior).
// Has no effect when the library was compiled with
// -DDYNAMIC_TRAJECTORY_GENERATOR_ENABLE_LOGGING=OFF, because in that case the
// DYNAMIC_LOG(...) macro is compiled out entirely and never consults the flag.
void setLoggingEnabled(bool enabled);
bool isLoggingEnabled();

// Install a custom sink for the formatted log messages. Pass nullptr to
// restore the default sink (writes to std::cout with a trailing newline).
//
// Typical use to route messages to the rclcpp logger of the owning ROS 2 node:
//   dynamic_traj_generator::setLogSink(
//     [node](const std::string & msg) {
//       RCLCPP_INFO(node->get_logger(), "%s", msg.c_str());
//     });
//
// Thread-safe: the sink may be reassigned at any time. The library serializes
// access to the sink with an internal mutex.
void setLogSink(LogSink sink);

namespace logging_internal {
// Implementation hook used by the DYNAMIC_LOG macro. Defined in
// src/utils/logging_utils.cpp so that the underlying state is unique across
// translation units and shared libraries.
void emit(const std::string & message);
}  // namespace logging_internal

}  // namespace dynamic_traj_generator

#ifndef __SCREEN_OUTPUT__
#define DYNAMIC_LOG(...) ;
#else
#define DYNAMIC_LOG(x)                                                       \
  do {                                                                       \
    if (::dynamic_traj_generator::isLoggingEnabled()) {                      \
      std::ostringstream _dtg_oss;                                           \
      _dtg_oss << '[' << #x << "]: " << (x);                                 \
      ::dynamic_traj_generator::logging_internal::emit(_dtg_oss.str());      \
    }                                                                        \
  } while (0)
#endif  // __SCREEN_OUTPUT__

#endif  // __LOGGING_UTILS_HPP__
