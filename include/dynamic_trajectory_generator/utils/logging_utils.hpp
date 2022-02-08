#ifndef __LOGGING_UTILS_HPP__
#define __LOGGING_UTILS_HPP__

#include <chrono>
#include <iostream>

#define COUNT_TIME(_function_)                                                                                                                                     \
  {                                                                                                                                                                \
    auto start = std::chrono::steady_clock::now();                                                                                                                 \
    _function_;                                                                                                                                                    \
    auto end = std::chrono::steady_clock::now();                                                                                                                   \
    std::cout << "[" << #_function_ << " ] takes :  " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0f << "s" << std::endl; \
  }

#ifndef __SCREEN_OUTPUT__
#define DYNAMIC_LOG(...) ;
#else
#ifndef RCLCPP__LOGGER_HPP_

template <typename T>
static void PRINT_STRINGS(const char *name, T t)
{
  std::cout << '[' << name << "]: " << t << std::endl;
}
static void PRINT_STRINGS(const char *name, std::string s)
{
  std::cout << s << std::endl;
}
static void PRINT_STRINGS(const char *name, const char *s)
{
  std::cout << s << std::endl;
}

#define DYNAMIC_LOG(x) PRINT_STRINGS(#x, x)

#else // RCLCPP__LOGGER_HPP_

template <typename T>
void PRINT_STRINGS(const char *name, T t)
{
  RCLCPP_INFO(rclcpp::get_logger("dynamic_traj_generator"), "[%s]: %s", name, t);
}
void PRINT_STRINGS(const char *name, std::string s)
{
  RCLCPP_INFO(rclcpp::get_logger("dynamic_traj_generator"), "%s", s.c_str());
}
void PRINT_STRINGS(const char *name, const char *s)
{
  RCLCPP_INFO(rclcpp::get_logger("dynamic_traj_generator"), "%s", s);
}

#define DYNAMIC_LOG(x) PRINT_STRINGS(#x, x)

#endif // RCLCPP__LOGGER_HPP_

#endif // __SCREEN_OUTPUT__

#endif // __LOGGING_UTILS_HPP__
