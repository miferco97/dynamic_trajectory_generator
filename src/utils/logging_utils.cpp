#include "dynamic_trajectory_generator/utils/logging_utils.hpp"

#include <atomic>
#include <iostream>
#include <mutex>
#include <utility>

namespace dynamic_traj_generator {

namespace {

// Default ON to preserve the prior behavior of the library (logs visible
// without any explicit configuration when compiled with __SCREEN_OUTPUT__).
std::atomic<bool> & enabledFlag()
{
  static std::atomic<bool> flag{true};
  return flag;
}

std::mutex & sinkMutex()
{
  static std::mutex m;
  return m;
}

LogSink & sinkRef()
{
  static LogSink sink;
  return sink;
}

}  // namespace

void setLoggingEnabled(bool enabled)
{
  enabledFlag().store(enabled, std::memory_order_relaxed);
}

bool isLoggingEnabled()
{
  return enabledFlag().load(std::memory_order_relaxed);
}

void setLogSink(LogSink sink)
{
  std::lock_guard<std::mutex> lock(sinkMutex());
  sinkRef() = std::move(sink);
}

namespace logging_internal {

void emit(const std::string & message)
{
  std::lock_guard<std::mutex> lock(sinkMutex());
  if (sinkRef()) {
    sinkRef()(message);
  } else {
    std::cout << message << std::endl;
  }
}

}  // namespace logging_internal

}  // namespace dynamic_traj_generator
