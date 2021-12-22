find_package(glog QUIET)
if (${glog_FOUND})
  MESSAGE(STATUS "Found glog.")
else (${glog_FOUND})
  MESSAGE(STATUS "Could not locate glog.")
  include(FetchContent)
  FetchContent_Declare(
    glog
    URL https://github.com/google/glog/archive/4ffa98388f8a28c55b0c8acfbba5f62df954c2a4.zip
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(glog)
endif(${glog_FOUND})

