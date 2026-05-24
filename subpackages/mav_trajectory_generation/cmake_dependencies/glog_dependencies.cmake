find_package(glog QUIET)
if (${glog_FOUND})
  MESSAGE(STATUS "Found glog.")
else (${glog_FOUND})
  MESSAGE(STATUS "Could not locate glog.")
  include(FetchContent)
  FetchContent_Declare(
    glog
    URL https://github.com/google/glog/archive/refs/tags/v0.7.1.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(glog)
endif(${glog_FOUND})

