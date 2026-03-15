
find_package(nlopt QUIET)
if (${NLopt_FOUND})
  MESSAGE(STATUS "Found NLopt.")
else (${NLopt_FOUND})
  MESSAGE(STATUS "Could not locate NLopt.")
  include(FetchContent)
  FetchContent_Declare(
    nlopt
    URL https://github.com/stevengj/nlopt/archive/refs/tags/v2.10.0.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(NLopt)
endif(${NLopt_FOUND})
