
find_package(nlopt QUIET)
if (${NLopt_FOUND})
  MESSAGE(STATUS "Found NLopt.")
else (${NLopt_FOUND})
  MESSAGE(STATUS "Could not locate NLopt.")
  include(FetchContent)
  FetchContent_Declare(
    NLopt
    URL https://github.com/stevengj/NLopt/archive/09b3c2a6da71cabcb98d2c8facc6b83d2321ed71.zip
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(NLopt)
endif(${NLopt_FOUND})
