cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME mav_trajectory_generation)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(geometry_msgs REQUIRED)
# find_package(glog REQUIRED)

include(cmake_dependencies/glog_dependencies.cmake)
message(STATUS "GLOG Used config file: ${glog_CONFIG}")

find_package(Eigen3 REQUIRED)

include("cmake_dependencies/nlopt_dependencies.cmake")
# find_package(NLopt REQUIRED)
find_package(yaml-cpp REQUIRED) # Link against system catkin yaml-cpp if installed.
message(STATUS "Used config file: ${yaml-cpp_CONFIG}")

if(${yaml_cpp_catkin_FOUND})
    message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
    set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
    set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
else()
    message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
    # pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
endif()

#############
# LIBRARIES #
#############

include_directories(
  include
  include/${PROJECT_NAME}
  include/mav_msgs
  ${EIGEN3_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

set(SOURCE_CPP_FILES 
  src/motion_defines.cpp
  src/polynomial.cpp
  src/segment.cpp
  src/timing.cpp
  src/trajectory.cpp
  src/trajectory_sampling.cpp
  src/vertex.cpp
  src/io.cpp
  src/rpoly/rpoly_ak1.cpp
  src/polynomial_timing_evaluation.cpp
)

set(PROJECT_DEPENDENCIES
  geometry_msgs
  yaml-cpp
  Eigen3
)

add_library(${PROJECT_NAME} ${SOURCE_CPP_FILES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_dependencies(${PROJECT_NAME} glog::glog nlopt yaml-cpp )
target_link_libraries(${PROJECT_NAME}
  glog::glog
  nlopt
  yaml-cpp
  ${EIGEN3_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

add_executable(test_polynomial test/test_polynomial.cpp)
target_link_libraries(test_polynomial ${PROJECT_NAME} nlopt glog::glog yaml-cpp)

add_test(TRAJ_TEST test_polynomial)
