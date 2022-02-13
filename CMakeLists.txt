cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME dynamic_trajectory_generator)
project(${PROJECT_NAME})

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_subdirectory(subpackages/mav_trajectory_generation)

# CPP matplotlib workaround
set (CMAKE_CXX_FLAGS " -I/usr/include/python3.8 -lpython3.8")
add_subdirectory(subpackages/matplotlib_cpp)
include_directories(subpackages/matplotlib_cpp)
# include(cmake_extern_dependencies/matplotlib_cpp_dependencies.cmake)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(SOURCE_CPP_FILES 
  src/dynamic_trajectory.cpp
  src/dynamic_waypoint.cpp
  # src/eth_traj_wrapper.cpp
  # src/trajectory_generator.cpp
  # src/trajectory_publisher.cpp
)

include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
  ${MATPLOTLIB_CPP_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_node src/main_trajectory_publisher.cpp ${SOURCE_CPP_FILES} ${HEADER_HPP_FILES})
# target_link_libraries(${PROJECT_NAME}_node nlopt glog::glog

#create library and export it
add_library(${PROJECT_NAME} ${SOURCE_CPP_FILES})
target_link_libraries(${PROJECT_NAME} mav_trajectory_generation nlopt )
target_include_directories(${PROJECT_NAME} PUBLIC include include/${PROJECT_NAME} ${EIGEN3_INCLUDE_DIRS} ${MATPLOTLIB_CPP_INCLUDE_DIRS})

#check if tests are enabled
if (${PROJECT_NAME}_TESTS_ENABLED)
  add_executable(compilation_tests tests/compilation_tests.cpp )
  target_link_libraries(compilation_tests ${PROJECT_NAME} mav_trajectory_generation python3.8)
endif()
