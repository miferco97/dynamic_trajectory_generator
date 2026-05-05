cmake_minimum_required(VERSION 3.16)
set(PYBIND_TARGET _dynamic_trajectory_generator_cpp)
set(PY_PACKAGE    dynamic_trajectory_generator_py)
project(${PYBIND_TARGET})

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# For shared libraries
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
set(PROJECT_DEPENDENCIES
  Eigen3
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

find_package(pybind11 CONFIG REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Pull the C++ core library from the parent if not already a target.
if(NOT TARGET dynamic_trajectory_generator)
  set(BUILD_TESTING OFF CACHE BOOL "Disable tests in parent project" FORCE)
  add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_BINARY_DIR}/dynamic_trajectory_generator_build
  )
endif()

# pybind11 module
pybind11_add_module(${PYBIND_TARGET} src/module.cpp)

target_include_directories(${PYBIND_TARGET} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PYBIND_TARGET} PRIVATE dynamic_trajectory_generator)

set_target_properties(${PYBIND_TARGET} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  INSTALL_RPATH "\$ORIGIN"
)

# Local mirror for PYTHONPATH usage. When PYBIND_PY_MIRROR_ROOT is set by a
# parent project (umbrella build), all sub-pybinds share the same python/
# tree. Otherwise fall back to the current binary dir.
set(PYTHON_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/python/${PY_PACKAGE}")
if(DEFINED PYBIND_PY_MIRROR_ROOT)
  set(LOCAL_PY_MIRROR_ROOT "${PYBIND_PY_MIRROR_ROOT}")
else()
  set(LOCAL_PY_MIRROR_ROOT "${CMAKE_CURRENT_BINARY_DIR}/python")
endif()
set(PYTHON_PACKAGE_BUILD_DIR  "${LOCAL_PY_MIRROR_ROOT}/${PY_PACKAGE}")

file(MAKE_DIRECTORY "${PYTHON_PACKAGE_BUILD_DIR}")
set_target_properties(${PYBIND_TARGET} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${PYTHON_PACKAGE_BUILD_DIR}"
)

add_custom_command(TARGET ${PYBIND_TARGET} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${PYTHON_PACKAGE_SOURCE_DIR}"
    "${PYTHON_PACKAGE_BUILD_DIR}"
  COMMENT "Mirroring ${PY_PACKAGE} sources to build tree"
)

# pip install via scikit-build-core
if(DEFINED SKBUILD_PLATLIB_DIR)
  install(DIRECTORY "${PYTHON_PACKAGE_SOURCE_DIR}/"
          DESTINATION "${SKBUILD_PLATLIB_DIR}/${PY_PACKAGE}"
  )
  install(TARGETS ${PYBIND_TARGET}
    LIBRARY DESTINATION "${SKBUILD_PLATLIB_DIR}/${PY_PACKAGE}"
    RUNTIME DESTINATION "${SKBUILD_PLATLIB_DIR}/${PY_PACKAGE}"
    ARCHIVE DESTINATION "${SKBUILD_PLATLIB_DIR}/${PY_PACKAGE}"
  )
  # Install transitive shared libraries so the wheel is self-contained.
  # RPATH is set to $ORIGIN so co-located libs resolve each other at runtime.
  foreach(_dep_target IN ITEMS dynamic_trajectory_generator mav_trajectory_generation nlopt)
    if(TARGET ${_dep_target})
      get_target_property(_dep_type ${_dep_target} TYPE)
      if(_dep_type STREQUAL "SHARED_LIBRARY")
        set_target_properties(${_dep_target} PROPERTIES
          INSTALL_RPATH "\$ORIGIN"
          BUILD_WITH_INSTALL_RPATH TRUE
        )
        install(TARGETS ${_dep_target}
          LIBRARY DESTINATION "${SKBUILD_PLATLIB_DIR}/${PY_PACKAGE}"
          RUNTIME DESTINATION "${SKBUILD_PLATLIB_DIR}/${PY_PACKAGE}"
        )
      endif()
    endif()
  endforeach()
endif()
