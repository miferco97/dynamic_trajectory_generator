# Tests CMakeLists.txt

# Test
file(GLOB_RECURSE TEST_SOURCE "*_test.cpp")

if(TEST_SOURCE)
  foreach(TEST_FILE ${TEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_NAME})

    add_test(
      NAME ${PROJECT_NAME}_${TEST_NAME}
      COMMAND ${PROJECT_NAME}_${TEST_NAME}
    )
  endforeach()
endif()

# Google Test tests
file(GLOB_RECURSE GTEST_SOURCE "*_gtest.cpp")

if(GTEST_SOURCE)
  find_package(GTest REQUIRED)

  foreach(TEST_FILE ${GTEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} GTest::gtest ${PROJECT_NAME})

    add_test(
      NAME ${PROJECT_NAME}_${TEST_NAME}
      COMMAND ${PROJECT_NAME}_${TEST_NAME}
    )
  endforeach()
endif()

# Google Benchmark tests
file(GLOB_RECURSE BTEST_SOURCE "*_benchmark.cpp")

if(BTEST_SOURCE)
  find_package(benchmark REQUIRED)

  foreach(TEST_FILE ${BTEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} benchmark::benchmark ${PROJECT_NAME})
  endforeach()
endif()
