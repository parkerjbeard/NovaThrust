# Minimum CMake version
cmake_minimum_required(VERSION 3.10)

# Project name
project(NovaThrust_Tests)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find GTest package
find_package(GTest REQUIRED)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
    ${GTEST_INCLUDE_DIRS}
)

# Add test source files
set(TEST_SOURCES
    thrust_calculator_test.cpp
    # Add other test files here
)

# Create test executable
add_executable(NovaThrust_tests ${TEST_SOURCES})

# Link libraries
target_link_libraries(NovaThrust_tests
    PRIVATE
    NovaThrust_lib
    GTest::GTest
    GTest::Main
)

# Add the test
add_test(NAME NovaThrust_tests COMMAND NovaThrust_tests)

# Optional: Set working directory for tests
set_tests_properties(NovaThrust_tests PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

# Optional: Add custom target for running tests
add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS NovaThrust_tests
)