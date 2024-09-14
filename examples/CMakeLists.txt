# Minimum CMake version
cmake_minimum_required(VERSION 3.10)

# Project name
project(NovaThrust_Examples)

# Set output directory for examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)

# Add example source files
set(EXAMPLE_SOURCES
    thrust_calculator_example.cpp
    performance_metrics_example.cpp
    multi_tube_pde_simulation.cpp
    # Add other example files here
)

# Create executables for each example
foreach(example_source ${EXAMPLE_SOURCES})
    get_filename_component(example_name ${example_source} NAME_WE)
    add_executable(${example_name} ${example_source})
    target_link_libraries(${example_name}
        PRIVATE
        NovaThrust_lib
        ${FFTW_LIB}
        ${OpenBLAS_LIBRARIES}
        Eigen3::Eigen
    )
    target_include_directories(${example_name} PRIVATE 
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${EIGEN3_INCLUDE_DIR}
    )
endforeach()

# Optional: Add custom target for running all examples
add_custom_target(run_examples)
foreach(example_source ${EXAMPLE_SOURCES})
    get_filename_component(example_name ${example_source} NAME_WE)
    add_custom_target(run_${example_name}
        COMMAND ${example_name}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Running ${example_name}"
    )
    add_dependencies(run_examples run_${example_name})
endforeach()