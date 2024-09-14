cmake_minimum_required(VERSION 3.10)
project(NovaThrust)

set(CMAKE_CXX_STANDARD 17)

# Set OpenBLAS_HOME if not already set
if(NOT DEFINED OpenBLAS_HOME)
    set(OpenBLAS_HOME "/usr/local/opt/openblas")
endif()

# Add OpenBLAS to the prefix path
list(APPEND CMAKE_PREFIX_PATH ${OpenBLAS_HOME})

# Find required packages
find_library(FFTW_LIB fftw3 REQUIRED)
find_path(FFTW_INCLUDES fftw3.h REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Find OpenBLAS
find_package(OpenBLAS REQUIRED)
if(OpenBLAS_FOUND)
    add_definitions(-DHAVE_BLAS -DHAVE_LAPACK)
else()
    message(FATAL_ERROR "OpenBLAS not found. Please install OpenBLAS or set OpenBLAS_HOME.")
endif()

# Find GTest
find_package(GTest REQUIRED)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
    ${FFTW_INCLUDES}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenBLAS_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)

# Add source files for the library
file(GLOB_RECURSE LIB_SOURCES 
    "src/*.cpp"
)

# Create a library for NovaThrust
add_library(NovaThrust_lib STATIC ${LIB_SOURCES})

# Add source files for the main executable
set(MAIN_SOURCES main.cpp)

# Add executable
add_executable(NovaThrust ${MAIN_SOURCES})

# Link libraries to the main executable
target_link_libraries(NovaThrust PRIVATE 
    NovaThrust_lib
    ${FFTW_LIB}
    ${OpenBLAS_LIBRARIES}
    Eigen3::Eigen
)

# Explicitly add include directories to the target
target_include_directories(NovaThrust PRIVATE 
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
)

# Enable testing
enable_testing()

# Add the tests subdirectory
add_subdirectory(tests)

option(BUILD_EXAMPLES "Build the examples" ON)

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
