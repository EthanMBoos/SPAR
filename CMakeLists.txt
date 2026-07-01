cmake_minimum_required(VERSION 3.22)
project(SPAR VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Strict warnings
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# Optional external dependencies — controlled by CMake options.
option(SPAR_ENABLE_ONNXRUNTIME  "Build with ONNX Runtime support (OnnxNavigateNode)" OFF)
option(SPAR_ENABLE_ZENOH        "Build with Zenoh-cpp sensor sources"  OFF)
option(SPAR_ENABLE_PYTHON       "Build pybind11 Python bindings (SparEnv gym wrapper)" OFF)

# Active backend: kinematic | ros2
#   kinematic: in-process bicycle model, no external process needed (training, eval)
#   ros2:      live vehicle via Ros2Adapter over Zenoh (rmw_zenoh); publishes cmd_vel,
#              consumes odometry from the external state estimator (robot_localization)
set(SPAR_BACKEND "kinematic" CACHE STRING "Active backend (kinematic|ros2)")

if(SPAR_ENABLE_ONNXRUNTIME)
    add_definitions(-DSPAR_HAVE_ONNXRUNTIME)
    find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
        PATHS /opt/homebrew/opt/onnxruntime/include/onnxruntime
              /usr/local/include/onnxruntime
        REQUIRED)
    find_library(ONNXRUNTIME_LIB onnxruntime
        PATHS /opt/homebrew/opt/onnxruntime/lib
              /usr/local/lib
        REQUIRED)
endif()

# OpenSSL for SHA-256 in admission (soft dependency)
find_package(OpenSSL QUIET)
if(OpenSSL_FOUND)
    add_definitions(-DSPAR_HAVE_OPENSSL)
endif()

add_subdirectory(spar_rover)

if(SPAR_ENABLE_PYTHON)
    add_subdirectory(python)
endif()
