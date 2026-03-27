cmake_minimum_required(VERSION 3.22)
project(SPAR VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Strict warnings
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# Optional external dependencies — controlled by CMake options so the build
# works out of the box without any of them (stub mode).
option(SPAR_ENABLE_MAVLINK      "Build with MAVLink support"      OFF)
option(SPAR_ENABLE_ONNXRUNTIME  "Build with ONNX Runtime support" OFF)
option(SPAR_ENABLE_ZENOH        "Build with Zenoh-cpp transport"  OFF)

if(SPAR_ENABLE_MAVLINK)
    add_definitions(-DSPAR_HAVE_MAVLINK)
    include_directories(${CMAKE_SOURCE_DIR}/third_party/mavlink/include)
endif()

if(SPAR_ENABLE_ONNXRUNTIME)
    add_definitions(-DSPAR_HAVE_ONNXRUNTIME)
    find_package(onnxruntime REQUIRED)
endif()

# OpenSSL for SHA-256 in admission (soft dependency)
find_package(OpenSSL QUIET)
if(OpenSSL_FOUND)
    add_definitions(-DSPAR_HAVE_OPENSSL)
endif()

add_subdirectory(spar_rover)
