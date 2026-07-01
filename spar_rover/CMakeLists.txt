# Core library: assembler + mission layer + monitor, no adapter, no main.
# Linked by both spar_rover executable and the pybind11 Python module.
set(SPAR_CORE_SOURCES
    mission/NavigateNode.cpp
    monitor/RuntimeMonitor.cpp
)

if(SPAR_BACKEND STREQUAL "kinematic")
    list(APPEND SPAR_CORE_SOURCES sim/KinematicBackend.cpp)
    add_definitions(-DSPAR_BACKEND_KINEMATIC)
endif()

if(SPAR_ENABLE_ONNXRUNTIME)
    list(APPEND SPAR_CORE_SOURCES mission/OnnxNavigateNode.cpp)
endif()

option(SPAR_ENABLE_EXPLOIT_NODE "Build ExploitNode for the policy_exploit eval scenario" OFF)
if(SPAR_ENABLE_EXPLOIT_NODE)
    list(APPEND SPAR_CORE_SOURCES mission/ExploitNode.cpp)
    add_definitions(-DSPAR_HAVE_EXPLOIT_NODE)
endif()

add_library(spar_core STATIC ${SPAR_CORE_SOURCES})
target_include_directories(spar_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/shared
)
if(SPAR_ENABLE_ONNXRUNTIME)
    target_include_directories(spar_core PUBLIC ${ONNXRUNTIME_INCLUDE_DIR})
    target_link_libraries(spar_core PUBLIC ${ONNXRUNTIME_LIB})
endif()

# Executable: main.cpp, plus the controller adapter when building the ros2 backend.
add_executable(spar_rover main.cpp)
target_link_libraries(spar_rover PRIVATE spar_core)

if(SPAR_BACKEND STREQUAL "ros2")
    target_compile_definitions(spar_rover PRIVATE SPAR_BACKEND_ROS2)
    target_sources(spar_rover PRIVATE adapter/Ros2Adapter.cpp)
    find_package(zenohcpp REQUIRED)
    target_link_libraries(spar_rover PRIVATE zenohcpp::lib)
endif()

if(OpenSSL_FOUND)
    target_link_libraries(spar_rover PRIVATE OpenSSL::Crypto)
endif()
