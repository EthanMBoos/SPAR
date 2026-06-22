if(NOT SPAR_BACKEND STREQUAL "kinematic")
    message(FATAL_ERROR
        "SPAR_ENABLE_PYTHON requires SPAR_BACKEND=kinematic "
        "(the gym wrapper uses KinematicBackend for training).")
endif()

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 REQUIRED)

pybind11_add_module(spar_bindings spar_bindings.cpp)

target_include_directories(spar_bindings PRIVATE
    ${CMAKE_SOURCE_DIR}/shared
    ${CMAKE_SOURCE_DIR}/spar_rover
)

target_link_libraries(spar_bindings PRIVATE spar_core)
