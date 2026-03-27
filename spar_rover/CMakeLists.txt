add_executable(spar_rover
    main.cpp
    mission/BehaviorTree.cpp
    mission/NavigateNode.cpp
    monitor/RuntimeMonitor.cpp
    adapter/ArdupilotAdapter.cpp

)

target_include_directories(spar_rover PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/shared
)

if(SPAR_ENABLE_ONNXRUNTIME)
    target_link_libraries(spar_rover PRIVATE onnxruntime)
endif()

if(OpenSSL_FOUND)
    target_link_libraries(spar_rover PRIVATE OpenSSL::Crypto)
endif()
