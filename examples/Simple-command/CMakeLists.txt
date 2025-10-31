cmake_minimum_required(VERSION 3.22)

project(Simple-command LANGUAGES CXX)

add_executable(${PROJECT_NAME} main.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN"
    SKIP_BUILD_RPATH FALSE
    BUILD_RPATH "$ORIGIN"
)

target_link_libraries(${PROJECT_NAME}
    pthread
    rt
    Eigen3::Eigen
    ${RCL_LIBRARY}
    onnxruntime::onnxruntime
    CycloneDDS-CXX::ddscxx
    CycloneDDS::ddsc
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${RCL_INCLUDE_DIR}
)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

get_target_property(ONNX_RUNTIME_LOCATION onnxruntime::onnxruntime IMPORTED_LOCATION_RELEASE)
install(FILES ${ONNX_RUNTIME_LOCATION}
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    APP_NAME=\"${PROJECT_NAME}\"
)
