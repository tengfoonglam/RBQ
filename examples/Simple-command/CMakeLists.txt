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
    CycloneDDS-CXX::ddscxx
    CycloneDDS::ddsc
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${RCL_INCLUDE_DIR}
)


target_compile_definitions(${PROJECT_NAME} PRIVATE
    APP_NAME=\"${PROJECT_NAME}\"
)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
