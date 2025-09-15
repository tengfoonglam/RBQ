cmake_minimum_required(VERSION 3.22)

project(Simple-RL LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-Wl,--gc-sections -s")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")

add_executable(${PROJECT_NAME} main.cpp JointControl.h PolicyRunner.h PolicyRunner.cpp)

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
    Qt5::Core
    Qt5::Network
    ${RBDL_LIBRARY}
    onnxruntime::onnxruntime
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${RCL_INCLUDE_DIR}
    ${RBQ_INCLUDE_DIR}
)

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

get_target_property(ONNX_RUNTIME_LOCATION onnxruntime::onnxruntime IMPORTED_LOCATION_RELEASE)


target_compile_definitions(${PROJECT_NAME} PRIVATE
    APP_NAME=\"${PROJECT_NAME}\"
)

