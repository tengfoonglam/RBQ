cmake_minimum_required(VERSION 3.22)

project(examples LANGUAGES CXX)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffunction-sections -fdata-sections")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-Wl,--gc-sections -s")

find_package(Eigen3 REQUIRED)
find_package(RCL REQUIRED)
find_package(onnxruntime REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS
    Core
    Widgets
    Svg
    Concurrent
    Multimedia
    Network
)
find_package(RBDL REQUIRED)

add_subdirectory(Simple-motion)
add_subdirectory(Simple-command)

