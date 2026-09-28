cmake_minimum_required(VERSION 3.20)
project(udimpacker C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Vendored meshoptimizer: the whole upstream src/ folder, built as-is.
file(GLOB MESHOPT_SOURCES CONFIGURE_DEPENDS third_party/meshoptimizer/*.cpp)
add_library(meshoptimizer STATIC ${MESHOPT_SOURCES})
target_include_directories(meshoptimizer PUBLIC third_party/meshoptimizer)

add_executable(udimpacker
    src/main.cpp
    src/fbx_input.cpp
    src/image.cpp
    src/pack.cpp
    src/bleed.cpp
    src/decimate.cpp
    src/subdiv.cpp
    third_party/ufbx.c
    third_party/lodepng.cpp)

target_include_directories(udimpacker PRIVATE src third_party)
target_link_libraries(udimpacker PRIVATE meshoptimizer)

if(MSVC)
    target_compile_options(udimpacker PRIVATE /utf-8 /W3)
    target_compile_definitions(udimpacker PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
endif()
