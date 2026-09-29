cmake_minimum_required(VERSION 3.20)
project(udimpacker C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Vendored meshoptimizer: the whole upstream src/ folder, built as-is.
file(GLOB MESHOPT_SOURCES CONFIGURE_DEPENDS third_party/meshoptimizer/*.cpp)
add_library(meshoptimizer STATIC ${MESHOPT_SOURCES})
target_include_directories(meshoptimizer PUBLIC third_party/meshoptimizer)

# The DCC-neutral core, shared by the command-line tool and the 3ds Max plugin.
add_library(udimpacker_core STATIC
    src/atlas.cpp
    src/image.cpp
    src/pack.cpp
    src/bleed.cpp
    src/decimate.cpp
    src/subdiv.cpp
    third_party/lodepng.cpp)
target_include_directories(udimpacker_core PUBLIC src third_party)
target_link_libraries(udimpacker_core PUBLIC meshoptimizer)

add_executable(udimpacker
    src/main.cpp
    src/fbx_input.cpp
    third_party/ufbx.c)
target_link_libraries(udimpacker PRIVATE udimpacker_core)

if(MSVC)
    foreach(target udimpacker_core udimpacker)
        target_compile_options(${target} PRIVATE /utf-8 /W3)
        target_compile_definitions(${target} PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
    endforeach()
endif()

# 3ds Max 2027 plugin (max/): skipped with a warning when the SDK is not installed.
option(UDIM_MAX "Build the 3ds Max 2027 plugin" ON)
if(UDIM_MAX)
    add_subdirectory(max)
endif()
