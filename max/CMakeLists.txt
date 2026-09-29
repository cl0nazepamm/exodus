# udimpacker for 3ds Max 2027: the SDK as installed, C++20, the compiler settings of the SDK's
# own property sheets (maxsdk/ProjectSettings/propertySheets).
set(MAXSDK_PATH "C:/Program Files/Autodesk/3ds Max 2027 SDK/maxsdk" CACHE PATH "3ds Max 2027 SDK (maxsdk folder)")
if(NOT EXISTS "${MAXSDK_PATH}/include/max.h")
    message(WARNING "3ds Max 2027 SDK not found at ${MAXSDK_PATH}; the plugin is not built (set MAXSDK_PATH)")
    return()
endif()

function(udim_max_settings target)
    target_include_directories(${target} PRIVATE "${MAXSDK_PATH}/include" "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_directories(${target} PRIVATE "${MAXSDK_PATH}/lib/x64/Release")
    target_compile_definitions(${target} PRIVATE
        UNICODE _UNICODE WIN32 WIN64 _WINDOWS WIN32_LEAN_AND_MEAN NOMINMAX
        _CRT_SECURE_NO_DEPRECATE _CRT_NONSTDC_NO_DEPRECATE _SCL_SECURE_NO_DEPRECATE ISOLATION_AWARE_ENABLED=1)
    target_compile_options(${target} PRIVATE /permissive- /Zc:__cplusplus /Zc:lambda /Zc:inline /GR /EHsc /utf-8 /W3 /MP)
endfunction()

# The work Max runs, in a DLL of its own: the plugin loads it from a copy, so a rebuild can
# replace it while Max is running and udimpacker.reload() picks it up.
add_library(udimpacker_maxcore SHARED maxcore.cpp)
udim_max_settings(udimpacker_maxcore)
target_link_libraries(udimpacker_maxcore PRIVATE udimpacker_core core maxutil geom mesh mnmath poly paramblk2 bmm)
set_target_properties(udimpacker_maxcore PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/max/core/$<0:>")

# The plugin Max loads at startup (registered through Plugin.UserSettings.ini, see README).
add_library(udimpacker_max SHARED plugin.cpp unsubdivide.cpp udimpacker.rc plugin.def)
udim_max_settings(udimpacker_max)
target_link_libraries(udimpacker_max PRIVATE core maxutil geom mesh mnmath poly paramblk2 gup)
set_target_properties(udimpacker_max PROPERTIES
    OUTPUT_NAME "udimpacker"
    SUFFIX ".gup"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/max/plugin/$<0:>")
add_dependencies(udimpacker_max udimpacker_maxcore)
