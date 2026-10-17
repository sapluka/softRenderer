cmake_minimum_required(VERSION 3.20)

project(renderer LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Eigen is header-only. This vendored copy omits directories required by its
# upstream CMakeLists.txt (such as unsupported), so expose its headers directly.
add_library(eigen INTERFACE)
add_library(Eigen3::Eigen ALIAS eigen)
target_include_directories(eigen SYSTEM INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/eigen"
)
target_compile_features(eigen INTERFACE cxx_std_14)

# This is GLAD 2: compile gl.c as C, and include <glad/gl.h> from C++ code.
add_library(glad STATIC lib/glad/src/gl.c)
target_include_directories(glad SYSTEM PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/glad/include"
)
set_target_properties(glad PROPERTIES
    C_STANDARD 99
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS OFF
)

# Build the bundled GLFW sources with the selected compiler, including MinGW.
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Build GLFW examples" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "Build GLFW tests" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "Build GLFW documentation" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "Generate GLFW installation rules" FORCE)
set(GLFW_LIBRARY_TYPE STATIC CACHE STRING "Build GLFW as a static library" FORCE)
add_subdirectory(lib/glfw EXCLUDE_FROM_ALL)

# Reconfigure automatically when source files are added or removed.
file(GLOB_RECURSE RENDERER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
)

# The current source files are empty. Once src/main.cpp is added, this target
# becomes an executable without requiring another CMakeLists.txt edit.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
    add_executable(renderer ${RENDERER_SOURCES})
else()
    add_library(renderer STATIC ${RENDERER_SOURCES})
    message(STATUS "No src/main.cpp yet; building renderer as a static library")
endif()

target_compile_features(renderer PUBLIC cxx_std_17)
set_target_properties(renderer PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(renderer PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Let GLAD provide the OpenGL declarations, even if GLFW is included first.
target_compile_definitions(renderer PUBLIC GLFW_INCLUDE_NONE)
target_link_libraries(renderer PUBLIC Eigen3::Eigen glad glfw)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(renderer PRIVATE -Wall -Wextra -Wpedantic)
endif()
