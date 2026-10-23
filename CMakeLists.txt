cmake_minimum_required(VERSION 3.20)

project(renderer LANGUAGES C CXX)

# Declare the standard explicitly so compile_commands.json also tells editors.
# GCC 15 already defaults to C++17, but clangd's Windows default is C++14.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Eigen is header-only. This vendored copy omits directories required by its
# upstream CMakeLists.txt (such as unsupported), so expose its headers directly.
add_library(eigen INTERFACE)
add_library(Eigen3::Eigen ALIAS eigen)
target_include_directories(eigen SYSTEM INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/eigen"
)
target_compile_features(eigen INTERFACE cxx_std_14)

# Keep OBJ/MTL parsing independent from the window and rendering code.
add_library(obj_loader STATIC src/objLoader.cpp)
target_compile_features(obj_loader PUBLIC cxx_std_17)
set_target_properties(obj_loader PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(obj_loader PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_include_directories(obj_loader SYSTEM PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/tinyobjloader"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/earcut/include"
)
target_link_libraries(obj_loader PUBLIC Eigen3::Eigen)

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
list(REMOVE_ITEM RENDERER_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/objLoader.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp"
)

# Keep reusable rendering code linkable by the application and the tests.
add_library(renderer STATIC ${RENDERER_SOURCES})

target_compile_features(renderer PUBLIC cxx_std_17)
set_target_properties(renderer PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(renderer PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Let GLAD provide the OpenGL declarations, even if GLFW is included first.
target_compile_definitions(renderer PUBLIC GLFW_INCLUDE_NONE)
target_compile_definitions(renderer PRIVATE
    "RENDERER_SHADER_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/shader\""
)
target_link_libraries(renderer PUBLIC obj_loader Eigen3::Eigen glad glfw)

# An empty main.cpp is a placeholder, not an application entry point.
set(RENDERER_MAIN "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${RENDERER_MAIN}")
if(EXISTS "${RENDERER_MAIN}")
    file(SIZE "${RENDERER_MAIN}" RENDERER_MAIN_SIZE)
    if(RENDERER_MAIN_SIZE GREATER 0)
        add_executable(renderer_app "${RENDERER_MAIN}")
        target_link_libraries(renderer_app PRIVATE renderer)
        set_target_properties(renderer_app PROPERTIES OUTPUT_NAME renderer)
    endif()
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(obj_loader_tests tests/objLoaderTests.cpp)
    target_link_libraries(obj_loader_tests PRIVATE obj_loader)
    add_test(NAME obj_loader_tests COMMAND obj_loader_tests
        "${CMAKE_CURRENT_SOURCE_DIR}/assets")

    add_executable(camera_tests tests/cameraTests.cpp)
    target_link_libraries(camera_tests PRIVATE renderer)
    add_test(NAME camera_tests COMMAND camera_tests)

    add_executable(rasterizer_draw_tests tests/rasterizerDrawTests.cpp)
    target_link_libraries(rasterizer_draw_tests PRIVATE renderer)
    add_test(NAME rasterizer_draw_tests COMMAND rasterizer_draw_tests)
    set_tests_properties(rasterizer_draw_tests PROPERTIES SKIP_RETURN_CODE 77)
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(renderer PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(obj_loader PRIVATE -Wall -Wextra -Wpedantic)
endif()
