cmake_minimum_required(VERSION 3.20)
project(virtual_camera LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Debug info by default so gdb/perf show line numbers.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_compile_options(-Wall -Wextra)

# Step 1: one-frame V4L2 capture
add_executable(capture_one capture/capture_one.c)

# Step 2+: reusable capture module, linked into the preview and the pipeline.
# A static library (.a) is just the compiled .o files bundled together;
# programs that link it get the code copied into their own executable.
add_library(v4l2_capture STATIC capture/v4l2_capture.c)
# Anyone linking v4l2_capture can #include "v4l2_capture.h" without a path.
target_include_directories(v4l2_capture PUBLIC capture)

# Step 2+: C++ processing code (colour conversion; filters arrive in Step 3).
add_library(pipeline STATIC pipeline/convert.cpp)
target_include_directories(pipeline PUBLIC pipeline)

# Step 2: live preview window. SDL2 ships a CMake config file
# (/usr/lib/x86_64-linux-gnu/cmake/SDL2), which defines the SDL2::SDL2 target:
# linking it adds SDL's include path and library in one go.
find_package(SDL2 REQUIRED CONFIG)
add_executable(preview pipeline/preview.cpp)
target_link_libraries(preview PRIVATE pipeline v4l2_capture SDL2::SDL2)
