cmake_minimum_required (VERSION 3.8)

project ("myPathTracer")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable (myPathTracer "myPathTracer.cpp"  "fpng.h"  "primitive.cpp" "primitive.h" "scene.cpp" "scene.h")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET myPathTracer PROPERTY CXX_STANDARD 20)
endif()

