cmake_minimum_required (VERSION 3.8)

project ("myPathTracer")

include_directories(include)

find_package(Eigen3 REQUIRED)

find_package(freeimage REQUIRED)

add_executable (myPathTracer "main.cpp" "primitive.cpp" "primitive.h" "scene.cpp" "scene.h" "pathtracer.cpp" "pathtracer.h"  "bvh.h" "bvh.cpp" ${include} "light.h" "light.cpp")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET myPathTracer PROPERTY CXX_STANDARD 20)
endif()

target_link_libraries (myPathTracer Eigen3::Eigen)

target_link_libraries(myPathTracer freeimage::FreeImage freeimage::FreeImagePlus)
