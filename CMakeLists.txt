# CMakeList.txt: myPathTracer 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.8)

project ("myPathTracer")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
# 将源代码添加到此项目的可执行文件。
add_executable (myPathTracer "myPathTracer.cpp"  "fpng.h"  "primitive.cpp" "primitive.h" "scene.cpp" "scene.h")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET myPathTracer PROPERTY CXX_STANDARD 20)
endif()

# TODO: 如有需要，请添加测试并安装目标。
