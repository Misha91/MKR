cmake_minimum_required(VERSION 2.8)
PROJECT(MotionModel)

SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")
add_definitions("-std=c++11")

find_package(VTK REQUIRED)


include(${VTK_USE_FILE})




include_directories ("${PROJECT_SOURCE_DIR}")

add_executable(MotionModel gui.cc mm_main.cc)

target_link_libraries(MotionModel  ${VTK_LIBRARIES})
