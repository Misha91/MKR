cmake_minimum_required(VERSION 2.8)
PROJECT(ParticleFilter)

SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")
add_definitions("-std=c++11")

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories ("${PROJECT_SOURCE_DIR}")

add_subdirectory(dataLoader)
set(LIBS LaserDataLoader)

add_subdirectory(gui)
set(LIBS ${LIBS} Gui)

add_subdirectory(laserSimulator)
set(LIBS ${LIBS} lasersimulator)

add_executable(ParticleFilter pf_main.cpp)
add_executable(ParticleFilter2 "pf_main (low_variance_sampling).cpp")
target_link_libraries(ParticleFilter ${LIBS} ${VTK_LIBRARIES})
target_link_libraries(ParticleFilter2 ${LIBS} ${VTK_LIBRARIES})
