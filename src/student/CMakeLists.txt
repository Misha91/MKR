cmake_minimum_required(VERSION 2.8)
     
PROJECT(kalman_filter)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
   message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(VTK REQUIRED
  #COMPONENTS
  #vtkCommonCore
  #vtkInteractionStyle
  #vtkRenderingOpenGL
)
#find_package(Eigen)
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIRS} )
include(${VTK_USE_FILE})


add_subdirectory(gui)
set(LIBS ${LIBS} Gui)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${Eigen_INCLUDE_DIR})
add_executable(KalmanFilter kf_main.cpp systemSimulator/system.cc)

target_link_libraries(KalmanFilter ${LIBS} ${VTK_LIBRARIES})
