cmake_minimum_required(VERSION 3.24)
project(
  SyncTrigger
  VERSION 1.0.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(depthai CONFIG REQUIRED)

add_executable(synchronizer test_synchronizer.cpp)
target_include_directories(synchronizer PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(
  synchronizer
  PUBLIC Eigen3::Eigen
   depthai::core
         ${OpenCV_LIBS})