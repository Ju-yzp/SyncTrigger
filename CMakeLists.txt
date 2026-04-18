cmake_minimum_required(VERSION 3.24)

project(
  SyncTrigger
  VERSION 1.0.0
  DESCRIPTION "A lightweight header-only multi-sensor synchronization framework"
  LANGUAGES CXX)

add_library(SyncTrigger INTERFACE)
add_library(SyncTrigger::SyncTrigger ALIAS SyncTrigger)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

target_include_directories(
  SyncTrigger INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)

option(SYNCTRIGGER_BUILD_TESTS "Build unit tests using GTest" ON)
option(SYNCTRIGGER_BUILD_EXAMPLES "Build example applications" ON)

if(SYNCTRIGGER_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(SYNCTRIGGER_BUILD_EXAMPLES)
  find_package(OpenCV REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(depthai CONFIG REQUIRED)
  find_package(Threads REQUIRED)

  add_executable(camera_imu_sync examples/camera_imu_sync.cpp)

  target_link_libraries(
    camera_imu_sync PRIVATE SyncTrigger::SyncTrigger Eigen3::Eigen
                            depthai::core ${OpenCV_LIBS} Threads::Threads)
endif()

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

install(
  TARGETS SyncTrigger
  EXPORT SyncTriggerTargets
  INCLUDES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT SyncTriggerTargets
  FILE SyncTriggerTargets.cmake
  NAMESPACE SyncTrigger::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SyncTrigger)

write_basic_package_version_file(
  "SyncTriggerConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/SyncTriggerConfig.cmake"
     "include(\"\${CMAKE_CURRENT_LIST_DIR}/SyncTriggerTargets.cmake\")\n")

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/SyncTriggerConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/SyncTriggerConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SyncTrigger)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
