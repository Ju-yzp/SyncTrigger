cmake_minimum_required(VERSION 3.24)

project(SyncTrigger 
    VERSION 1.0.0 
    DESCRIPTION "A lightweight header-only multi-sensor synchronization framework"
    LANGUAGES CXX
)

add_library(SyncTrigger INTERFACE)
add_library(SyncTrigger::SyncTrigger ALIAS SyncTrigger)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

target_include_directories(SyncTrigger INTERFACE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

install(TARGETS SyncTrigger
    EXPORT SyncTriggerTargets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT SyncTriggerTargets
    FILE SyncTriggerTargets.cmake
    NAMESPACE SyncTrigger::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SyncTrigger
)

write_basic_package_version_file(
    "SyncTriggerConfigVersion.cmake"
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY SameMajorVersion
)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/SyncTriggerConfig.cmake"
    "include(\"\${CMAKE_CURRENT_LIST_DIR}/SyncTriggerTargets.cmake\")\n"
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/SyncTriggerConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/SyncTriggerConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/SyncTrigger
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

option(SYNCTRIGGER_BUILD_EXAMPLES "Build example applications" ON)

if(SYNCTRIGGER_BUILD_EXAMPLES)
    find_package(OpenCV REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(depthai CONFIG REQUIRED)

    add_executable(dataBuffer examples/test_dataBuffer.cpp)
    target_link_libraries(dataBuffer PRIVATE 
        SyncTrigger::SyncTrigger 
        Eigen3::Eigen 
        depthai::core 
        ${OpenCV_LIBS}
    )

    add_executable(synchronizer examples/test_synchronizer.cpp)
    target_link_libraries(synchronizer PRIVATE 
        SyncTrigger::SyncTrigger 
        Eigen3::Eigen 
        depthai::core 
        ${OpenCV_LIBS}
    )
endif()