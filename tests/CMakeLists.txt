find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(unit_tests unit/test_buffer.cpp)

# target_compile_options(unit_tests PRIVATE -fsanitize=thread)
# target_link_options(unit_tests PRIVATE -fsanitize=thread)

target_include_directories(unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(unit_tests PRIVATE GTest::gtest GTest::gtest_main
                                         Threads::Threads ${OpenCV_LIBS})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(unit_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME BufferTestSuite COMMAND unit_tests)
