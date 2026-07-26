cmake_minimum_required(VERSION 3.10)
project(RoboCheck)

set(CMAKE_CXX_STANDARD 11)

enable_testing()

add_subdirectory(googletest)

include_directories(
    ${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_SOURCE_DIR}/src/generate-AABB
)

add_executable(hello_gtest test/hello_gtest.cpp)
target_link_libraries(hello_gtest gtest gtest_main)

add_test(NAME HelloGTest COMMAND hello_gtest)