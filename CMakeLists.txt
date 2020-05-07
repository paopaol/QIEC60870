cmake_minimum_required(VERSION 3.10)

project(QIEC60870)


option(QIEC60870_BUILD_TEST "build unit test" ON)


set(IEC60870_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(IEC60870_BUILD_ROOT ${CMAKE_CURRENT_BINARY_DIR})


add_subdirectory(thrd)
add_subdirectory(iec60870)
