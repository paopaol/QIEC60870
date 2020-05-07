cmake_minimum_required(VERSION 3.10)

project(QIEC60870)


option(QIEC60870_BUILD_TEST "build unit test" ON)


set(IEC60870_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(IEC60870_BUILD_ROOT ${CMAKE_CURRENT_BINARY_DIR})

if(NOT WIN32)
  add_compile_options("-std=c++11")
endif()

if (WIN32)
  add_compile_options(/source-charset:utf-8)
  add_compile_options(/execution-charset:utf-8)
  add_compile_options(/wd4018 /wd4267 /wd4305 /wd4800)
  add_compile_options(/WX-)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-D_SCL_SECURE_NO_WARNINGS)
  add_definitions(-DNOMINMAX)
endif()

add_subdirectory(thrd)
add_subdirectory(iec60870)
