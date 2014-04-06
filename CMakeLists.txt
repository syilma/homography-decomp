cmake_minimum_required(VERSION 2.8)

project(HomographyDecomp)

add_subdirectory(src)

option(WITH_TESTS "Build Tests" ON)

if(WITH_TESTS)
    add_subdirectory(test)
endif()

find_package(OpenCV REQUIRED)

