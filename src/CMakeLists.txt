cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED core)

set(trgt HomographyDecomp)

file(GLOB srcFiles *.cpp *.h)

add_library(${trgt} ${srcFiles})

target_link_libraries(${trgt} ${OpenCV_LIBS})


