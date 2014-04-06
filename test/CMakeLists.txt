cmake_minimum_required(VERSION 2.8)

find_package(OpenCV REQUIRED ts)

set(trgt ut_HomograpyDecomp)

file(GLOB srcFiles *.cpp *.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src)

add_executable(${trgt} ${srcFiles})

target_link_libraries(${trgt}  HomographyDecomp ${OpenCV_LIBS})

