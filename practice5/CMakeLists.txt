cmake_minimum_required(VERSION 3.26)
project(practice5)

set(CMAKE_CXX_STANDARD 17)

set(OpenCV_FOUND 1)
find_package(OpenCV REQUIRED)

set(
        SOURCE_FILES
        lib/tools.h
        lib/lib_arturo.h
)

add_executable(SegColorLab SegColorLab.cpp ${SOURCE_FILES})
target_link_libraries(SegColorLab ${OpenCV_LIBS})
