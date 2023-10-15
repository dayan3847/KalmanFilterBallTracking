cmake_minimum_required(VERSION 3.26)
project(practice5)

set(CMAKE_CXX_STANDARD 17)

set(OpenCV_FOUND 1)
find_package(OpenCV REQUIRED)

add_executable(practice5 main.cpp)
target_link_libraries(practice5 ${OpenCV_LIBS})
