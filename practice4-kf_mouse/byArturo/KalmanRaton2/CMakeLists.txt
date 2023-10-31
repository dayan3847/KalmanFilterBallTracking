cmake_minimum_required(VERSION 3.26)
project(KalmanRaton)

set(CMAKE_CXX_STANDARD 17)

set(OpenCV_FOUND 1)
find_package(OpenCV REQUIRED)

add_executable(EjemploRaton EjemploRaton.cpp)
target_link_libraries(EjemploRaton ${OpenCV_LIBS})
