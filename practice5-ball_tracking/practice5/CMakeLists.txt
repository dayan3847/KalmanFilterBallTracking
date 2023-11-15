cmake_minimum_required(VERSION 3.26)
project(practice5)

set(CMAKE_CXX_STANDARD 17)

set(OpenCV_FOUND 1)
find_package(OpenCV REQUIRED)

set(
        SOURCE_FILES
        lib/arturo/functions.h
        lib/arturo/Circle/Circle.cpp
        lib/arturo/Circle/Circle.h
        lib/dayan/tools.h
        lib/dayan/Config.h
        lib/dayan/functions.h
        lib/dayan/KalmanFilterType.h
        lib/dayan/KalmanFilter.h
        lib/dayan/KalmanFilterExtended.h
        lib/dayan/KalmanFilterExtendedImplicit.h
        lib/dayan/BallTrackingKalmanFilterExtended.h
        lib/dayan/BallTrackingKalmanFilterExtendedImplicit.h
)

add_executable(SegColorLab SegColorLab.cpp ${SOURCE_FILES})
target_link_libraries(SegColorLab ${OpenCV_LIBS})
