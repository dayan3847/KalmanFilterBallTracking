cmake_minimum_required(VERSION 3.26)
project(practice5)

set(CMAKE_CXX_STANDARD 17)

set(OpenCV_FOUND 1)
find_package(OpenCV REQUIRED)

set(
        SOURCE_FILES
        third_party_libraries/arturoemx/functions.h
        third_party_libraries/arturoemx/Circle/Circle.cpp
        third_party_libraries/arturoemx/Circle/Circle.h
        src/tools/tools.h
        src/tools/Config.h
        src/tools/functions.h
        src/kf/KalmanFilterType.h
        src/kf/KalmanFilter.h
        src/kf/KalmanFilterExtended.h
        src/kf/KalmanFilterExtendedImplicit.h
        src/ball_tracking/BallTrackingKalmanFilterExtended.h
        src/ball_tracking/BallTrackingKalmanFilterExtendedImplicit.h
)

add_executable(SegColorLab kalman_filter_ball_tracking.cpp ${SOURCE_FILES})
target_link_libraries(SegColorLab ${OpenCV_LIBS})
