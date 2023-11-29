cmake_minimum_required(VERSION 3.22)
project(KalmanFilterBallTracking)

set(CMAKE_CXX_STANDARD 17)

set(OpenCV_FOUND 1)
find_package(OpenCV REQUIRED)

set(
        SOURCE_FILES
        lib/arturoemx/functions.h
        lib/arturoemx/Circle/Circle.cpp
        lib/arturoemx/Circle/Circle.h
        src/tools/tools.h
        src/tools/Config.h
        src/tools/functions.h
        src/kf/KalmanFilterType.h
        src/kf/KalmanFilter.h
        src/kf/KalmanFilterExtended.h
        src/kf/KalmanFilterExtendedImplicit.h
        src/ball_tracking/BallTrackingKalmanFilterExtended.h
        src/ball_tracking/BallTrackingKalmanFilterExtendedImplicit.h
        src/kf/KalmanFilterUnscented.h
        src/ball_tracking/BallTrackingKalmanFilterUnscented.h
        src/ball_tracking/BallTrackingKalmanFilterExtended_6x6.h
        src/ball_tracking/BallTrackingKalmanFilterExtended_9x8.h
        src/ball_tracking/BallTrackingKalmanFilterExtended_9x9.h
)

add_executable(KalmanFilterBallTracking kalman_filter_ball_tracking.cpp ${SOURCE_FILES})
target_link_libraries(KalmanFilterBallTracking ${OpenCV_LIBS})
