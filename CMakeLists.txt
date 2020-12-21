cmake_minimum_required(VERSION 2.8) 
PROJECT (ros_tp)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

include_directories(include)

if (BUILD_EXAMPLES)
    set(BALL_DETECTOR_INCLUDES include/ball_detector.h)
    set(BALL_DETECTOR_SOURCES src/ball_detector.cpp)

    add_executable(ball_detector_example examples/ball_detector_example.cpp ${BALL_DETECTOR_SOURCES})
    target_link_libraries(ball_detector_example ${OpenCV_LIBS})

    set(ADAPTIVE_GAMMA_INCLUDES include/adaptive_gamma.h)
    set(ADAPTIVE_GAMMA_SOURCES src/adaptive_gamma.cpp)

    add_executable(adaptive_gamma_example examples/adaptive_gamma_example.cpp ${ADAPTIVE_GAMMA_SOURCES})
    target_link_libraries(adaptive_gamma_example ${OpenCV_LIBS})

endif()

