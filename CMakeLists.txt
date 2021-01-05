cmake_minimum_required(VERSION 2.8) 
PROJECT (ros_tp)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

include_directories(include)

if (BUILD_EXAMPLES)
    set(BALL_DETECTOR_INCLUDES include/ros_basics_vision/ball_detector.h)
    set(BALL_DETECTOR_SOURCES src/ball_detector.cpp)

    add_executable(ball_detector_example examples/ball_detector_example.cpp ${BALL_DETECTOR_SOURCES})
    target_link_libraries(ball_detector_example ${OpenCV_LIBS})

    set(ARUCO_DETECTOR_INCLUDES include/ros_basics_vision/aruco_detector.h)
    set(ARUCO_DETECTOR_SOURCES src/aruco_detector.cpp)

    add_executable(aruco_detector_example examples/aruco_detector_example.cpp ${ARUCO_DETECTOR_SOURCES})
    target_link_libraries(aruco_detector_example ${OpenCV_LIBS})


    set(ADAPTIVE_GAMMA_INCLUDES include/ros_basics_vision/adaptive_gamma.h)
    set(ADAPTIVE_GAMMA_SOURCES src/adaptive_gamma.cpp)

    add_executable(adaptive_gamma_example examples/adaptive_gamma_example.cpp ${ADAPTIVE_GAMMA_SOURCES})
    target_link_libraries(adaptive_gamma_example ${OpenCV_LIBS})

    set(SETUP_DECTOR_INCLUDES include/ros_basics_vision/setup_detector.h)
    set(SETUP_DECTOR_SOURCES src/setup_detector.cpp)

    add_executable(setup_detector_example examples/setup_detector_example.cpp ${SETUP_DECTOR_SOURCES} ${ADAPTIVE_GAMMA_SOURCES})
    target_link_libraries(setup_detector_example ${OpenCV_LIBS})

endif()

