cmake_minimum_required(VERSION 3.0.2)
project(ros_basics_solution)
PROJECT (ros_basics_vision)

add_compile_options(-std=c++11)

find_package(
    catkin REQUIRED COMPONENTS
    OpenCV REQUIRED
    roscpp
    rospy
    geometry_msgs
    ros_basics_msgs
)

catkin_package(
  CATKIN_DEPENDS ros_basics_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


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

