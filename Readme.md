# ROS TP - Computer vision module

This package contains computer vision methods to:

- Adapt to lighting conditions dynamically and reduce image to a region of interest (the setup)
- Track orange ping-pong balls
- Track aruco markers on the robot

## Dependencies

- Opencv3 (with opencv-contrib)

## Adapting to lighting conditions

WIP

## Tracking orange ping-pong balls

<img src="examples/images/pingpong_tracking.gif" alt="ping_pong" width="640"/>

## Tracking aruco markers on the robot

WIP


## Running the examples

To compile the examples you should first invoke the following commands:

```shell
mkdir build; cd build
cmake .. -DBUILD_EXAMPLES=true
```

And then run the examples:

```shell 
# Ball tracking
./ball_detector_example 1 # where 1 is the camera device for your configuration
```