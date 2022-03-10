#!/bin/bash

set -e

if [[ $(pidof gst-launch-1.0) ]]; then
        echo Stopping gstreamer active streams
        sudo kill -9 `pidof gst-launch-1.0`
fi

sudo modprobe -r uvcvideo
sudo modprobe -r v4l2loopback
sleep 2
sudo modprobe v4l2loopback devices=2 exclusive_caps=0,0
sleep 4

sudo modprobe uvcvideo

sleep 5

# v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat='YUYV'
# v4l2-ctl -d /dev/video0 -c exposure_absolute=305

gst-launch-1.0 v4l2src device=/dev/video2 ! video/x-raw,framerate=30/1,width=640,height=480 ! videoflip method=horizontal-flip ! videoconvert ! tee ! v4l2sink device=/dev/video0 sync=false &

