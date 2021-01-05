#include <ros_basics_vision/aruco_detector.h>

#include <ros_basics_msgs/SimplePoseStamped.h>

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>

using namespace ros_tp;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thymio_tracking_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    int camera_dev;
    nh.param<int>("camera_dev", camera_dev, -1);
    if (camera_dev < 0) {
        camera_dev = 6;
        ROS_INFO("Defaulting to camera device %d", camera_dev);
    }

    cv::VideoCapture camera(camera_dev);
    if (!camera.isOpened()) {
        ROS_ERROR("ERROR: Could not open camera");
        return 1;
    }

    int camera_px_width, camera_px_height;
    double pix2mm = 0.05109397995;
    nh.param<int>("camera_px_width", camera_px_width, 640);
    nh.param<int>("camera_px_height", camera_px_height, 480);
    nh.param<double>("pix2mm", pix2mm, pix2mm);

    camera.set(cv::CAP_PROP_FRAME_WIDTH, camera_px_width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, camera_px_height);

    image_transport::Publisher raw_image_pub = it.advertise("robot_view/image_raw", 1);
    image_transport::Publisher annot_image_pub;
    bool annotate_image;
    nh.param<bool>("annotate_image", annotate_image, true);
    if (annotate_image) {
        annot_image_pub = it.advertise("robot_view/image_annot", 1);
    }

    ros::Publisher robot_pose_pub;
    robot_pose_pub = nh.advertise<ros_basics_msgs::SimplePoseStamped>("robot_pose", 1);

    cv::Mat frame;
    ArucoDetector ad;
    ros::Rate loop_rate(30);
    while (ros::ok()) {
        camera >> frame;

        // publish raw image
        sensor_msgs::ImagePtr raw_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        raw_image_pub.publish(raw_msg);

        // detect aruco markers
        ad.detect(frame);

        // publish robot pose
        std::vector<cv::Vec6d> poses = ad.get_current_poses();
        ros_basics_msgs::SimplePoseStamped pose; // ! we only use the first robot (no multi robot support for now)
        pose.header.stamp = ros::Time::now();
        pose.pose.xyz.x = poses[0][0];
        pose.pose.xyz.y = poses[0][1];
        pose.pose.xyz.z = poses[0][2];
        pose.pose.rpy.roll = poses[0][3];
        pose.pose.rpy.pitch = poses[0][4];
        pose.pose.rpy.yaw = poses[0][5];
        robot_pose_pub.publish(pose);

        // publish annotated image
        if (annotate_image) {
            ad.annotate_image(frame);
            sensor_msgs::ImagePtr annot_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            annot_image_pub.publish(annot_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}