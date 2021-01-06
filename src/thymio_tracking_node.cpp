#include <ros_basics_vision/aruco_detector.h>

#include <ros_basics_msgs/SimplePoseStamped.h>
#include <ros_basics_msgs/GetWaypoints.h>
#include <ros_basics_msgs/AddWaypoint.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>

using namespace ros_tp;

class FrameInfo {
public:
    FrameInfo(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<ArucoDetector> ad, std::shared_ptr<image_transport::ImageTransport> it, double pix2mm, int width, int height) : _nh(nh), _ad(ad), _it(it), _pix2mm(pix2mm)
    {
        _annot_image_pub = _it->advertise("robot_view/image_annot", 1);
        _get_waypoints_cli = _nh->serviceClient<ros_basics_msgs::GetWaypoints>("get_waypoints");
        _prev_stamp = ros::Time::now();

        cv::Vec3d rvec(0., 0., 0.);
        cv::Vec3d tvec((width / 2.) * _pix2mm / 1000, (height / 2.) * _pix2mm / 1000, -0.68);
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        cv::Mat C = R.inv() * tvec;
        _ccenter = cv::Point(C.at<double>(0, 0) * 1000. / _pix2mm, C.at<double>(0, 1) * 1000. / _pix2mm);
    }

    void draw_info(cv::Mat& frame)
    {
        ros::Time now = ros::Time::now();
        double dt = ros::Duration(now - _prev_stamp).toSec();

        _ad->annotate_image(frame); // publish annotated image with aruco marker
        draw_heading(frame);
        draw_waypoints(frame);
        // draw_origin(frame);
        draw_camera_origin(frame);
        draw_axes(frame);
        draw_fps(frame, dt);

        _prev_stamp = now;
        publish(frame);
    }

    void draw_heading(cv::Mat& frame)
    {
        // heading on frame
        cv::Point2f centroid = _ad->get_pixel_positions()[0];
        cv::Vec6d pose = _ad->get_current_poses()[0];

        if (_ad->get_current_poses().size()) {
            std::stringstream yaw_stream;
            yaw_stream << std::fixed << std::setprecision(2) << pose[5] * 180. / M_PI;
            cv::putText(frame,
                "Hdg: " + yaw_stream.str(),
                cv::Point(centroid.x * 1.06, centroid.y * 1.1),
                cv::FONT_HERSHEY_DUPLEX,
                0.5,
                CV_RGB(180, 0, 0),
                2);

            std::stringstream pos_stream;
            pos_stream << std::fixed << std::setprecision(4) << centroid.x * _pix2mm / 1000. << ", " << centroid.y * _pix2mm / 1000.;
            cv::putText(frame,
                "Pos: (" + pos_stream.str() + ")",
                cv::Point(centroid.x * 1.06, centroid.y * 1.2),
                cv::FONT_HERSHEY_DUPLEX,
                0.5,
                CV_RGB(180, 0, 0),
                2);
        }
    }

    void draw_waypoints(cv::Mat& frame)
    {
        ros_basics_msgs::GetWaypoints srv;
        if (_get_waypoints_cli.call(srv)) {
            auto waypoints = srv.response.waypoints;
            for (size_t i = 0; i < waypoints.size(); ++i) {
                cv::Point wp((waypoints[i].x * 1000.) / _pix2mm + _ccenter.x, (waypoints[i].y * 1000.) / _pix2mm + _ccenter.y);
                cv::circle(frame, wp, 5, cv::Scalar(255, 255, 0), cv::FILLED, 10, 0);
                std::stringstream stream;
                stream << i;
                cv::putText(frame,
                    stream.str(),
                    wp + cv::Point(10, -10),
                    cv::FONT_HERSHEY_DUPLEX,
                    0.5,
                    cv::Scalar(255, 255, 0),
                    2);
            }
        }
        else {
            ROS_ERROR("Failed to call service get_waypoints");
        }
    }

    void draw_origin(cv::Mat& frame)
    {
        cv::putText(frame,
            "O(0, 0)",
            cv::Point(frame.size().width / 2 + 10, frame.size().height / 2 - 10),
            cv::FONT_HERSHEY_DUPLEX,
            0.5,
            CV_RGB(255, 0, 0),
            2);
        cv::circle(frame, cv::Point(frame.size().width / 2, frame.size().height / 2), 4, cv::Scalar(0, 0, 255), cv::FILLED, 10, 0);
    }

    void draw_camera_origin(cv::Mat& frame)
    {
        cv::circle(frame, _ccenter, 4, cv::Scalar(0, 0, 255), cv::FILLED, 10, 0);
        cv::putText(frame,
            "C(0, 0)",
            _ccenter + cv::Point(10, -10),
            cv::FONT_HERSHEY_DUPLEX,
            0.5,
            CV_RGB(255, 0, 0),
            2);
    }

    void draw_axes(cv::Mat& frame)
    {
        cv::Point center(frame.size().width * 0.03, frame.size().height * 0.95);
        cv::arrowedLine(frame, center, center - cv::Point(0, frame.size().height * 0.1), CV_RGB(255, 0, 0), 2, 8, 0, 0.1);
        cv::arrowedLine(frame, center, center + cv::Point(frame.size().height * 0.1, 0), CV_RGB(255, 0, 0), 2, 8, 0, 0.1);

        cv::putText(frame,
            "y",
            center - cv::Point(30, frame.size().height * 0.1) / 2.5,
            cv::FONT_HERSHEY_DUPLEX,
            0.5,
            CV_RGB(255, 0, 0),
            1);
        cv::putText(frame,
            "x",
            center + cv::Point(frame.size().height * 0.1, 30) / 2.5,
            cv::FONT_HERSHEY_DUPLEX,
            0.5,
            CV_RGB(255, 0, 0),
            1);
    }

    void draw_fps(cv::Mat& frame, double dt)
    {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(1) << 1 / dt;
        cv::putText(frame,
            "FPS: " + stream.str(),
            cv::Point(frame.size().width * 0.03, frame.size().height * 0.05),
            cv::FONT_HERSHEY_DUPLEX,
            0.5,
            CV_RGB(180, 0, 0),
            2);
    }

    void publish(cv::Mat& frame)
    {
        sensor_msgs::ImagePtr annot_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        _annot_image_pub.publish(annot_msg);
    }

protected:
    image_transport::Publisher _annot_image_pub;
    std::shared_ptr<ros::NodeHandle> _nh;
    std::shared_ptr<ArucoDetector> _ad;
    std::shared_ptr<image_transport::ImageTransport> _it;
    double _pix2mm;
    ros::ServiceClient _get_waypoints_cli;
    ros::Time _prev_stamp;
    ros::Duration _dt;
    cv::Point _ccenter;
};

struct MouseDetect {
    static void onMouse(int event, int x, int y, int flags, void* param) // now it's in param
    {
        if (event == cv::EVENT_LBUTTONDOWN) {
            cv::Point p(x - camera_px_width / 2, y - camera_px_height / 2);
            geometry_msgs::Pose2D wp;
            wp.x = (p.x * pix2mm) / 1000;
            wp.y = (p.y * pix2mm) / 1000;

            ros_basics_msgs::AddWaypoint srv;
            srv.request.goal = wp;
            if (!add_waypoint_cli.call(srv)) {
                ROS_ERROR("Failed to add waypoint");
            }
        }
    }

    static double pix2mm;
    static ros::ServiceClient add_waypoint_cli;
    static double camera_px_width;
    static double camera_px_height;
};
double MouseDetect::pix2mm;
ros::ServiceClient MouseDetect::add_waypoint_cli;
double MouseDetect::camera_px_width;
double MouseDetect::camera_px_height;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thymio_tracking_node");
    ros::NodeHandle nh;
    std::shared_ptr<image_transport::ImageTransport> it(new image_transport::ImageTransport(nh));

    int camera_dev;
    nh.param<int>("camera_dev", camera_dev, 6);
    cv::VideoCapture camera(camera_dev);
    ROS_INFO("Using camera device %d", camera_dev);
    if (!camera.isOpened()) {
        ROS_ERROR("ERROR: Could not open camera");
        return 1;
    }

    int camera_px_width, camera_px_height;
    double pix2mm;
    nh.param<int>("camera_px_width", camera_px_width, 640);
    nh.param<int>("camera_px_height", camera_px_height, 480);
    nh.param<double>("pix2mm", pix2mm, 1.002142315);

    camera.set(cv::CAP_PROP_FRAME_WIDTH, camera_px_width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, camera_px_height);

    image_transport::Publisher raw_image_pub = it->advertise("robot_view/image_raw", 1);
    bool annotate_image, use_mouse_waypoints;
    nh.param<bool>("annotate_image", annotate_image, true);
    nh.param<bool>("use_mouse_waypoints", use_mouse_waypoints, false);
    if (use_mouse_waypoints) {
        std::string name = "Tracking Robot (interactive)";
        cv::namedWindow(name);
        MouseDetect::pix2mm = pix2mm;
        MouseDetect::camera_px_width = camera_px_width;
        MouseDetect::camera_px_height = camera_px_height;
        MouseDetect::add_waypoint_cli = nh.serviceClient<ros_basics_msgs::AddWaypoint>("add_waypoint");
        cv::setMouseCallback(name, MouseDetect::onMouse);
    }

    ros::Publisher robot_pose_pub;
    robot_pose_pub = nh.advertise<ros_basics_msgs::SimplePoseStamped>("robot_pose", 1);

    cv::Mat frame;
    std::shared_ptr<ArucoDetector> ad(new ArucoDetector());
    FrameInfo fi(std::make_shared<ros::NodeHandle>(nh), ad, it, pix2mm, camera_px_width, camera_px_height);
    ros::Rate loop_rate(30);
    while (ros::ok()) {
        camera >> frame;

        // publish raw image
        sensor_msgs::ImagePtr raw_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        raw_image_pub.publish(raw_msg);

        // detect aruco markers
        ad->detect(frame);

        std::vector<cv::Vec6d> poses = ad->get_current_poses();
        if (poses.size()) {
            // publish robot pose if a robot was detected
            ros_basics_msgs::SimplePoseStamped pose; // ! we only use the first robot (no multi robot support for now)
            pose.header.stamp = ros::Time::now();
            pose.pose.xyz.x = poses[0][0];
            pose.pose.xyz.y = poses[0][1];
            pose.pose.xyz.z = poses[0][2];
            pose.pose.rpy.roll = poses[0][3];
            pose.pose.rpy.pitch = poses[0][4];
            pose.pose.rpy.yaw = poses[0][5];
            robot_pose_pub.publish(pose);

            // draw info on the frame
            if (annotate_image) {
                fi.draw_info(frame);
                if (use_mouse_waypoints) {
                    cv::imshow("Tracking Robot (interactive)", frame);
                    cv::waitKey(10);
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}