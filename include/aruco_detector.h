#ifndef ROS_TP_ARUCO_DETECTOR_H
#define ROS_TP_ARUCO_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <cassert>

namespace ros_tp {
    namespace defaults {
        struct ArucoDetectorParams {
            cv::Mat camera_matrix = (cv::Mat_<float>(3, 3) << 1.57415145e+03, 0.00000000e+00, 9.60062009e+02,
                0.00000000e+00, 1.57042499e+03, 5.41781766e+02, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00);
            cv::Mat distortion_coeffs = (cv::Mat_<float>(1, 5) << 1.47066028e-02, -7.89474928e-01, -1.53707885e-03, -6.92785954e-03, 2.46659765e+00);
        };
    } // namespace defaults

    class ArucoDetector {
    public:
        ArucoDetector();

        template <typename Params>
        ArucoDetector(Params params) : _aruco_params(cv::aruco::DetectorParameters::create()),
                                       _aruco_dict(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000))
        {
            _camera_matrix = params.camera_matrix;
            _distortion_coeffs = params.distortion_coeffs;
        }

        void detect(const cv::Mat& img);
        void annotate_image(cv::Mat& img);

        void set_camera_matrix(const cv::Mat& matrix);
        void set_distortion_coeffs(const cv::Mat& matrix);
        cv::Mat get_camera_matrix() const;
        cv::Mat get_distortion_coeffs() const;

    protected:
        std::vector<int> _current_ids;
        std::vector<std::vector<cv::Point2f>> _current_corners;
        std::vector<std::vector<cv::Point2f>> _rejected_candidates;

        cv::Ptr<cv::aruco::DetectorParameters> _aruco_params;
        cv::Ptr<cv::aruco::Dictionary> _aruco_dict;

        cv::Mat _camera_matrix;
        cv::Mat _distortion_coeffs;
        std::vector<cv::Vec3d> _rotation_vecs;
        std::vector<cv::Vec3d> _translation_vecs;
    };

} // namespace ros_tp

#endif
