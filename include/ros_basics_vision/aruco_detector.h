#ifndef ROS_TP_ARUCO_DETECTOR_H
#define ROS_TP_ARUCO_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <cassert>

namespace ros_tp {
    namespace defaults {
        struct ArucoDetectorParams {
            cv::Mat camera_matrix = (cv::Mat_<float>(3, 3) << 674.905111, 0.000000, 300.802978, 0.000000, 681.255848, 282.315032, 0.000000, 0.000000, 1.000000);
            cv::Mat distortion_coeffs = (cv::Mat_<float>(1, 5) << 0.066646, -0.058044, 0.007943, -0.004270, 0.000000);
            double marker_size_m = 0.075;
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
            _marker_size_m = params.marker_size_m;
        }

        void detect(const cv::Mat& img);
        void annotate_image(cv::Mat& img);

        cv::Vec3d rot2euler(cv::Mat& rot) const;

        void set_camera_matrix(const cv::Mat& matrix);
        void set_distortion_coeffs(const cv::Mat& matrix);
        void set_marker_size(const double marker_size_m);
        cv::Mat get_camera_matrix() const;
        cv::Mat get_distortion_coeffs() const;
        std::vector<cv::Vec3d> get_rotation_vecs() const;
        std::vector<cv::Vec3d> get_translation_vecs() const;

        const std::vector<int>& get_current_ids() const;
        const std::vector<std::vector<cv::Point2f>>& get_current_corners() const;
        const std::vector<std::vector<cv::Point2f>>& get_rejected_candidates() const;
        const std::vector<cv::Vec6d>& get_current_poses() const;
        const std::vector<cv::Point2f> get_pixel_positions() const;

    protected:
        double _angle_to_pipi(double angle);

        std::vector<int> _current_ids;
        std::vector<std::vector<cv::Point2f>> _current_corners;
        std::vector<std::vector<cv::Point2f>> _rejected_candidates;
        std::vector<cv::Vec6d> _current_poses;

        cv::Ptr<cv::aruco::DetectorParameters> _aruco_params;
        cv::Ptr<cv::aruco::Dictionary> _aruco_dict;

        cv::Mat _camera_matrix;
        cv::Mat _distortion_coeffs;
        std::vector<cv::Vec3d> _rotation_vecs;
        std::vector<cv::Vec3d> _translation_vecs;

        double _marker_size_m;
    };

} // namespace ros_tp

#endif
