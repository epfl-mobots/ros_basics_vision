#include <aruco_detector.h>

namespace ros_tp {

    ArucoDetector::ArucoDetector() : ArucoDetector(defaults::ArucoDetectorParams())
    {
    }

    void ArucoDetector::detect(const cv::Mat& img)
    {
        _current_ids.clear();
        _current_corners.clear();
        _rejected_candidates.clear();
        cv::aruco::detectMarkers(img, _aruco_dict, _current_corners, _current_ids, _aruco_params, _rejected_candidates);
    }

    void ArucoDetector::annotate_image(cv::Mat& img)
    {
        cv::aruco::drawDetectedMarkers(img, _current_corners, _current_ids, cv::Scalar(0, 0, 255));
        cv::aruco::estimatePoseSingleMarkers(_current_corners, 0.05, _camera_matrix, _distortion_coeffs, _rotation_vecs, _translation_vecs);
        for (int i = 0; i < _rotation_vecs.size(); ++i) {
            auto rvec = _rotation_vecs[i];
            auto tvec = _translation_vecs[i];
            cv::aruco::drawAxis(img, _camera_matrix, _distortion_coeffs, rvec, tvec, 0.1);
        }
    }

    void ArucoDetector::set_camera_matrix(const cv::Mat& matrix)
    {
        _camera_matrix = matrix;
    }

    void ArucoDetector::set_distortion_coeffs(const cv::Mat& matrix)
    {
        _distortion_coeffs = matrix;
    }

    cv::Mat ArucoDetector::get_camera_matrix() const
    {
        return _camera_matrix;
    }

    cv::Mat ArucoDetector::get_distortion_coeffs() const
    {
        return _distortion_coeffs;
    }

} // namespace ros_tp