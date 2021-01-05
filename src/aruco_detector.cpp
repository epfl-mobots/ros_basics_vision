#include <ros_basics_vision/aruco_detector.h>

namespace ros_tp {

    ArucoDetector::ArucoDetector() : ArucoDetector(defaults::ArucoDetectorParams())
    {
    }

    void ArucoDetector::detect(const cv::Mat& img)
    {
        _current_ids.clear();
        _current_corners.clear();
        _rejected_candidates.clear();
        _current_poses.clear();

        cv::aruco::detectMarkers(img, _aruco_dict, _current_corners, _current_ids, _aruco_params, _rejected_candidates);
        cv::aruco::estimatePoseSingleMarkers(_current_corners, 0.05, _camera_matrix, _distortion_coeffs, _rotation_vecs, _translation_vecs);

        for (int i = 0; i < _rotation_vecs.size(); ++i) {
            auto rvec = _rotation_vecs[i];
            auto tvec = _translation_vecs[i];
            cv::Mat rot, rodrigues;
            cv::Rodrigues(rvec, rot);
            cv::Vec3d rpy = rot2euler(rot);
            _current_poses.push_back(cv::Vec6d(tvec[0], tvec[1], tvec[2], rpy[0], rpy[1], rpy[2]));
        }
    }

    void ArucoDetector::annotate_image(cv::Mat& img)
    {
        cv::aruco::drawDetectedMarkers(img, _current_corners, _current_ids, cv::Scalar(0, 0, 255));
        for (int i = 0; i < _rotation_vecs.size(); ++i) {
            auto rvec = _rotation_vecs[i];
            auto tvec = _translation_vecs[i];
            cv::aruco::drawAxis(img, _camera_matrix, _distortion_coeffs, rvec, tvec, 0.1);
        }
    }

    cv::Vec3d ArucoDetector::rot2euler(cv::Mat& rot) const
    {
        double sy = sqrt(rot.at<double>(0, 0) * rot.at<double>(0, 0) + rot.at<double>(1, 0) * rot.at<double>(1, 0));
        double x, y, z;
        if (sy >= 1e-6) {
            x = atan2(rot.at<double>(2, 1), rot.at<double>(2, 2));
            y = atan2(-rot.at<double>(2, 0), sy);
            z = atan2(rot.at<double>(1, 0), rot.at<double>(0, 0));
        }
        else {
            x = atan2(-rot.at<double>(1, 2), rot.at<double>(1, 1));
            y = atan2(-rot.at<double>(2, 0), sy);
            z = 0;
        }
        return cv::Vec3d(x, y, z);
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

    const std::vector<int>& ArucoDetector::get_current_ids() const
    {
        return _current_ids;
    }

    const std::vector<std::vector<cv::Point2f>>& ArucoDetector::get_current_corners() const
    {
        return _current_corners;
    }

    const std::vector<std::vector<cv::Point2f>>& ArucoDetector::get_rejected_candidates() const
    {
        return _rejected_candidates;
    }

    const std::vector<cv::Vec6d>& ArucoDetector::get_current_poses() const
    {
        return _current_poses;
    }

} // namespace ros_tp