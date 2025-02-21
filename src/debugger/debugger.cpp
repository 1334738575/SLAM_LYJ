#include "debugger.h"

NSP_SLAM_LYJ_DEBUGGER_BEGIN


void drawFeatures(const cv::Mat& _img, const std::vector<Eigen::Vector2f>& _kps, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _radius, const int _thick) {
    _imgOut = _img.clone();
    for (const auto& kp : _kps)
        cv::circle(_imgOut, cv::Point2f(kp.x(), kp.y()), _radius, _color, _thick);
}
void drawFeatures(const cv::Mat& _img, const std::vector<cv::KeyPoint>& _kps, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _radius, const int _thick
) {
    //cv::drawKeypoints(_img, _kps, _imgOut, cv::Scalar(0));
    _imgOut = _img.clone();
    for (const auto& kp : _kps)
        cv::circle(_imgOut, kp.pt, _radius, _color, _thick);
}
void drawLineFeatures(const cv::Mat& _img, const std::vector<cv::Vec4f>& _lines, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _thick)
{
    _imgOut = _img.clone();
    for (const auto& line : _lines)
        cv::line(_imgOut, cv::Point2f(line(0), line(1)), cv::Point2f(line(2), line(3)), _color, _thick);
}
void drawLineFeatures(const cv::Mat& _img, const std::vector<cv::Vec4f>& _lines,
    const cv::Ptr<cv::LineSegmentDetector>& _lsd, cv::Mat& _imgOut) {
    _imgOut = _img.clone();
    _lsd->drawSegments(_imgOut, _lines);
}
void drawEdgeFeatures(const cv::Mat& _img, const std::shared_ptr<Frame::EdgeFeatures> _edges, cv::Mat& _imgOut,
    const cv::Scalar _color) {
    if (_edges->cannyM.cols != 0 && _edges->cannyM.rows != 0) {
        _imgOut = _edges->cannyM.clone();
        return;
    }
    else if (!_edges->edges_.empty()) {
        drawFeatures(_img, _edges->edges_, _imgOut, _color, 0);
        return;
    }
}



NSP_SLAM_LYJ_DEBUGGER_END