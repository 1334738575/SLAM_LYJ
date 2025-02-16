#ifndef SLAM_LYJ_DEBUGGER_H
#define SLAM_LYJ_DEBUGGER_H


#include <base/PreDefine.h>
#include <opencv2/opencv.hpp>
#include <base/Frame.h>


NSP_SLAM_LYJ_DEBUGGER_BEGIN



//debug
void drawFeatures(const cv::Mat& _img, const std::vector<Eigen::Vector2f>& _kps, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _radius = 3, const int _thick = 1);
void drawFeatures(const cv::Mat& _img, const std::vector<cv::KeyPoint>& _kps, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _radius = 3, const int _thick = 1);
void drawLineFeatures(const cv::Mat& _img, const std::vector<cv::Vec4f>& _lines, cv::Mat& _imgOut,
    const cv::Scalar _color, const int _thick = 1);
void drawLineFeatures(const cv::Mat& _img, const std::vector<cv::Vec4f>& _lines,
    const cv::Ptr<cv::LineSegmentDetector>& _lsd, cv::Mat& _imgOut);
void drawEdgeFeatures(const cv::Mat& _img, const std::shared_ptr<Frame::EdgeFeatures> _edges, cv::Mat& _imgOut,
    const cv::Scalar _color);


NSP_SLAM_LYJ_DEBUGGER_END




#endif //SLAM_LYJ_DEBUGGER_H