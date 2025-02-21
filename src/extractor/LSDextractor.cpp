#include "LSDextractor.h"

NSP_SLAM_LYJ_BEGIN


LSDExtractor::LSDExtractor(Option _opt) : ExtractorAbr(ExtractorAbr::TYPE::LSD), opt_(_opt)
{
    if(!_opt.needDesc)
        lsd_ = cv::createLineSegmentDetector();
    else {
        lsdKeyLine_ = cv::line_descriptor::LSDDetector::createLSDDetector();
        lineDescriptor_ = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
    }
}

LSDExtractor::~LSDExtractor()
{
}

void LSDExtractor::extract(cv::Mat _img, Frame& _frame)
{
    if(!opt_.needDesc)
        lsd_->detect(_img, _frame.getLines());
    else {
        lsdKeyLine_->detect(_img, _frame.getKeyLines(), 2, 2);
        lineDescriptor_->compute(_img, _frame.getKeyLines(), _frame.getLineDescriptors());
    }
}

void LSDExtractor::convertKeyLine2CVVec4f(const std::vector<cv::line_descriptor::KeyLine>& _keyLines, std::vector<cv::Vec4f>& _lines)
{
    _lines.resize(_keyLines.size());
    for (size_t i = 0; i < _keyLines.size(); ++i) {
        _lines[i](0) = _keyLines[i].startPointX;
        _lines[i](1) = _keyLines[i].startPointY;
        _lines[i](2) = _keyLines[i].endPointX;
        _lines[i](3) = _keyLines[i].endPointY;
    }
}

void LSDExtractor::convertCVVec4f2KeyLine(const std::vector<cv::Vec4f>& _lines, std::vector<cv::line_descriptor::KeyLine>& _keyLines)
{
    _keyLines.resize(_lines.size());
    float dx = 0;
    float dy = 0;
    for (size_t i = 0; i < _lines.size(); ++i) {
        _keyLines[i].startPointX = _lines[i](0);
        _keyLines[i].startPointY = _lines[i](1);
        _keyLines[i].endPointX = _lines[i](2);
        _keyLines[i].endPointY = _lines[i](3);
        _keyLines[i].class_id = static_cast<int>(i);
        dx = _lines[i](2) - _lines[i](0);
        dy = _lines[i](3) - _lines[i](1);
        _keyLines[i].angle = atan2(dy, dx);
    }
}


NSP_SLAM_LYJ_END