#include "SIFTextractor.h"


NSP_SLAM_LYJ_BEGIN


SIFTExtractor::SIFTExtractor(Option _opt) : ExtractorAbr(ExtractorAbr::TYPE::SIFT), opt_(_opt)
{
    sift_ = cv::xfeatures2d::SiftFeatureDetector::create();
}

SIFTExtractor::~SIFTExtractor()
{
}

void SIFTExtractor::extract(cv::Mat _img, Frame& _frame)
{
    sift_->detectAndCompute(_img, cv::Mat(), _frame.getKeyPoints(), _frame.getDescriptors());
}

NSP_SLAM_LYJ_END