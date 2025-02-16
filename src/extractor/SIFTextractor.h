#ifndef SLAM_LYJ_SIFTEXTRACTOR_H
#define SLAM_LYJ_SIFTEXTRACTOR_H

#include "extractorAbr.h"
#include <base/Frame.h>
#include <opencv2/xfeatures2d.hpp>

NSP_SLAM_LYJ_BEGIN

class SIFTExtractor : public ExtractorAbr
{
public:
	struct Option
	{

	};
	SIFTExtractor(Option _opt);
	~SIFTExtractor();

	// Í¨¹ý ExtractorAbr ¼Ì³Ð
	void extract(cv::Mat _img, Frame& _frame) override;

private:
	cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> sift_;
	Option opt_;
};


NSP_SLAM_LYJ_END

#endif //SLAM_LYJ_SIFTEXTRACTOR_H