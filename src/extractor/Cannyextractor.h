#ifndef SLAM_LYJ_CANNYEXTRACTOR_H
#define SLAM_LYJ_CANNYEXTRACTOR_H

#include "extractorAbr.h"
#include <base/Frame.h>

NSP_SLAM_LYJ_BEGIN

class CannyExtractor : public ExtractorAbr
{
public:
	struct Option
	{
		int lowTh = 50;
		int highTh = 140;
	};
	CannyExtractor(Option _opt);
	~CannyExtractor();

	// Í¨¹ý ExtractorAbr ¼Ì³Ð
	void extract(cv::Mat _img, Frame& _frame) override;

private:
	Option opt_;
};



NSP_SLAM_LYJ_END

#endif //SLAM_LYJ_CANNYEXTRACTOR_H