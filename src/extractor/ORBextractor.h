#ifndef SLAM_LYJ_ORBEXTRACTOR_H
#define SLAM_LYJ_ORBEXTRACTOR_H

#include "extractorAbr.h"
#include "base/Frame.h"


NSP_SLAM_LYJ_BEGIN

class ORBExtractor : public ExtractorAbr
{
public:
	struct Option
	{

	};
	ORBExtractor(Option _opt);
	~ORBExtractor();
	struct ORBExtractOutput
	{
		ORBExtractOutput(std::vector<cv::KeyPoint>* _kps, cv::Mat* _descriptors) {
			kps = _kps;
			descriptors = _descriptors;
		}
		std::vector<cv::KeyPoint>* kps = nullptr;
		cv::Mat* descriptors = nullptr;
	};

	// Í¨¹ý ExtractorAbr ¼Ì³Ð
	void extract(cv::Mat _img, ExtractorOutput _output) override;

	void extract(cv::Mat _img, Frame& _frame) override;
private:
	cv::Ptr<cv::ORB> orb_;
	Option opt_;
};



NSP_SLAM_LYJ_END


#endif //SLAM_LYJ_ORBEXTRACTOR_H