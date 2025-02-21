#include "ORBextractor.h"

NSP_SLAM_LYJ_BEGIN

ORBExtractor::ORBExtractor(Option _opt)
	:ExtractorAbr(ExtractorAbr::TYPE::OBR), opt_(_opt)
{
	orb_ = cv::ORB::create();
}

ORBExtractor::~ORBExtractor()
{
}

void ORBExtractor::extract(cv::Mat _img, ExtractorOutput _output)
{
	ORBExtractOutput* output = (ORBExtractOutput*)_output;
	//std::vector<cv::KeyPoint> kps;
	//cv::Mat des;
	orb_->detectAndCompute(_img, cv::Mat(), *output->kps, *output->descriptors);
	//output->kps = std::move(kps);
	//output->descriptors = std::move(des);
}

void ORBExtractor::extract(cv::Mat _img, Frame& _frame)
{
	if (_img.type() != CV_8UC1)
		std::cout << "type error!" << std::endl;
	//std::vector<cv::KeyPoint> kps;
	//cv::Mat des;
	orb_->detectAndCompute(_img, cv::Mat(), _frame.getKeyPoints(), _frame.getDescriptors());
	//_frame.getKeyPoints() = std::move(kps);
	//_frame.getDescriptors() = std::move(des);
}


NSP_SLAM_LYJ_END