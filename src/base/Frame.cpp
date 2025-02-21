#include "Frame.h"
#include "extractor/ORBextractor.h"

NSP_SLAM_LYJ_BEGIN

//Frame::Frame(const uint64_t _id, const cv::Mat& _img, ExtractorAbr* _extractor, CameraModule* _cam,
//	const KeyPointIndexMode _kpIndMode)
//	:id_(_id), cam_(_cam), kpIndMode_(_kpIndMode)
//{
//#ifdef SYS_DEBUG
//	img_ = _img;
//#endif // SYS_DEBUG
//	extractFeature(_img, _extractor);
//}
//Frame::Frame(const uint64_t _id, const std::string& _imgPath, ExtractorAbr* _extractor, CameraModule* _cam,
//	const KeyPointIndexMode _kpIndMode)
//	:id_(_id), cam_(_cam), kpIndMode_(_kpIndMode)
//{
//	imgPath_ = _imgPath;
//	cv::Mat img = cv::imread(_imgPath);
//	extractFeature(img, _extractor);
//}

Frame::Frame(const uint64_t _id, CameraModule* _cam, const KeyPointIndexMode _kpIndMode)
	:id_(_id), cam_(_cam), kpIndMode_(_kpIndMode)
{
	edgeFeatures_ = std::make_shared<EdgeFeatures>();
}

Frame::~Frame()
{
}

//void Frame::extractFeature(const cv::Mat& _img, ExtractorAbr* _extractor)
//{
//	ORBExtractor::ORBExtractOutput features(&kps_, &descriptors_);
//	_extractor->extract(_img, (void*)(&features));
//	//kps_ = std::move(features.kps);
//	//descriptors_ = std::move(features.descriptors);
//}

std::vector<int> Frame::getKpIndsNear(const cv::KeyPoint& _kp) const
{
	if(!grid_)
		return std::vector<int>();
	int resolution = grid_->getResolution();
	std::list<int> inds = grid_->getIndsNear(Eigen::Vector2f(_kp.pt.x, _kp.pt.y));
	return std::vector<int>(inds.begin(), inds.end());
}

void Frame::write_binary(std::ofstream &os)
{
}

void Frame::read_binary(std::ifstream &os)
{
}

NSP_SLAM_LYJ_END