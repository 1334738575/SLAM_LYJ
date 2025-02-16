#include "Cannyextractor.h"


NSP_SLAM_LYJ_BEGIN


CannyExtractor::CannyExtractor(Option _opt) :ExtractorAbr(ExtractorAbr::TYPE::CANNY), opt_(_opt)
{
}

CannyExtractor::~CannyExtractor()
{
}

void CannyExtractor::extract(cv::Mat _img, Frame& _frame)
{
	std::shared_ptr<Frame::EdgeFeatures> edgeFeatures = _frame.getEdgeFeatures();
	cv::Sobel(_img, edgeFeatures->dxM, CV_16SC1, 1, 0);
	cv::Sobel(_img, edgeFeatures->dxM, CV_16SC1, 0, 1);
	cv::Canny(edgeFeatures->dxM, edgeFeatures->dyM, edgeFeatures->cannyM, opt_.lowTh, opt_.highTh);
	return; //后续步骤建议结合3D信息的情况下做
	int preSize = _img.rows * _img.cols / 4;
	edgeFeatures->edgesDir_.reserve(preSize);
	edgeFeatures->edges_.reserve(preSize);
	edgeFeatures->isParallel_ = false;
	edgeFeatures->nParallelVar_.setZero();
	edgeFeatures->nParallel_.setZero();
	Eigen::Vector2f dirTmp;
	Eigen::Vector2f uvTmp;
	int cnt = 0;
	for (int i = 0; i < _img.rows; ++i) {
		for (int j = 0; j < _img.cols; ++j) {
			auto v = ((int16_t*)edgeFeatures->cannyM.data)[i * _img.cols + j];
			if (v <= 0)
				continue;
			uvTmp(0) = static_cast<float>(j);
			uvTmp(1) = static_cast<float>(i);
			dirTmp(0) = static_cast<float>(((int16_t*)edgeFeatures->dxM.data)[i * _img.cols + j]);
			dirTmp(1) = static_cast<float>(((int16_t*)edgeFeatures->dyM.data)[i * _img.cols + j]);
			edgeFeatures->edges_.push_back(uvTmp);
			edgeFeatures->edgesDir_.push_back(dirTmp);
		}
	}
}

NSP_SLAM_LYJ_END