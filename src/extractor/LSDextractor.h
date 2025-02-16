#ifndef SLAM_LYJ_LSDEXTRACTOR_H
#define SLAM_LYJ_LSDEXTRACTOR_H

#include "extractorAbr.h"
#include <base/Frame.h>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/line_descriptor.hpp>

NSP_SLAM_LYJ_BEGIN


class LSDExtractor : public ExtractorAbr
{
public:
	struct Option
	{
		bool needDesc = false;
	};
	LSDExtractor(Option _opt);
	~LSDExtractor();


	// Í¨¹ý ExtractorAbr ¼Ì³Ð
	void extract(cv::Mat _img, Frame& _frame) override;

	static void convertKeyLine2CVVec4f(const std::vector<cv::line_descriptor::KeyLine>& _keyLines, std::vector<cv::Vec4f>& _lines);
	static void convertCVVec4f2KeyLine(const std::vector<cv::Vec4f>& _lines, std::vector<cv::line_descriptor::KeyLine>& _keyLines);


private:
	cv::Ptr<cv::LineSegmentDetector> lsd_;
	cv::Ptr<cv::line_descriptor::LSDDetector> lsdKeyLine_;
	cv::Ptr<cv::line_descriptor::BinaryDescriptor> lineDescriptor_;
	Option opt_;
};


NSP_SLAM_LYJ_END

#endif //SLAM_LYJ_LSDEXTRACTOR_H