#ifndef SLAM_LYJ_EXTRACTORABR_H
#define SLAM_LYJ_EXTRACTORABR_H

#include "base/PreDefine.h"
#include <opencv2/opencv.hpp>

NSP_SLAM_LYJ_BEGIN

class Frame;
class ExtractorAbr
{
public:
	typedef void* ExtractorOutput;
	enum class TYPE
	{
		OBR = 0,
		SIFT,
		LSD,
		CANNY
	};
	ExtractorAbr(const TYPE _type) :type_(_type) {};
	~ExtractorAbr() {};

	//for visual feature，暂时弃用，图片过大可能堆损坏，需要进一步判断原因（普遍、旧机子、移动硬盘），Mat传输有点问题
	virtual void extract(cv::Mat _img, ExtractorOutput _output) {
		std::cout << "This have been discaeded!" << std::endl;
	}

	virtual void extract(cv::Mat _img, Frame& _frame) = 0;
	TYPE getType() { return type_; }
protected:
	TYPE  type_;
};


NSP_SLAM_LYJ_END

#endif //SLAM_LYJ_EXTRACTORABR_H