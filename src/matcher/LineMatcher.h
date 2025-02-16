#ifndef SLAM_LYJ_LINEMATCHER_H
#define SLAM_LYJ_LINEMATCHER_H

#include "matcherAbr.h"
#include <base/Frame.h>

NSP_SLAM_LYJ_BEGIN


class LineMatcher : public MatcherAbr
{
public:
	enum MODE
	{
		LDB=0,
		FUNDAMENTAL
	};
	struct Option
	{
		MODE mode = LDB;
		float ldbTh = 25;
	};
	LineMatcher(Option _opt);
	~LineMatcher();


	// Í¨¹ý MatcherAbr ¼Ì³Ð
	int match(const Frame& _frame1, const Frame& _frame2, std::vector<int>& _match2to1) override;
private:
	Option opt_;
	cv::Ptr<cv::line_descriptor::BinaryDescriptorMatcher> ldb_;
};



NSP_SLAM_LYJ_END

#endif //SLAM_LYJ_GRIDMATCHER_H