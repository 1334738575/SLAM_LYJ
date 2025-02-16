#include "LineMatcher.h"

NSP_SLAM_LYJ_BEGIN


LineMatcher::LineMatcher(Option _opt) : MatcherAbr(MatcherAbr::TYPE::LINE), opt_(_opt)
{
	if(_opt.mode = LDB)
		ldb_ = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
}

LineMatcher::~LineMatcher()
{
}

int LineMatcher::match(const Frame& _frame1, const Frame& _frame2, std::vector<int>& _match2to1)
{
	_match2to1.assign(_frame1.getKeyLines().size(), -1);
	if (opt_.mode == LDB) {
		std::vector<cv::DMatch> matches;
		ldb_->match(_frame1.getLineDescriptors(), _frame2.getLineDescriptors(), matches);
		for (size_t i = 0; i < matches.size(); ++i) {
			if (matches[i].distance < opt_.ldbTh) {
				_match2to1[matches[i].queryIdx] = matches[i].trainIdx;
			}
		}
	}
	else if (opt_.mode == FUNDAMENTAL) {
		Eigen::Matrix3d F = SLAM_LYJ_MATH::calculateFundamentalMatrix(
			_frame1.getTcw().getR(), _frame1.getTcw().gett(), _frame1.getCamera()->getK(),
			_frame2.getTcw().getR(), _frame2.getTcw().gett(), _frame2.getCamera()->getK()
		);
	}
	return 0;
}



NSP_SLAM_LYJ_END