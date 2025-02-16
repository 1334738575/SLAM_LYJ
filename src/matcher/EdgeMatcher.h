#ifndef SLAM_LYJ_EDGEMATCHER_H
#define SLAM_LYJ_EDGEMATCHER_H

#include "matcherAbr.h"
#include <base/Frame.h>

NSP_SLAM_LYJ_BEGIN

class EdgeMatcher : public MatcherAbr
{
public:
	struct Option
	{
		float dirTh = 0.9f; //around -20degree, +20degree
		int nearN = 3;
		float distTh = 10;
	};
	EdgeMatcher(Option _opt);
	~EdgeMatcher();



	// Í¨¹ý MatcherAbr ¼Ì³Ð
	int match(const Frame& _frame1, const Frame& _frame2, std::vector<int>& _match2to1) override;

private:
	Option opt_;
};

EdgeMatcher::EdgeMatcher(Option _opt): MatcherAbr(MatcherAbr::TYPE::EDGE), opt_(_opt)
{
}

EdgeMatcher::~EdgeMatcher()
{
}

int EdgeMatcher::match(const Frame& _frame1, const Frame& _frame2, std::vector<int>& _match2to1)
{
	const auto features1 = _frame1.getEdgeFeatures();
	const auto features2 = _frame2.getEdgeFeatures();
	int fSize1 = (int)features1->edges_.size();
	int fSize2 = (int)features2->edges_.size();
	_match2to1.assign(fSize1, -1);
	std::vector<std::pair<Eigen::Vector2f, int>> nearPs;
	std::vector<float> dists;
	int nearSize = -1;
	//auto funcDist = [&features1](const SLAM_LYJ_MATH::KdTree<float, 2>::Node* _node, const Eigen::Vector2f& _p)->float {
	//};
	for (int i = 0; i < fSize2; ++i) {
		nearSize = features1->kdtree_->search2(features2->edges_[i], opt_.nearN, opt_.distTh, nearPs, dists);
		if (nearSize <= 0)
			continue;
		for (int j = 0; j < nearSize; ++j) {
			if(features1->edgesDir_[nearPs[j].second].dot(features2->edgesDir_[i]) < opt_.dirTh)
				_match2to1[nearPs[i].second] = i;
		}
	}
	return 0;
}


NSP_SLAM_LYJ_END

#endif //SLAM_LYJ_KDTREEMATCHER_H