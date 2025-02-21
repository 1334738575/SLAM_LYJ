#ifndef SLAM_LYJ_MATCHERABR_H
#define SLAM_LYJ_MATCHERABR_H


#include "base/PreDefine.h"
//#include "base/Frame.h"


NSP_SLAM_LYJ_BEGIN

class Frame;
class MatcherAbr
{
public:
	enum class TYPE
	{
		POINT=0,
		LINE,
		EDGE
	};
	MatcherAbr(TYPE _type) :type_(_type) {}
	~MatcherAbr() {}

	virtual int match(const Frame& _frame1, const Frame& _frame2, std::vector<int>& _match2to1) = 0;

	TYPE getType() { return type_; }
protected:
	TYPE  type_;
};



NSP_SLAM_LYJ_END


#endif //SLAM_LYJ_MATCHERABR_H
