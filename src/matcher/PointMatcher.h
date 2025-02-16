#ifndef SLAM_LYJ_POINTMATCHER_H
#define SLAM_LYJ_POINTMATCHER_H

#include "matcherAbr.h"
#include <base/Frame.h>

NSP_SLAM_LYJ_BEGIN

class PointMatcher : public MatcherAbr
{
public:
	/*
	*	NULL				  = 0,
	    FLANNBASED            = 1,
        BRUTEFORCE            = 2,
        BRUTEFORCE_L1         = 3,
        BRUTEFORCE_HAMMING    = 4,
        BRUTEFORCE_HAMMINGLUT = 5,
        BRUTEFORCE_SL2        = 6,
		USER_DEFINE			  >=7
	*/
	struct Option
	{
		int mode = 1;
		//float distTh = 25;
	};
	PointMatcher(Option _opt);
	~PointMatcher();

	// Í¨¹ý MatcherAbr ¼Ì³Ð
	int match(const Frame& _frame1, const Frame& _frame2, std::vector<int>& _match2to1) override;
private:
	Option opt_;
	cv::Ptr<cv::DescriptorMatcher> descMatcher_;
};



//namespace colmap {
//#define IMAGE_ROWS 2048
//#define IMAGE_COLS 2048
//    //5%*255
//#define MATCH_TH 1632
//#define NN_TH 0.6
//    class featureGrid {
//    public:
//        featureGrid() = delete;
//        //resolution=20
//        featureGrid(const int resolution, std::shared_ptr<FeatureKeypoints> features);
//
//        void getKeypointIdsAround(const Eigen::Vector3d& line, std::vector<size_t>& ids);
//
//        inline int& getResolution() {
//            return resolution;
//        }
//    public:
//        int resolution;
//        int max_row;
//        int max_col;
//        std::unordered_map<int, std::vector<size_t>> grid;
//    };
//
//    class matcherByF {
//    public:
//        matcherByF() = delete;
//        matcherByF(const int resolution,
//            const Eigen::Matrix3d& K1,
//            const std::shared_ptr<FeatureKeypoints> features1,
//            const std::shared_ptr<FeatureDescriptors> desc1,
//            const cv::Mat& img1);
//        void MatchByF(const Eigen::Matrix3d& rota, const Eigen::Vector3d& trans,
//            const std::shared_ptr<FeatureKeypoints> features2,
//            const std::shared_ptr<FeatureDescriptors> desc2,
//            const cv::Mat& img2,
//            std::vector<std::pair<int, int>>& results);
//
//    private:
//        std::shared_ptr<FeatureKeypoints> features;
//        std::shared_ptr<FeatureDescriptors> desc;
//        featureGrid grid;
//        Eigen::Matrix3d K;
//        cv::Mat img;
//    };
//
//}


NSP_SLAM_LYJ_END

#endif //SLAM_LYJ_BFMATCHER_H