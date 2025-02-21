#include "PointMatcher.h"

NSP_SLAM_LYJ_BEGIN


PointMatcher::PointMatcher(Option _opt) : MatcherAbr(MatcherAbr::TYPE::POINT), opt_(_opt)
{
	if (_opt.mode <= 0) {

	}
	else if (_opt.mode <= 6) {
		descMatcher_ = cv::DescriptorMatcher::create(_opt.mode);
	}
	else {

	}
}

PointMatcher::~PointMatcher()
{
}

int PointMatcher::match(const Frame& _frame1, const Frame& _frame2, std::vector<int>& _match2to1)
{
	_match2to1.assign(_frame1.getKeyPoints().size(), -1);
	if (opt_.mode <= 0) {
		return 0;
	}
	else if (opt_.mode <= 6) {
		std::vector<cv::DMatch> matches;
		descMatcher_->match(_frame1.getDescriptors(), _frame2.getDescriptors(), matches);
		for (size_t i = 0; i < matches.size(); ++i) {
			_match2to1[matches[i].queryIdx] = matches[i].trainIdx;
		}
	}
	//fundamental
	else if(opt_.mode == 7){

	}
	else {

	}
	return 0;
}


//namespace colmap {
//
//    int imgi = 0;
//
//    featureGrid::featureGrid(const int resolution, std::shared_ptr<FeatureKeypoints> features) {
//        this->resolution = resolution;
//        max_row = IMAGE_ROWS / resolution;
//        max_col = IMAGE_COLS / resolution;
//        for (size_t i = 0; i < features->size(); ++i) {
//            int x = (*features)[i].x / resolution;
//            int y = (*features)[i].y / resolution;
//            grid[y * max_col + x].push_back(i);
//        }
//    }
//
//    void featureGrid::getKeypointIdsAround(const Eigen::Vector3d& line, std::vector<size_t>& ids) {
//        bool xtrend = true;
//        Eigen::Vector3d line_resol = line;
//        line_resol(2) /= resolution;
//        if (abs(line_resol(0)) > abs(line_resol(1)))
//            xtrend = false;
//        if (xtrend) {
//            double k = -1 * (line_resol(0)) / line_resol(1);
//            double b = -1 * (line_resol(2)) / line_resol(1);
//            for (int indx = 0; indx < max_col; ++indx) {
//                int indy = k * indx + b;
//                if (indy < 0) {
//                    if (k > 0) continue;
//                    else break;
//                }
//                else if (indy > max_row) {
//                    if (k < 0) continue;
//                    else break;
//                }
//                int starty = (indy - 1) > 0 ? indy - 1 : 0;
//                int endy = (indy + 1) < max_row ? indy + 1 : max_row;
//                for (int iy = starty; iy <= endy; ++iy) {
//                    // std::cout<<"get ids in grid("<<indx<<","<<iy<<")"<<std::endl;
//                    if (grid.count(iy * max_col + indx)) {
//                        std::vector<size_t>& idsInGrid = grid[iy * max_col + indx];
//                        ids.insert(ids.end(), idsInGrid.begin(), idsInGrid.end());
//                    }
//                }
//            }
//        }
//        else {
//            double k = -1 * (line_resol(1)) / line_resol(0);
//            double b = -1 * (line_resol(2)) / line_resol(0);
//            for (int indy = 0; indy < max_row; ++indy) {
//                int indx = k * indy + b;
//                if (indx < 0) {
//                    if (k > 0) continue;
//                    else break;
//                }
//                else if (indx > max_col) {
//                    if (k < 0) continue;
//                    else break;
//                }
//                int startx = (indx - 1) > 0 ? indx - 1 : 0;
//                int endx = (indx + 1) < max_col ? indx + 1 : max_col;
//                for (int ix = startx; ix <= endx; ++ix) {
//                    if (grid.count(indy * max_col + ix)) {
//                        // std::cout<<"get ids in grid("<<ix<<","<<indy<<")"<<std::endl;
//                        std::vector<size_t>& idsInGrid = grid[indy * max_col + ix];
//                        ids.insert(ids.end(), idsInGrid.begin(), idsInGrid.end());
//                    }
//                }
//            }
//        }
//    }
//
//
//    matcherByF::matcherByF(const int resolution,
//        const Eigen::Matrix3d& K1,
//        const std::shared_ptr<FeatureKeypoints> features1,
//        const std::shared_ptr<FeatureDescriptors> desc1,
//        const cv::Mat& img1) :
//        K(K1), features(features1), desc(desc1), img(img1), grid(resolution, features1)
//    { }
//    void matcherByF::MatchByF(const Eigen::Matrix3d& rota, const Eigen::Vector3d& trans,
//        const std::shared_ptr<FeatureKeypoints> features2,
//        const std::shared_ptr<FeatureDescriptors> desc2,
//        const cv::Mat& img2,
//        std::vector<std::pair<int, int>>& results) {
//        std::vector<int> matches(features->size(), -1);
//        std::vector<int> matchesDist(features->size(), -1);
//
//        //debug
//        std::vector<Eigen::Vector4d> linePoints(features2->size());
//        int downTimes = 4;
//        cv::Mat mmatch(img.rows / downTimes, (img.cols + img2.cols) / downTimes, CV_8UC3);
//        cv::Rect rect1(0, 0, img.cols / downTimes, img.rows / downTimes);
//        cv::Rect rect2(img2.cols / downTimes, 0, img.cols / downTimes, img.rows / downTimes);
//        cv::Mat imgDown, imgDown2;
//        cv::pyrDown(img, imgDown);
//        cv::pyrDown(imgDown, imgDown);
//        cv::pyrDown(img2, imgDown2);
//        cv::pyrDown(imgDown2, imgDown2);
//        imgDown.copyTo(mmatch(rect1));
//        imgDown2.copyTo(mmatch(rect2));
//
//        //compute F matrix
//        Eigen::Matrix3d t_mul;
//        t_mul << 0, -1 * trans(2), trans(1), \
//            trans(2), 0, -1 * trans(0), \
//            - 1 * trans(1), trans(0), 0;
//        Eigen::Matrix3d F = K.inverse().transpose() * t_mul * rota * K.inverse();
//
//        for (size_t i = 0; i < features2->size(); ++i) {
//            Eigen::Vector3d p2((*features2)[i].x, (*features2)[i].y, 1);
//            const FeatureDescriptor& des2 = desc2->row(i);
//
//            //compute line
//            Eigen::Vector3d line = F * p2;
//
//            //get ids
//            std::vector<size_t> ids;
//            grid.getKeypointIdsAround(line, ids);
//            if (ids.empty()) {
//                // std::cout<<"not find ids around line, point("<<p2(0)<<","<<p2(1)<<")"<<std::endl;
//                continue;
//            }
//
//            // //debug
//            // cv::Mat matLine = mmatch.clone();
//            // // // std::cout<<"ids around line, point("<<p2(0)<<","<<p2(1)<<")"<<std::endl;
//
//            //match
//            int best_id = -1;
//            int best_dist = 128 * 255;
//            int second_id = -1;
//            int second_dist = 128 * 255;
//            for (size_t j = 0; j < ids.size(); ++j) {
//
//                // // //debug
//                // cv::Point2d p1(features->at(ids[j]).x/downTimes, features->at(ids[j]).y/downTimes);
//                // cv::circle(matLine, p1, 1, cv::Scalar(255,0,0));
//
//                //compute distance
//                int dist = 0;
//                const FeatureDescriptor& des1 = desc->row(ids[j]);
//                for (size_t di = 0; di < desc->cols(); ++di) {
//                    if (des1(di) > des2(di))
//                        dist = dist + int(des1(di) - des2(di));
//                    else
//                        dist = dist + int(des2(di) - des1(di));
//                    // std::cout<<"to:"<<di<<", dist:"<<dist<<std::endl;
//                }
//
//                //judge
//                if (dist < best_dist) {
//                    second_id = (int)best_id;
//                    second_dist = best_dist;
//                    best_id = (int)ids[j];
//                    best_dist = dist;
//                }
//                else if (dist < second_dist) {
//                    second_id = (int)ids[j];
//                    second_dist = dist;
//                }
//            }
//
//            // //debug
//            // // std::cout<<"best dist:"<<best_dist<<" second dist:"<<second_dist<<std::endl;
//            // cv::line(matLine, cv::Point(linePoints[i](0)/downTimes,linePoints[i](1)/downTimes), 
//            //                     cv::Point(linePoints[i](2)/downTimes,linePoints[i](3)/downTimes),
//            //                     cv::Scalar(0,255,255),1);
//            // cv::Point2d p((features2->at(i).x+img2.cols)/downTimes, features2->at(i).y/downTimes);
//            // cv::circle(matLine, p, 3, cv::Scalar(255,0,255));
//            // // cv::imshow("F line", matLine);
//            // // cv::waitKey();
//
//            //record
//            if (best_dist > MATCH_TH || second_dist * NN_TH < best_dist)
//                continue;
//            if ((matches[best_id] != -1) && (matchesDist[best_id] < best_dist)) {
//                continue;
//            }
//            matches[best_id] = i;
//            matchesDist[best_id] = best_dist;
//
//            // cv::Point2d pbest(features->at(best_id).x/downTimes, features->at(best_id).y/downTimes);
//            // cv::circle(matLine, pbest, 3, cv::Scalar(255,0,255));
//            // // cv::imshow("F line", matLine);
//            // // cv::waitKey();
//
//        }
//
//        for (size_t ind = 0; ind < matches.size(); ++ind) {
//            if (matches[ind] == -1)
//                continue;
//            results.push_back({ ind, matches[ind] });
//        }
//
//        //debug
//        // for(size_t ind=0;ind<features2->size();++ind){
//        //     // std::cout<<"draw line:"<<ind<<std::endl;
//        //     cv::Mat matLine = mmatch.clone();
//        //     cv::line(matLine, cv::Point(linePoints[ind](0)/downTimes,linePoints[ind](1)/downTimes), 
//        //                     cv::Point(linePoints[ind](2)/downTimes,linePoints[ind](3)/downTimes),
//        //                     cv::Scalar(0,255,255),1);
//        //     cv::Point2d p((features2->at(ind).x+img2.cols)/downTimes, features2->at(ind).y/downTimes);
//        //     cv::circle(matLine, p, 3, cv::Scalar(255,0,255));
//        //     cv::imshow("F line", matLine);
//        //     cv::waitKey();
//        // }
//        for (size_t ind = 0; ind < matches.size(); ++ind) {
//            if (matches[ind] == -1)
//                continue;
//            cv::Point2d p1(features->at(ind).x / downTimes, features->at(ind).y / downTimes);
//            cv::Point2d p2((features2->at(matches[ind]).x + img2.cols) / downTimes, features2->at(matches[ind]).y / downTimes);
//            cv::circle(mmatch, p1, 3, cv::Scalar(255, 0, 255));
//            cv::circle(mmatch, p2, 3, cv::Scalar(255, 0, 255));
//            cv::line(mmatch, p1, p2, cv::Scalar(0, 255, 255), 1);
//        }
//        // cv::namedWindow("test match", cv::WINDOW_NORMAL);
//        // cv::imshow("test match", mmatch);
//        // cv::waitKey(1);
//        std::string savepath = "/home/yang/imgdebug/match/" + std::to_string(imgi++) + ".png";
//        cv::imwrite(savepath, mmatch);
//    }
//
//}  // namespace colmap



NSP_SLAM_LYJ_END