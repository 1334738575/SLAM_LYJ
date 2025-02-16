#ifndef SLAM_LYJ_FRAME_H
#define SLAM_LYJ_FRAME_H

#include "PreDefine.h"
#include "extractor/extractorAbr.h"
#include "base/CameraModule.h"
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/line_descriptor.hpp>
#include <opencv2/highgui.hpp>

NSP_SLAM_LYJ_BEGIN


class Frame : public BaseLYJ
{
public:
    enum KeyPointIndexMode
    {
        BRUTEFORCE = 0,
        GRID,
        KDTREE
    };
    struct EdgeFeatures
    {
        std::vector<Eigen::Vector2f> edges_;
        std::shared_ptr<SLAM_LYJ_MATH::KdTree<float, 2>> kdtree_ = nullptr;
        std::vector<Eigen::Vector2f> edgesDir_;
        bool isParallel_ = false;
        Eigen::Vector3f nParallel_ = Eigen::Vector3f::Zero();
        Eigen::Vector3f nParallelVar_ = Eigen::Vector3f::Zero();
        cv::Mat dxM;
        cv::Mat dyM;
        cv::Mat cannyM;
    };
    Frame() = delete;
    //Frame(const uint64_t _id, const cv::Mat& _img, ExtractorAbr* _extractor, CameraModule* _cam,
    //    const KeyPointIndexMode _kpIndMode = GRID);
    //Frame(const uint64_t _id, const std::string& _imgPath, ExtractorAbr* _extractor, CameraModule* _cam,
    //    const KeyPointIndexMode _kpIndMode = GRID);
    Frame(const uint64_t _id, CameraModule* _cam,
        const KeyPointIndexMode _kpIndMode = GRID);
    ~Frame();

    inline const uint64_t getId() const { return id_;}
    inline const bool isKeyFrame() const { return bKey_;}
    inline void setKeyFrame (bool _bKey) { bKey_ = _bKey; }
    inline CameraModule* getCamera() { return cam_; }
    inline const CameraModule* getCamera() const { return cam_; }
    inline Pose3D& getTcw() { return Tcw_; }
    inline const Pose3D& getTcw() const { return Tcw_; }

    inline std::vector<cv::KeyPoint>& getKeyPoints() { return kps_; }
    inline const std::vector<cv::KeyPoint>& getKeyPoints() const { return kps_; }
    inline const cv::KeyPoint& getKeyPoint(const int _i) const { return kps_[_i]; }
    inline cv::Mat& getDescriptors() { return  descriptors_; }
    inline const cv::Mat& getDescriptors() const { return  descriptors_; }
    inline const cv::Mat& getDescriptor(const int _i) const { return descriptors_.row(_i); }
    inline std::shared_ptr<SLAM_LYJ_MATH::KdTree2d> getKpsKdtree() { return kdtree_; }
    inline std::shared_ptr<SLAM_LYJ_MATH::Grid2Df> getKpsGrid() { return grid_; }
    std::vector<int> getKpIndsNear(const cv::KeyPoint& _kp) const;

    inline const cv::Vec4f& getLine(const int _i) { return vecLines_[_i]; }
    inline std::vector<cv::Vec4f>& getLines() { return vecLines_; }
    inline const std::vector<cv::Vec4f>& getLines() const { return vecLines_; }
    inline const cv::line_descriptor::KeyLine& getKeyLines(const int _i) { return vecKeyLines_[_i]; }
    inline std::vector<cv::line_descriptor::KeyLine>& getKeyLines() { return vecKeyLines_; }
    inline const std::vector<cv::line_descriptor::KeyLine>& getKeyLines() const { return vecKeyLines_; }
    inline const cv::Mat& getLineDescriptor(const int _i) { return lineDescriptors_.row(_i); }
    inline cv::Mat& getLineDescriptors() { return lineDescriptors_; }
    inline const cv::Mat& getLineDescriptors() const { return lineDescriptors_; }

    inline std::shared_ptr<EdgeFeatures> getEdgeFeatures() { return edgeFeatures_; }
    inline const std::shared_ptr<EdgeFeatures> getEdgeFeatures() const { return edgeFeatures_; }

    //void extractFeature(const cv::Mat& _img, ExtractorAbr* _extractor);

    //override
    void write_binary(std::ofstream& os);
    //override
    void read_binary(std::ifstream& os);

private:
    //base
    uint64_t id_ = UINT64_MAX;
    bool bKey_ = false;
    uint64_t kfId_ = UINT64_MAX;
    CameraModule* cam_ = nullptr;
    Pose3D Tcw_;

    ///后续将对应特征相关参数打包，便于管理和使用
    //keypoint
    //std::vector<Eigen::Vector2f> kps_;
    std::vector<cv::KeyPoint> kps_;
    cv::Mat descriptors_;
    KeyPointIndexMode kpIndMode_ = GRID;
    std::shared_ptr<SLAM_LYJ_MATH::KdTree2d> kdtree_ = nullptr;
    std::shared_ptr<SLAM_LYJ_MATH::Grid2Df> grid_ = nullptr;
    //line
    std::vector<cv::Vec4f> vecLines_;
    std::vector<cv::line_descriptor::KeyLine> vecKeyLines_;
    cv::Mat lineDescriptors_;
    //edge
    //std::vector<Eigen::Vector2f> edges_;
    //std::vector<Eigen::Vector2f> edgesDir_;
    //bool isParallel_ = false;
    //Eigen::Vector3f nParallel_ = Eigen::Vector3f::Zero();
    //Eigen::Vector3f nParallelVar_ = Eigen::Vector3f::Zero();
    std::shared_ptr<EdgeFeatures> edgeFeatures_ = nullptr;

    //debug
    std::string imgPath_ = "";
    cv::Mat img_;
};


NSP_SLAM_LYJ_END

#endif //SLAM_LYJ_FRAME_H