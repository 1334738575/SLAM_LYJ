#include <iostream>
#include "SLAM_LYJ.h"
#include "QT_LYJ.h"
#include <SLAM_LYJ_src_Include.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/line_descriptor.hpp>
#include <ceres/ceres.h>

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

#include <CUDAInclude.h>
//#include <DBoW2/TemplatedVocabulary.h>
//#include <DBoW2/FORB.h>
#include <Optimize_LYJ.h>
#include <common/BaseTriMesh.h>
#include <IO/MeshIO.h>
#include <PathPlanningDefines.h>
#include <PathPlanningInclude.h>
#include <base/Pose.h>
#include <opencv2/highgui.hpp>
#include <base/CameraModule.h>
#include <Variable/Variable.h>
#include <Factor/Factor.h>
#include <Optimizer/optimizer.h>
#include <VulkanBuffer.h>
#include <VulkanCommon.h>
#include <ImageProcess_LYJ_Include.h>
#include <IO/SimpleIO.h>

#include <projector/ProjectorVK.h>

#include <fstream>
#include <random>
#include <string>
#include <cmath>

using namespace std;
using namespace Eigen;
using namespace OPTIMIZE_LYJ;
// 生成3D点云 ([-1,1]x[-1,1]x[2,5]范围)
vector<Vector3d> generate3DPoints(int num_points)
{
    vector<Vector3d> points;
    points.reserve(num_points);

    for (int i = 0; i < num_points; ++i)
    {
        Vector3d p = Vector3d::Random();
        p = p + Vector3d(0, 0, 2);
        points.push_back(p);
    }
    return points;
}
// 投影3D点到图像平面
Vector2d project(const Vector3d& p3d, const Matrix3d& K, const Eigen::Matrix<double, 3, 4>& T_cam)
{
    Matrix3d R = T_cam.block<3, 3>(0, 0);
    Vector3d t = T_cam.block<3, 1>(0, 3);
    Vector3d p_cam = R * p3d + t;
    Vector3d uv_hom = K * p_cam;
    return uv_hom.hnormalized();
}
// 验证重投影误差
void verifyReprojection(const vector<Vector3d>& points3d,
    const vector<pair<Vector2d, Vector2d>>& matches,
    const Matrix3d& K,
    const Eigen::Matrix<double, 3, 4>& T1,
    const Eigen::Matrix<double, 3, 4>& T2)
{
    double max_error = 0;
    for (size_t i = 0; i < points3d.size(); ++i)
    {
        Vector2d p1 = project(points3d[i], K, T1);
        Vector2d p2 = project(points3d[i], K, T2);

        double e1 = (p1 - matches[i].first).norm();
        double e2 = (p2 - matches[i].second).norm();
        max_error = max({ max_error, e1, e2 });
    }
    cout << "Max reprojection error: " << max_error << " pixels" << endl;
}


struct Point3D
{
    float x, y, z;
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

void generate_cuboid_surface_points(
    float length, float width, float height,
    int num_points, const std::string& filename = "D:/tmp/cuboid_points.ply")
{
    std::vector<Point3D> points;
    points.reserve(num_points);

    // 随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());

    // 每个面的基础点数
    const int base_points = num_points / 6;
    const int remainder = num_points % 6;

    // 定义长方体6个面的生成规则
    struct FaceRule
    {
        enum FixedAxis
        {
            X,
            Y,
            Z
        };
        FixedAxis axis;
        float fixed_value;
        std::pair<float, float> range1;
        std::pair<float, float> range2;
    };

    const FaceRule faces[] = {
        {FaceRule::X, length / 2, {-width / 2, width / 2}, {-height / 2, height / 2}},   // 前面
        {FaceRule::X, -length / 2, {-width / 2, width / 2}, {-height / 2, height / 2}},  // 后面
        {FaceRule::Y, width / 2, {-length / 2, length / 2}, {-height / 2, height / 2}},  // 右面
        {FaceRule::Y, -width / 2, {-length / 2, length / 2}, {-height / 2, height / 2}}, // 左面
        {FaceRule::Z, height / 2, {-length / 2, length / 2}, {-width / 2, width / 2}},   // 顶面
        {FaceRule::Z, -height / 2, {-length / 2, length / 2}, {-width / 2, width / 2}}   // 底面
    };

    // 生成每个面的点
    for (int i = 0; i < 6; ++i)
    {
        const auto& face = faces[i];
        const int points_this_face = base_points + (i < remainder ? 1 : 0);

        std::uniform_real_distribution<float> dist1(face.range1.first, face.range1.second);
        std::uniform_real_distribution<float> dist2(face.range2.first, face.range2.second);

        for (int j = 0; j < points_this_face; ++j)
        {
            const float val1 = dist1(gen);
            const float val2 = dist2(gen);

            switch (face.axis)
            {
            case FaceRule::X:
                points.emplace_back(face.fixed_value, val1, val2);
                break;
            case FaceRule::Y:
                points.emplace_back(val1, face.fixed_value, val2);
                break;
            case FaceRule::Z:
                points.emplace_back(val1, val2, face.fixed_value);
                break;
            }
        }
    }

    //// 写入PLY文件
    // std::ofstream file(filename);
    // if (!file) {
    //     std::cerr << "无法打开文件: " << filename << std::endl;
    //     return;
    // }
    // file << "ply\n"
    //     << "format ascii 1.0\n"
    //     << "element vertex " << points.size() << "\n"
    //     << "property float x\n"
    //     << "property float y\n"
    //     << "property float z\n"
    //     << "end_header\n";
    // for (const auto& p : points) {
    //     file << p.x << " " << p.y << " " << p.z << "\n";
    // }

    std::vector<Eigen::Vector3f> Ps(num_points);
    for (int i = 0; i < num_points; ++i)
        Ps[i] = Eigen::Vector3f(points[i].x, points[i].y, points[i].z);
    SLAM_LYJ::BaseTriMesh btmPoints;
    btmPoints.setVertexs(Ps);
    SLAM_LYJ::writePLYMesh(filename, btmPoints);

    std::cout << "已生成 " << points.size() << " 个点至 " << filename << std::endl;
}

struct PoseOut
{
    double R[3][3]{ {1, 0, 0}, {0, 1, 0}, {0, 0, 1} }; // 旋转矩阵
    double t[3]{ 0, 0, 0 };                            // 平移向量
};

struct Observation
{
    int point_id;
    double u, v; // 像素坐标
};

// 3D点结构体
struct Point3DOut
{
    double x, y, z;
};

// 相机内参配置
const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 480;
const double fx = 525.0; // 焦距x
const double fy = 525.0; // 焦距y
const double cx = 319.5; // 主点x
const double cy = 239.5; // 主点y

// 生成相机位姿
void generate_camera_poses(PoseOut& pose1, PoseOut& pose2)
{
    // 第一个位姿：正对长方体，距离2米
    // pose1.R = { {1,0,0}, {0,1,0}, {0,0,1} }; // 单位矩阵
    pose1.t[2] = -5.0; // Z方向平移

    // 第二个位姿：右上方45度视角
    const double theta = 3.1415926 / 4; // 45度
    pose2.R[0][0] = cos(theta);
    pose2.R[0][2] = sin(theta);
    pose2.R[1][1] = 1;
    pose2.R[2][0] = -sin(theta);
    pose2.R[2][2] = cos(theta);
    pose2.t[0] = -3.0;
    pose2.t[1] = -0.5;
    pose2.t[2] = -4.5;
}

// 投影3D点到图像平面
bool project_point(const Point3DOut& world_point,
    const PoseOut& pose,
    double& u, double& v)
{
    // 世界坐标转相机坐标
    double Xc = pose.R[0][0] * (world_point.x) +
        pose.R[0][1] * (world_point.y) +
        pose.R[0][2] * (world_point.z) + pose.t[0];
    double Yc = pose.R[1][0] * (world_point.x) +
        pose.R[1][1] * (world_point.y) +
        pose.R[1][2] * (world_point.z) + pose.t[1];
    double Zc = pose.R[2][0] * (world_point.x) +
        pose.R[2][1] * (world_point.y) +
        pose.R[2][2] * (world_point.z) + pose.t[2];

    // 检查深度
    if (Zc <= 0.1)
        return false; // 忽略太近的点

    // 透视投影
    u = fx * (Xc / Zc) + cx;
    v = fy * (Yc / Zc) + cy;

    // 检查边界
    return (u >= 0 && u < IMAGE_WIDTH && v >= 0 && v < IMAGE_HEIGHT);
}

int generateObs()
{
    SLAM_LYJ::BaseTriMesh btmPoints;
    SLAM_LYJ::readPLYMesh("D:/tmp/cuboid_points.ply", btmPoints);
    // 加载3D点（示例数据，实际应从文件读取）
    std::vector<Point3DOut> points(btmPoints.getVn());
    const auto& Ps = btmPoints.getVertexs();
    for (int i = 0; i < btmPoints.getVn(); ++i)
    {
        points[i].x = Ps[i](0);
        points[i].y = Ps[i](1);
        points[i].z = Ps[i](2);
    }

    // 生成相机位姿
    PoseOut pose1, pose2;
    generate_camera_poses(pose1, pose2);
    SLAM_LYJ::Pose3D Twc1;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            Twc1.getR()(i, j) = pose1.R[i][j];
        }
        Twc1.gett()(i) = pose1.t[i];
    }
    SLAM_LYJ::Pose3D Twc2;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            Twc2.getR()(i, j) = pose2.R[i][j];
        }
        Twc2.gett()(i) = pose2.t[i];
    }
    SLAM_LYJ::Pose3D Tcw1 = Twc1.inversed();
    SLAM_LYJ::Pose3D Tcw2 = Twc2.inversed();
    SLAM_LYJ::PinholeCamera cam(IMAGE_WIDTH, IMAGE_HEIGHT, fx, fy, cx, cy);
    COMMON_LYJ::drawCam("D:/tmp/cam1.ply", cam, Twc1, 10);
    COMMON_LYJ::drawCam("D:/tmp/cam2.ply", cam, Twc2, 10);
    COMMON_LYJ::writeT34("D:/tmp/Tcw1.txt", Tcw1);
    COMMON_LYJ::writeT34("D:/tmp/Tcw2.txt", Tcw2);
    COMMON_LYJ::writePinCamera("D:/tmp/cam.txt", cam);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            pose1.R[i][j] = Tcw1.getR()(i, j);
        }
        pose1.t[i] = Tcw1.gett()(i);
    }
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            pose2.R[i][j] = Tcw2.getR()(i, j);
        }
        pose2.t[i] = Tcw2.gett()(i);
    }

    // 存储观测数据
    std::vector<Observation> obs1, obs2;

    // 进行投影
    for (size_t i = 0; i < points.size(); ++i)
    {
        double u, v;

        // 投影到第一个位姿
        if (project_point(points[i], pose1, u, v))
        {
            obs1.push_back({ static_cast<int>(i), u, v });
        }

        // 投影到第二个位姿
        if (project_point(points[i], pose2, u, v))
        {
            obs2.push_back({ static_cast<int>(i), u, v });
        }
    }

    // 保存结果
    std::ofstream out("D:/tmp/observations.txt");
    out << "Camera Pose1:\n";
    out << "R: [" << pose1.R[0][0] << "," << pose1.R[0][1] << "," << pose1.R[0][2] << "\n"
        << pose1.R[1][0] << "," << pose1.R[1][1] << "," << pose1.R[1][2] << "\n"
        << pose1.R[2][0] << "," << pose1.R[2][1] << "," << pose1.R[2][2] << "]\n";
    out << "t: [" << pose1.t[0] << "," << pose1.t[1] << "," << pose1.t[2] << "]\n";
    out << "Observations: " << obs1.size() << "\n";
    for (const auto& o : obs1)
    {
        out << o.point_id << " " << o.u << " " << o.v << "\n";
    }

    out << "\nCamera Pose2:\n";
    out << "R: [" << pose2.R[0][0] << "," << pose2.R[0][1] << "," << pose2.R[0][2] << "\n"
        << pose2.R[1][0] << "," << pose2.R[1][1] << "," << pose2.R[1][2] << "\n"
        << pose2.R[2][0] << "," << pose2.R[2][1] << "," << pose2.R[2][2] << "]\n";
    out << "t: [" << pose2.t[0] << "," << pose2.t[1] << "," << pose2.t[2] << "]\n";
    out << "Observations: " << obs2.size() << "\n";
    for (const auto& o : obs2)
    {
        out << o.point_id << " " << o.u << " " << o.v << "\n";
    }

    std::cout << "观测数据已保存至 observations.txt" << std::endl;
    return 0;
}

int main2(int argc, char* argv[])
{
    //// 示例：生成长=2.0，宽=1.0，高=0.5的长方体表面1000个点
    // generate_cuboid_surface_points(2.0f, 1.0f, 0.5f, 1000);
    // generateObs();
    // return 0;

    // OPTIMIZE_LYJ::ceres_Check_UV_Pose3d_P3d2();
    ////test_optimize_UV_Pose3d_P3d2_v2();
    // return 0;
    std::cout << "start test" << std::endl;
    using namespace SLAM_LYJ;

    if (false)
    {
        std::vector<double> camd = { 765.955, 766.549, 1024, 1024 };
        SLAM_LYJ::PinholeCamera cam(2048, 2048, camd);

        cv::Mat img = cv::imread("D:/tmp/images/7.png", 0);
        cv::pyrDown(img, img);
        // cv::imshow("image", img);
        // cv::waitKey();
        ImageProcess_LYJ::ImageExtractData frame;
        frame.cam = &cam;
        frame.img = img;
        ImageProcess_LYJ::extractFeature(&frame);
        std::cout << "key point size: " << frame.kps_.size() << std::endl;
        cv::drawKeypoints(img, frame.kps_, img, cv::Scalar(255, 0, 0));
        cv::imshow("keypoints", img);
        cv::waitKey();
        cv::Mat img2 = cv::imread("D:/tmp/images/8.png", 0);
        cv::pyrDown(img2, img2);
        ImageProcess_LYJ::ImageExtractData frame2;
        frame2.Tcw.gett() = Eigen::Vector3d(0.1, 0.1, 0.1);
        frame2.cam = &cam;
        frame2.img = img2;
        ImageProcess_LYJ::extractFeature(&frame2);
        std::cout << "key point2 size: " << frame2.kps_.size() << std::endl;
        cv::drawKeypoints(img2, frame2.kps_, img2, cv::Scalar(255, 0, 0));
        cv::imshow("keypoints2", img2);
        cv::waitKey();

        ImageProcess_LYJ::ImageMatchData matchResult;
        matchResult.usePointMatch = true;
        matchResult.debugPath = "D:/tmp/imageProcess/match/";
        ImageProcess_LYJ::matchFeature(&frame, &frame2, &matchResult);
    }

    //if (false)
    //{
    //    DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> voc;
    //}
    if (false)
    {
        using namespace OPTIMIZE_LYJ;
        test_optimize_UV_Pose3d_P3d2();
        return 0;
        // OPTIMIZE_LYJ::optimize_version();
        // OPTIMIZE_LYJ::test_optimize_Pose3d_Pose3d();
        // std::shared_ptr<OptVarAbr<double>> varSPtr = std::make_shared<OptVarPoint3d>(0);
        // Eigen::Vector3d data(1, 2, 3);
        // varSPtr->setData(data.data());
        // std::shared_ptr<OptFactorAbr<double>> factorSPtr = std::make_shared<OptFactorP3d_P3d>(0);
        // Eigen::Vector3d obs(0, 0, 0);
        // OptFactorP3d_P3d *factor = dynamic_cast<OptFactorP3d_P3d *>(factorSPtr.get());
        // factor->setObs(obs.data());
        // std::vector<uint64_t> vIds;
        // vIds.push_back(0);
        //{
        //     // OptimizerSmalld optimizer;
        //     OptimizerLargeSparse optimizer;
        //     optimizer.addVariable(varSPtr);
        //     optimizer.addFactor(factorSPtr, vIds);
        //     optimizer.run();
        // }
        // return 0;
    }
    SLAM_LYJ::BaseTriMesh btm;
    SLAM_LYJ::readPLYMesh("D:/tmp/res_mesh.ply", btm);
    // SLAM_LYJ::writePLYMesh("D:/tmp/copy3.ply", btm);
    //if (false)
    //{
    //    std::vector<Eigen::Vector3f> vertexs = btm.getVertexs();
    //    std::vector<BaseTriFace> fs = btm.getFaces();
    //    btm.enableFCenters();
    //    btm.calculateFCenters();
    //    btm.enableFNormals();
    //    btm.calculateFNormals();
    //    std::string imgName = "D:/tmp/images/7.png";
    //    cv::Mat img = cv::imread(imgName);
    //    // cv::imshow("img", img);
    //    // cv::waitKey();
    //    /*
    //    * 765.955       0    1024
    //      0 766.549    1024
    //      0       0       1
    //    */
    //    float cam[4] = { 765.955, 766.549, 1024, 1024 };
    //    std::vector<double> camd = { 765.955, 766.549, 1024, 1024 };
    //    int w = img.cols;
    //    int h = img.rows;
    //    PinholeCamera pinCam(2048, 2048, camd);

    //    Pose3D TcwP;
    //    TcwP.getR() << 0.00774473, 0.0383352, 0.999235,
    //        -0.0480375, 0.998125, -0.0379202,
    //        -0.998815, -0.047707, 0.00957183;
    //    TcwP.gett() << -0.0615093,
    //        -0.16702,
    //        0.011672;
    //    Eigen::Vector3d p1(0, 0, 10);
    //    Eigen::Vector3d p2(w, 0, 10);
    //    Eigen::Vector3d p3(w, h, 10);
    //    Eigen::Vector3d p4(0, h, 10);
    //    Eigen::Vector3d P0(0, 0, 0);
    //    Eigen::Vector3d P1;
    //    Eigen::Vector3d P2;
    //    Eigen::Vector3d P3;
    //    Eigen::Vector3d P4;
    //    pinCam.image2World(p1, P1);
    //    pinCam.image2World(p2, P2);
    //    pinCam.image2World(p3, P3);
    //    pinCam.image2World(p4, P4);
    //    Pose3D Twc = TcwP.inversed();
    //    P0 = Twc * P0;
    //    P1 = Twc * P1;
    //    P2 = Twc * P2;
    //    P3 = Twc * P3;
    //    P4 = Twc * P4;
    //    BaseTriMesh btmTmp;
    //    std::vector<Eigen::Vector3f> psTmp;
    //    psTmp.push_back(P0.cast<float>());
    //    psTmp.push_back(P1.cast<float>());
    //    psTmp.push_back(P2.cast<float>());
    //    psTmp.push_back(P3.cast<float>());
    //    psTmp.push_back(P4.cast<float>());
    //    std::vector<BaseTriFace> fsTmp;
    //    fsTmp.emplace_back(0, 1, 2);
    //    fsTmp.emplace_back(0, 2, 3);
    //    fsTmp.emplace_back(0, 3, 4);
    //    fsTmp.emplace_back(0, 4, 1);
    //    btmTmp.setVertexs(psTmp);
    //    btmTmp.setFaces(fsTmp);
    //    writePLYMesh("D:/tmp/cam.ply", btmTmp);

    //    float* Pws = vertexs[0].data();
    //    unsigned int PSize = btm.getVn();
    //    float* centers = btm.getFCenters()[0].data();
    //    float* fNormals = btm.getFNormals()[0].data();
    //    unsigned int* faces = fs[0].vId_;
    //    unsigned int fSize = btm.getFn();
    //    float* camParams = cam;
    //    CUDA_LYJ::ProHandle proHandle = CUDA_LYJ::initProjector(Pws, PSize, centers, fNormals, faces, fSize, camParams, w, h);

    //    Eigen::Matrix<float, 3, 4> Tcw34;
    //    Tcw34.block(0, 0, 3, 3) = TcwP.getR().cast<float>();
    //    Tcw34.block(0, 3, 3, 1) = TcwP.gett().cast<float>();
    //    float* Tcw = Tcw34.data();
    //    cv::Mat depthsM(w, h, CV_32FC1);
    //    float* depths = (float*)depthsM.data;
    //    std::vector<char> allvisiblePIds(PSize, 0);
    //    std::vector<char> allvisibleFIds(fSize, 0);
    //    std::vector<unsigned int> fIdss(w * h, 0);
    //    unsigned int* fIds = fIdss.data();
    //    char* allVisiblePIds = allvisiblePIds.data();
    //    char* allVisibleFIds = allvisibleFIds.data();
    //    CUDA_LYJ::project(proHandle, Tcw, depths, fIds, allVisiblePIds, allVisibleFIds);
    //    cv::Mat depthsMShow(w, h, CV_8UC1);
    //    depthsMShow.setTo(cv::Scalar(0));
    //    std::vector<Eigen::Vector3f> Pcs;
    //    Pcs.reserve(w * h);
    //    for (int i = 0; i < h; ++i)
    //    {
    //        for (int j = 0; j < w; ++j)
    //        {
    //            float d = depths[i * w + j];
    //            if (d == FLT_MAX)
    //            {
    //                depthsMShow.at<uchar>(i, j) = 0;
    //                continue;
    //            }
    //            Eigen::Vector3d Pc;
    //            pinCam.image2World(j, i, d, Pc);
    //            Pcs.push_back(Pc.cast<float>());
    //            depthsMShow.at<uchar>(i, j) = d * 20 < 255 ? (char)(d * 20) : 255;
    //        }
    //    }
    //    cv::imshow("depth", depthsMShow);
    //    cv::waitKey();
    //    BaseTriMesh btmPcs;
    //    btmPcs.setVertexs(Pcs);
    //    writePLYMesh("D:/tmp/BtmPcs.ply", btmPcs);

    //    CUDA_LYJ::release(proHandle);
    //}
    if (false)
    {
        Eigen::Vector3d minP(-50, -50, -50);
        Eigen::Vector3d maxP(50, 50, 50);
        double resolution = 0.5;
        double rbtRadius = 0.05;
        std::vector<Eigen::Vector3d> obstacles;
        const auto& vertexs = btm.getVertexs();
        int vSize = vertexs.size();
        for (int i = 0; i < vSize; ++i)
        {
            obstacles.push_back(vertexs[i].cast<double>());
        }
        // std::cout << obstacles.back() << std::endl;
        PATH_PLAN_LYJ::PathPlanParam param;
        param.method = PATH_PLAN_LYJ::ASTAR;
        param.debugPath = "D:/tmp";
        PATH_PLAN_LYJ::PathPlanHandle handle = PATH_PLAN_LYJ::createPathPlanner(minP, maxP, resolution, rbtRadius, obstacles, param);

        Eigen::Vector3d src(-2, -3, 1);
        Eigen::Vector3d dst(7, 10, 0.3);
        std::vector<Eigen::Vector3d> path;
        if (!PATH_PLAN_LYJ::planTwoLocations(handle, src, dst, path))
            std::cout << "can't find path!" << std::endl;

        std::ofstream f(param.debugPath + "/PathPlanPath.txt");
        for (size_t i = 0; i < path.size(); ++i)
        {
            f << path[i](0) << " " << path[i](1) << " " << path[i](2) << " " << 0 << " " << 255 << " " << 0 << std::endl;
        }
        f.close();

        PATH_PLAN_LYJ::releasePathPlanner(handle);
    }
    if (true)
    {
        // data
        SLAM_LYJ::BaseTriMesh btmVulkan = btm;
        const auto& vertexs = btmVulkan.getVertexs();
        const auto& faces = btmVulkan.getFaces();
        btmVulkan.enableFCenters();
        btmVulkan.calculateFCenters();
        const auto& fCenters = btmVulkan.getFCenters();
        btmVulkan.enableFNormals();
        btmVulkan.calculateFNormals();
        const auto& fNormals = btmVulkan.getFNormals();
        int vn = btmVulkan.getVn();
        int fn = btmVulkan.getFn();
        int w = 2048;
        int h = 2048;
        std::vector<float> K{ 765.955, 766.549, 1024, 1024 };
        std::vector<double> Kd{ 765.955, 766.549, 1024, 1024 };
        Pose3D TcwP;
        TcwP.getR() << 0.00774473, 0.0383352, 0.999235,
            -0.0480375, 0.998125, -0.0379202,
            -0.998815, -0.047707, 0.00957183;
        TcwP.gett() << -0.0615093,
            -0.16702,
            0.011672;
        SLAM_LYJ::PinholeCamera cam(w, h, Kd);
        COMMON_LYJ::drawCam("D:/tmp/camVulkan.ply", cam, TcwP.inversed(), 10);
        Eigen::Matrix<float, 3, 4> T;
        T.block(0, 0, 3, 3) = TcwP.getR().cast<float>();
        T.block(0, 3, 3, 1) = TcwP.gett().cast<float>();


        LYJ_VK::ProjectorVK projectVK;
        projectVK.create(vertexs[0].data(), vn, fCenters[0].data(), fNormals[0].data(), faces[0].vId_, fn, K.data(), w, h);
        std::vector<uint> fIdsOut(w * h, UINT_MAX);
        std::vector<float> depthsOut(w * h, FLT_MAX);
        std::vector<char> PValidsOut(vn, 0);
        std::vector<char> fValidsOut(fn, 0);
        projectVK.project(T.data(), depthsOut.data(), fIdsOut.data(), PValidsOut.data(), fValidsOut.data(), 0, 30, 0.0, 0.1);
        SLAM_LYJ::Pose3D Tcw2;
        COMMON_LYJ::readT34("D:/tmp/texture_data/RT_61.txt", Tcw2);
        COMMON_LYJ::drawCam("D:/tmp/camVulkan.ply", cam, Tcw2.inversed(), 10);
        Eigen::Matrix<float, 3, 4> T2;
        T2.block(0, 0, 3, 3) = Tcw2.getR().cast<float>();
        T2.block(0, 3, 3, 1) = Tcw2.gett().cast<float>();
        projectVK.project(T2.data(), depthsOut.data(), fIdsOut.data(), PValidsOut.data(), fValidsOut.data(), 0, 30, 0.0, 0.1);

        {
            std::vector<Eigen::Vector3f> retPs;
            for (int i = 0; i < vn; ++i)
            {
                if (PValidsOut[i] == 0)
                    continue;
                retPs.push_back(vertexs[i]);
            }
            SLAM_LYJ::BaseTriMesh btmTmp;
            btmTmp.setVertexs(retPs);
            SLAM_LYJ::writePLYMesh("D:/tmp/checkV.ply", btmTmp);
            std::vector<Eigen::Vector3f> retFs;
            for (int i = 0; i < fn; ++i)
            {
                if (fValidsOut[i] == 1)
                    continue;
                retFs.push_back(fCenters[i]);
            }
            SLAM_LYJ::BaseTriMesh btmTmp2;
            btmTmp2.setVertexs(retFs);
            SLAM_LYJ::writePLYMesh("D:/tmp/checkF.ply", btmTmp2);
        }
        {
            std::vector<Eigen::Vector3f> fccc;
            for (int i = 0; i < h; ++i)
            {
                for (int j = 0; j < w; ++j)
                {
                    const uint32_t& fid = fIdsOut[i * w + j];
                    if (fid == UINT_MAX)
                        continue;
                    fccc.push_back(fCenters[fid]);
                }
            }
            SLAM_LYJ::BaseTriMesh btmtmp;
            btmtmp.setVertexs(fccc);
            SLAM_LYJ::writePLYMesh("D:/tmp/fccc.ply", btmtmp);
        }
        {
            std::vector<Eigen::Vector3f> PcsTmp;
            Eigen::Vector2d uvTmp;
            cv::Mat mmmd(h, w, CV_8UC1);
            for (int i = 0; i < h; ++i)
            {
                for (int j = 0; j < w; ++j)
                {
                    Eigen::Vector3d Pc;
                    double dd = depthsOut[i * w + j];
                    uvTmp(0) = j;
                    uvTmp(1) = i;
                    if (dd > 0.0f && dd != FLT_MAX)
                    {
                        cam.image2World(uvTmp, dd, Pc);
                        PcsTmp.push_back(Pc.cast<float>());
                    }
                    const float ddd = depthsOut[i * w + j] / 30.0f;
                    int dddc = ddd * 255 > 255 ? 255 : ddd * 255;
                    mmmd.at<uchar>(i, j) = (uchar)dddc;
                }
            }
            SLAM_LYJ::BaseTriMesh btmtmp;
            btmtmp.setVertexs(PcsTmp);
            SLAM_LYJ::writePLYMesh("D:/tmp/PcsTmp.ply", btmtmp);
            cv::imwrite("D:/tmp/depth.png", mmmd);
        }

        projectVK.release();



    }
    if (false)
    {
        std::vector<double> camd = { 765.955, 766.549, 1024, 1024 };
        SLAM_LYJ::PinholeCamera cam(2048, 2048, camd);

        std::string dataPath = "D:/tmp/images/";
        std::string dataPath2 = "D:/tmp/texture_data/RT_";
        int id1 = 47;
        int id2 = 49;
        auto funcReadTcw = [](const std::string& _file, SLAM_LYJ::Pose3D& Tcw)
            {
                std::ifstream f(_file);
                if (!f.is_open())
                {
                    std::cout << "read pose failed!" << std::endl;
                    return;
                }
                f >> Tcw.getR()(0, 0) >> Tcw.getR()(0, 1) >> Tcw.getR()(0, 2) >> Tcw.gett()(0) >> Tcw.getR()(1, 0) >> Tcw.getR()(1, 1) >> Tcw.getR()(1, 2) >> Tcw.gett()(1) >> Tcw.getR()(2, 0) >> Tcw.getR()(2, 1) >> Tcw.getR()(2, 2) >> Tcw.gett()(2);
                f.close();
            };

        cv::Mat img = cv::imread(dataPath + std::to_string(id1) + ".png", 0);
        ImageProcess_LYJ::ImageExtractData frame;
        frame.id = id1;
        frame.cam = &cam;
        frame.img = img;
        funcReadTcw(dataPath2 + std::to_string(id1) + ".txt", frame.Tcw);
        ImageProcess_LYJ::extractFeature(&frame);
        std::cout << "key point size: " << frame.kps_.size() << std::endl;
        // cv::drawKeypoints(img, frame.kps_, img, cv::Scalar(255, 0, 0));
        // cv::imshow("keypoints", img);
        // cv::waitKey();
        cv::Mat img2 = cv::imread(dataPath + std::to_string(id2) + ".png", 0);
        ImageProcess_LYJ::ImageExtractData frame2;
        frame2.id = id2;
        frame2.cam = &cam;
        frame2.img = img2;
        funcReadTcw(dataPath2 + std::to_string(id2) + ".txt", frame2.Tcw);
        ImageProcess_LYJ::extractFeature(&frame2);
        std::cout << "key point2 size: " << frame2.kps_.size() << std::endl;
        // cv::drawKeypoints(img2, frame2.kps_, img2, cv::Scalar(255, 0, 0));
        // cv::imshow("keypoints2", img2);
        // cv::waitKey();

        ImageProcess_LYJ::ImageMatchData matchResult;
        matchResult.usePointMatch = true;
        matchResult.debugPath = "D:/tmp/imageProcess/match/";
        ImageProcess_LYJ::matchFeature(&frame, &frame2, &matchResult);

        ImageProcess_LYJ::ImageTriangleData triangleResult;
        ImageProcess_LYJ::reconstructTwo(&frame, &frame2, &matchResult, &triangleResult);
        std::vector<Eigen::Vector3f> PwsTir;
        for (int i = 0; i < triangleResult.bTris.size(); ++i)
        {
            if (!triangleResult.bTris[i])
                continue;
            PwsTir.push_back(triangleResult.Ps[i].cast<float>());
        }
        SLAM_LYJ::BaseTriMesh btmTri;
        btmTri.setVertexs(PwsTir);
        SLAM_LYJ::writePLYMesh("D:/tmp/tri.ply", btmTri);
    }

    std::cout << "end test" << std::endl;
    return 0;
}

template<typename T>
void testTemplate(const T* _a) {
    std::cout << _a << std::endl;
}

template<int A>
Eigen::Matrix<double, A, 1> createMatrix() {
    return Eigen::Matrix<double, A, 1>::Zero();
}
template<int... Args>
class GrowClass
{
public:
    GrowClass() {
    }
    void printTuple() {
        ((std::cout << createMatrix<Args>() << std::endl << std::endl), ...);
        std::cout << std::endl;
        ((std::cout << Eigen::Matrix<double, Args, 1>::Zero() << std::endl << std::endl), ...);
        std::cout << std::endl;
        ((std::cout << Args << " "), ...);
        std::cout << std::endl;
        auto tpl = std::make_tuple(Args...);
        std::vector<int> v = std::vector<int>{ Args... };
        std::apply([](const auto&... elems) {
            ((std::cout << elems << " "), ...);
            }, std::make_tuple(Args...));
        std::cout << std::endl;
    }
    ~GrowClass() {}

private:

};
void testGrowTemplate() {
    GrowClass<2, 3, 4, 6> cls;
    cls.printTuple();
}
void test() {
    std::vector<Eigen::Vector3i> dd;
    for (int i = 0; i < 10; ++i) {
        dd.push_back(Eigen::Vector3i(i, i, i));
    }
    for (size_t i = 0; i < 10; i++)
    {
        std::cout << i << "-- \n" << dd[i] << std::endl;
    }
    const int dim = 0;
    auto funcComp = [&](const Eigen::Vector3i& _v1, const Eigen::Vector3i& _v2)->bool {
        if (_v1[dim] < _v2[dim])
            return false;
        return true;
    };
    std::sort(dd.begin(), dd.end(), funcComp);
    for (size_t i = 0; i < 10; i++)
    {
        std::cout << i << "-- \n" << dd[i] << std::endl;
    }
    return;
}

int main(int argc, char* argv[]){
    //main2(argc, argv);
    //test();
    //testGrowTemplate();
    std::cout<<"Hello SLAM_LYJ!" <<std::endl;
    //std::cout<< "Current version is: " << SLAM_LYJ::getVersion() << std::endl;
    //SLAM_LYJ::testCeres();
    //SLAM_LYJ::testEigen();
    //SLAM_LYJ::testOpenCV();
    //SLAM_LYJ::testThreadPool();
    //SLAM_LYJ::testVulkan();
    //SLAM_LYJ::testKdTree();
    //SLAM_LYJ::testPoint();
    //SLAM_LYJ::testArchive();
    // SLAM_LYJ::testTensor();
    //SLAM_LYJ::testCommonAlgorithm();
    //SLAM_LYJ::testRANSAC();
    //SLAM_LYJ::testLine();
    //SLAM_LYJ::testPlane();
    //SLAM_LYJ::testPose();
    //SLAM_LYJ::testCamera();
    //SLAM_LYJ::testTriangler();
    //SLAM_LYJ::testOptimizer();
    //SLAM_LYJ::testBitFlagVec();
    //SLAM_LYJ::testFrame();
    //SLAM_LYJ::testIncrementalAgvAndVar();
    //SLAM_LYJ::testGrid();
    //SLAM_LYJ::testSTLPlus();
    //SLAM_LYJ::testBuffer();
    //SLAM_LYJ::testGlobalOption();
	//SLAM_LYJ::testFlann();
	//SLAM_LYJ::testPCL();
    //QT_LYJ::testQT(argc, argv);
    //SLAM_LYJ::testPatchMatch();
    //SLAM_LYJ::testDiffuser();
    //SLAM_LYJ::testPolarGrid();
    //QT_LYJ::testOpenGLOnly();
    //SLAM_LYJ::testOcTreeAndQuadTree();
    //SLAM_LYJ::testCUDA();
    //return 0;
    //QT_LYJ::debugWindows(argc, argv);
    //return 0;
    SLAM_LYJ_src::ProcessOption opt;
    opt.imgDir = "D:/tmp/testImages";
    opt.priTcwDir = "D:/tmp/testTcws";
    opt.camFile = "D:/tmp/testCam.txt";
    opt.vocPath = "D:/tmp/voc.gz";
    opt.writeVoc = false;
    opt.imageMatchOpt.bTriangle = false;
    opt.imageMatchOpt.pointMatchCheck = true;
    //opt.readCache = true;
    opt.meshPath = "D:/tmp/res_mesh.ply";
    SLAM_LYJ_src::reconstructVisualPoint(opt);
    //SLAM_LYJ_src::reconstructVisualWithMesh(opt);
    QT_LYJ::debugWindows(argc, argv);
    return 0;
}