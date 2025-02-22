#include "SLAM_LYJ.h"
#include "base/PreDefine.h"
#include "common/ThreadPool.h"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include "vulkanImp/vulkanImp.h"
#include "base/Triangler.h"
#include "optimizer/Factor.h"
#include "base/Frame.h"
#include "extractor/ORBextractor.h"
#include "extractor/LSDextractor.h"
#include "extractor/Cannyextractor.h"
#include "extractor/SIFTextractor.h"
#include "STLPlus/include/file_system.h"

#include "debugger/debugger.h"

NSP_SLAM_LYJ_BEGIN

class CURVE_FITTING_COST {
public:
    CURVE_FITTING_COST(double x, double y) :_x(x), _y(y) {}
    template<typename T>
    bool operator()(const T* const abc, T* residual) const
    {
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
    }
    const double _x, _y; //x,y����
};
SLAM_LYJ_API void testCeres() {
    double ae = 2.0, be = -1.0, ce = 5.0;        // ���Ʋ���ֵ
    //�ڴ���������Ͼ��ȵ�����100�����ݵ㣬���ϰ���������Ϊ���������
    int N = 100;                                 // ���ݵ�
    double w_sigma = 1.0;                        // ����Sigmaֵ
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;                                 // OpenCV�����������
    std::vector<double> x_data, y_data;      // ����
    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ae * x * x + be * x + ce) + rng.gaussian(w_sigma * w_sigma));
    }
    double abc[3] = { ae,be,ce };
    //����������С��������
    ceres::Problem problem;
    for (int i = 0; i < N; ++i)
    {
        //����������������ʹ���Զ��󵼣�ģ�������������ͣ����ά�ȣ��в��ά�ȣ�������ά�ȣ����Ż�������ά�ȣ���ά��Ҫ��ǰ���struct��һ�£�
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(new CURVE_FITTING_COST(x_data[i], y_data[i])),
            nullptr,  //�˺��������ﲻ��Ҫ����Ϊ��
            abc      //�����Ʋ���
        );
    }
    //�������������
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;//��������������
    options.minimizer_progress_to_stdout = true;  //�����cout
    ceres::Solver::Summary summary; //�Ż���Ϣ
    ceres::Solve(options, &problem, &summary);//��ʼ�Ż�
    //������
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "estimated a,b,c=";
    for (auto a : abc) std::cout << a << " ";
}
SLAM_LYJ_API void testOpenCV() {
    cv::Mat m = cv::imread(LYJOPT->sysHomePath + "other/down.png");
    cv::pyrDown(m, m);
    cv::pyrDown(m, m);
    cv::imshow("test", m);
    cv::waitKey();
}
SLAM_LYJ_API void testEigen() {
    Eigen::Vector3d x;
    x.setZero();
    std::cout << x << std::endl;
}
SLAM_LYJ_API void testVulkan() {
    VulkanImp vk;
    vk.run();
}
SLAM_LYJ_API void testKdTree() {
    using TYPE = Eigen::Vector3d;
    std::vector<TYPE> points;
    points.push_back(TYPE(0, 0, 0));
    points.push_back(TYPE(2, 2, 2));
    points.push_back(TYPE(9, 9, 9));
    points.push_back(TYPE(4, 4, 4));
    points.push_back(TYPE(6, 6, 6));
    //SLAM_LYJ_MATH::KdTree2d kdtree;
    SLAM_LYJ_MATH::KdTree<double, 3> kdtree;
    auto funcDist = [](const SLAM_LYJ_MATH::KdTree<double, 3>::Node* _node, const Eigen::Vector3d& _p)->double {
        if (_node->data.first.norm() < 0.1)
            return DBL_MAX;
        return (_p - _node->data.first).norm();
    };
    kdtree.build(points, funcDist);
    std::vector<TYPE> rets;
    std::vector<double> dists;
    TYPE p(-3, -3, -3);
    kdtree.search(p, 2, 1000, rets, dists);
    for (size_t i = 0; i < rets.size(); i++)
    {
        std::cout << i << ": \n" << rets[i] << std::endl;
    }
    return;
}
SLAM_LYJ_API void testPoint() {
    SLAM_LYJ_MATH::Point2d p2d(1, 3);
    std::cout << p2d[0] << std::endl;
    std::cout << p2d << std::endl;
    SLAM_LYJ_MATH::Point2d p2d2(3, 3);
    std::cout << p2d + p2d2 << std::endl;
    p2d += p2d2;
    std::cout << p2d << std::endl;
    std::cout << p2d.x() << std::endl;
    std::cout << p2d.y() << std::endl;
    p2d[0] = 3;
    std::cout << p2d.x() << std::endl;
    SLAM_LYJ_MATH::Point2d p2d3 = p2d + p2d2;
    SLAM_LYJ_MATH::Point2d xx = 2 * p2d * 2;
    std::cout << "xx: " << xx << std::endl;
    //SLAM_LYJ_MATH::Point2i p2i(1, 1);
    //std::cout << (SLAM_LYJ_MATH::Point2i)(++p2i) << std::endl;
}
SLAM_LYJ_API void testArchive() {
    SLAM_LYJ_MATH::Point2d p2d(3, 3);
    SLAM_LYJ_MATH::Point2d p2d2;
    //std::stringstream ss;
    //boost::archive::text_oarchive oar{ ss };
    //oar << p2d;
    //boost::archive::text_iarchive iar{ ss };
    //iar >> p2d2;
    std::string binFile = "test.dat";
    std::ofstream ofs(binFile, std::ios::binary);
    if (!ofs)
        return;
    p2d.write_binary(ofs);
    ofs.close();
    std::ifstream ifs(binFile, std::ios::binary);
    if (!ifs)
        return;
    p2d2.read_binary(ifs);
    ifs.close();
    std::cout << p2d2 << std::endl;
}
SLAM_LYJ_API void testTensor() {
    //Eigen::Matrix<double, 3, 4> mat;
    //mat << 1.0, 2.0, 3.0, 6.0,
    //    4.0, 2.0, 6.0, 12.0,
    //    7.0, 3.0, 9.0, 18.0;
    //Eigen::Matrix3d Q;
    //Eigen::Matrix<double, 3, 4> R;
    //SLAM_LYJ_MATH::QRLYJ<3, 4> qr;
    //qr.QR(mat.data(), Q.data(), R.data());

    //Eigen::Matrix<double, 3, 3> mat;
    //mat << 1.0, 2.0, 3.0,
    //    4.0, 2.0, 6.0,
    //    7.0, 3.0, 9.0;
    //Eigen::Matrix3d Q;
    //Eigen::Matrix<double, 3, 3> R;
    //SLAM_LYJ_MATH::QRLYJ<3, 3> qr;
    //qr.QR(mat.data(), Q.data(), R.data());

    Eigen::Matrix<double, 4, 3> mat;
    mat << 1.0, 2.0, 3.0,
        4.0, 2.0, 6.0,
        7.0, 3.0, 9.0,
        7.0, 3.0, 9.0;
    Eigen::Matrix<double, 4, 4> Q;
    Eigen::Matrix<double, 4, 3> R;
    SLAM_LYJ_MATH::QRLYJ<4, 3> qr;
    qr.QR(mat.data(), Q.data(), R.data());
}
SLAM_LYJ_API void testCommonAlgorithm() {
	//Rodrigues
	auto R = SLAM_LYJ_MATH::Rodrigues2RotMatrix<double>(Eigen::Vector3d(1, 0, 0), 0.523);
	std::cout << R << std::endl;
    //�׳�
    uint64_t res1 = SLAM_LYJ_MATH::factorial(10, 9);
    std::cout << "�׳ˣ�"<<res1 << std::endl;
    //����
    uint64_t res2 = SLAM_LYJ_MATH::pernutateNum(10, 2);
    std::cout << "����:"<<res2 << std::endl;
    //���
    uint64_t res3 = SLAM_LYJ_MATH::selectNum(2, 1);
    std::cout << "���:"<<res3 << std::endl;
    std::vector<std::vector<int>> results;
    SLAM_LYJ_MATH::fullSelection(10, 10, results);
    for (int i = 0; i < (int)results.size(); ++i) {
        std::cout << i << ": ";
        for (int j = 0; j < (int)results[i].size(); ++j) {
            std::cout << results[i][j] << " ";
        }
        std::cout << std::endl;
    }
}
SLAM_LYJ_API void testRANSAC() {
    //ax+by+c=0
    std::vector<SLAM_LYJ_MATH::Point2d> points;
    int pSize = 10;
    for (int i = 0; i < pSize; ++i) {
        points.emplace_back(i, 2 * i + 1); //2x-y+1=0
    }
    points[0].x() += 0.5;
    points[2].y() += 1;
    Eigen::Vector3d params; //a b c
    int eveSize = 2;
    double errTh = 0.01;
    auto funcCalError = [&](const Eigen::Vector3d& _params, const SLAM_LYJ_MATH::Point2d& _p, double& _err)->bool {
        _err = _params[0] * _p[0] + _params[1] * _p[1] + _params[2];
        _err /= (_params.block(0, 0, 2, 1).norm());
        if (std::abs(_err) > errTh) {
            return false;
        }
        return true;
    };
    auto funcCalModule = [](const std::vector<const SLAM_LYJ_MATH::Point2d*>& _samples, Eigen::Vector3d& _params)->bool {
        _params[0] = _samples.at(0)->y() - _samples.at(1)->y();
        _params[1] = -1 * (_samples.at(0)->x() - _samples.at(1)->x());
        _params[2] = -1 * _params[0] * _samples.at(0)->x() - _params[1] * _samples.at(0)->y();
        return true;
    };
    double inlineRatioTh = 0.8;
    SLAM_LYJ_MATH::RANSAC<SLAM_LYJ_MATH::Point2d, double, Eigen::Vector3d> ransac(inlineRatioTh, eveSize);
    ransac.setCallBack(funcCalError, funcCalModule);
    std::vector<double> errs;
    std::vector<bool> inlines;
    double inlineRatio = ransac.run(points, errs, inlines, params);
    std::cout << "line2d: \n" << params << std::endl;
    if (errs.empty()) {
        std::cout << "fail" << std::endl;
        return;
    }
    for (int i = 0; i < pSize; ++i) {
        std::cout << "err: " << errs[i] << ", inline: " << (int)inlines[i] << std::endl;
    }
}
SLAM_LYJ_API void testLine() {
    SLAM_LYJ_MATH::Line3d line3d(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, -1));
    SLAM_LYJ_MATH::Line3d line3d2(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0));
    std::cout << SLAM_LYJ_MATH::Line3d::angleL2L(line3d, line3d2) << std::endl;
    std::cout << line3d.distP2L(Eigen::Vector3d(1,0,0)) << std::endl;
    return;
    SLAM_LYJ_MATH::Line2d line2d(Eigen::Vector2d(1, 0), Eigen::Vector2d(1, -1.71));
    std::cout << line2d << std::endl;
    std::cout << line2d.angle() << std::endl;
    std::cout << line2d.dist() << std::endl;
    SLAM_LYJ_MATH::Line2d line2d2(Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1));
    std::cout << line2d2 << std::endl;
    Eigen::Vector2d interP;
    if(SLAM_LYJ_MATH::Line2d::interestL2L(line2d, line2d2, interP))
        std::cout << interP << std::endl;
    Eigen::Vector2d p(1, 0);
    std::cout << line2d2.distP2L(p) << std::endl;
    std::cout << line2d2.dir() << std::endl;
    std::cout << line2d2.pointInL(p) << std::endl;
    std::cout << SLAM_LYJ_MATH::Line2d::angleL2L(line2d, line2d2) << std::endl;
    cv::Mat m(100, 100, CV_8UC1, cv::Scalar(0));
    std::vector<Eigen::Vector2i> ps;
    SLAM_LYJ_MATH::bresenhamLine(0, 0, 90, 50, ps);
    for (const auto& point : ps) {
        m.at<uchar>(cv::Point(point.x(), point.y())) = 255;
    }
    cv::imshow("bresenham", m);
    cv::waitKey();
}
SLAM_LYJ_API void testPlane() {
    SLAM_LYJ_MATH::Plane3d plane3d(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1));
    std::cout << plane3d << std::endl;
}
SLAM_LYJ_API void testPose() {
    Pose2D pose2D(1.57, Eigen::Vector2d(0,0));
    std::cout << pose2D.getR() << std::endl;
}
SLAM_LYJ_API void testCamera() {
    std::string path = "E:/SLAM_LYJ/config/PinholeCamera.txt";
    PinholeCmera cam(path);
    std::cout << cam << std::endl;
    Eigen::Vector3d P(10, 1, 5);
    Eigen::Vector2d p;
    cam.world2Image(P, p);
    std::cout << p << std::endl;
    p(0) = 512;
    p(1) = 512;
    cam.image2World(p, 1, P);
    std::cout << P << std::endl;
}
SLAM_LYJ_API void testTriangler() {
    TriangleOption option;
    Trianglerd triangler(option);
}
SLAM_LYJ_API void testOptimizer() {
    OptVarPoint3d pointVar(0);
    std::cout << pointVar << std::endl;
    Eigen::Vector3d p3d(2, 2, 2);
    Eigen::Vector3d p3dDet(1, 1, 1);
    pointVar.setData(p3d.data());
    std::cout << pointVar << std::endl;
    pointVar.update(p3dDet.data());
    std::cout << pointVar << std::endl;

    OptFactorP3d_P3d factor(0);
    std::cout << factor.getId() << std::endl;
    const auto vDims = factor.getVDims();
    for (const auto& vd : vDims) {
        std::cout << vd << std::endl;
    }
    std::cout << factor.getEDim() << std::endl;
    Eigen::Vector3d p3dOb(4, 4, 6);
    factor.setData(p3dOb.data());
    //std::vector<Eigen::Matrix3d> jac(1);
    std::vector<std::vector<double>> jac(1, std::vector<double>(9));
    double* jacPtr = (jac.data()->data());
    double** jacPP = &jacPtr;
    double w = 1;
    Eigen::Vector3d err;
    factor.calculateErrAndJac(err.data(), jacPP, w, &pointVar);
    //std::cout << jac[0] << std::endl;
    std::cout << jac[0][0] << std::endl;
    std::cout << jac[0][4] << std::endl;
    std::cout << jac[0][8] << std::endl;
}
SLAM_LYJ_API void testBitFlagVec() {
    SLAM_LYJ_MATH::BitFlagVec bfv(10);
    bfv.setFlag(5, true);
    std::cout << bfv[4] << std::endl;
}
SLAM_LYJ_API void testFrame() {
    cv::Mat m = cv::imread("F:/�о���/ǩ��.jpg", 0);
    cv::pyrDown(m, m);
    cv::pyrDown(m, m);
    cv::pyrDown(m, m);
    cv::GaussianBlur(m, m, cv::Size(3, 3), 10, 20);
    std::string camPath = "E:/SLAM_LYJ/config/PinholeCamera.txt";
    PinholeCmera cam(camPath);
    Frame frame(0, (CameraModule*)(&cam));

    //orb
    //cv::Ptr<cv::ORB> orb_ = cv::ORB::create();
    //std::vector<cv::KeyPoint> kps;
    //cv::Mat des;
    //orb_->detectAndCompute(m, cv::Mat(), kps, des);
    //cv::Mat sTmp;
    //SLAM_LYJ_DEBUGGER::drawFeatures(m, kps, sTmp, cv::Scalar(255), 5, 2);
    //cv::imshow("111", sTmp);
    //cv::waitKey();
    ORBExtractor::Option optORB;
    ORBExtractor orb(optORB);
    //Frame frame(0, m, (ExtractorAbr*)&orb, (CameraModule*)(&cam));
    orb.extract(m, frame);
    cv::Mat im2ShowORB;
    SLAM_LYJ_DEBUGGER::drawFeatures(m, frame.getKeyPoints(), im2ShowORB, cv::Scalar(255), 5, 2);
    cv::imshow("orb", im2ShowORB);
    cv::waitKey();

    //lsd
    LSDExtractor::Option optLSD;
    LSDExtractor lsd(optLSD);
    lsd.extract(m, frame);
    cv::Mat im2ShowLSD;
    SLAM_LYJ_DEBUGGER::drawLineFeatures(m, frame.getLines(), im2ShowLSD, cv::Scalar(255), 1);
    cv::imshow("line", im2ShowLSD);

    //sift
    SIFTExtractor::Option optSIFT;
    SIFTExtractor sift(optSIFT);
    sift.extract(m, frame);
    cv::Mat im2ShowSIFT;
    SLAM_LYJ_DEBUGGER::drawFeatures(m, frame.getKeyPoints(), im2ShowSIFT, cv::Scalar(255));
    cv::imshow("sift", im2ShowSIFT);
    cv::waitKey();

    //canny
    CannyExtractor::Option optCanny;
    CannyExtractor canny(optCanny);
    canny.extract(m, frame);
    cv::Mat im2ShowCanny;
    SLAM_LYJ_DEBUGGER::drawEdgeFeatures(m, frame.getEdgeFeatures(), im2ShowCanny, cv::Scalar(255));
    cv::imshow("canny", im2ShowCanny);
    cv::waitKey();
}
SLAM_LYJ_API void testIncrementalAgvAndVar() {
    std::vector<double> v{ 0,1,2,3,4,5,6 };
    double agv = 0;
    double var = 0;
    for (int i = 0; i < (int)v.size(); ++i) {
        SLAM_LYJ_MATH::calculateAgvAndVar<double, 1>(i, &v[i], &agv, &var);
        std::cout << "agv: " << agv << " var: " << var << std::endl;
    }
}
SLAM_LYJ_API void testGrid() {
    std::vector<Eigen::Vector2d> points;
    for (int i = 0; i < 100; ++i) {
        points.push_back(Eigen::Vector2d(i, i));
    }
    //SLAM_LYJ_MATH::Grid<double, 2> grid2Dd(10, points);
    SLAM_LYJ_MATH::Grid<double, 2> grid2Dd(10, std::vector<int>{100, 100}, Eigen::Vector2d(50,50), points);
    const auto& inds = grid2Dd.getIndsNear(Eigen::Vector2d(20, 20));
    for (const auto& ind : inds) {
        std::cout << points[ind] << std::endl << std::endl;
    }
}
SLAM_LYJ_API void testSTLPlus() {
    std::string folderPath = "E:/SLAM_LYJ/build/example/Release/testSTLPlus";
    //boost::filesystem::create_directories(folderPath);
    //boost::filesystem::remove(folderPath);
    //boost::filesystem::exi
    if (stlplus::folder_exists(folderPath))
        stlplus::folder_delete(folderPath, true);
    stlplus::folder_create(folderPath);
    for (int i = 0; i < 10; ++i) {
        std::ofstream ifs(folderPath + "/" + std::to_string(i) + ".txt");
        ifs.close();
    }
    std::vector<std::string> files = stlplus::folder_files(folderPath);
    int fSize = (int)files.size();
    std::cout << "Number of files in " << folderPath << " is: " << fSize << std::endl;
    for (int i = 0; i < fSize; ++i) {
        std::cout << files[i] << std::endl;
    }
}
SLAM_LYJ_API void testBuffer(){
    {
        LYJBuffer buffer;
        Pose3D* poseCache = buffer.alloc<Pose3D>("pose");
        if(poseCache)
            std::cout << *poseCache << std::endl;
        Pose2D* poseCache2 = buffer.get<Pose2D>("pose2");
        if (!poseCache2)
            std::cout << "no such cache in buffer." << std::endl;
        Pose2D* poseCache3 = buffer.alloc<Pose2D>("pose");
        if (!poseCache3)
            std::cout << "alloc cache in buffer failed." << std::endl;
    }
}
SLAM_LYJ_API void testGlobalOption() {
    //GlobalInnerOption* gOpt = GlobalInnerOption::get();
    LYJOPT->sysName = "LYJ_SLAM";
    std::cout << LYJOPT->sysName << std::endl;
}

SLAM_LYJ_API int getVersion(){
    return SYS_VERSION;
}

SLAM_LYJ_API void testThreadPool(){
    SLAM_LYJ_MATH::ThreadPool threadPool(5);
    for(int i=0;i<100;++i)
    {
        threadPool.addTask([i]{ std::cout<< "hello " << i << std::endl;});
    }
    threadPool.finish();
}





NSP_SLAM_LYJ_END