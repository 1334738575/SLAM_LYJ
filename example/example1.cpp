#include <iostream>
#include "SLAM_LYJ.h"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/line_descriptor.hpp>
#include <ceres/ceres.h>
#include <vulkan/vulkan.hpp>
#include <glfw/glfw3.h>
 using namespace std;


template<typename T>
void testTemplate(const T* _a) {
    std::cout << _a << std::endl;
}

class CURVE_FITTING_COST {
public:
    CURVE_FITTING_COST(double x, double y) :_x(x), _y(y) {}
    template<typename T>
    bool operator()(const T* const abc, T* residual) const
    {
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
    }
    const double _x, _y; //x,y数据
};
void testCeres() {
    double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值
    //在待拟合曲线上均匀的生成100个数据点，加上白噪声，作为待拟合曲线
    int N = 100;                                 // 数据点
    double w_sigma = 1.0;                        // 噪声Sigma值
    double inv_sigma = 1.0 / w_sigma;
    cv::RNG rng;                                 // OpenCV随机数产生器
    vector<double> x_data, y_data;      // 数据
    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ae * x * x + be * x + ce) + rng.gaussian(w_sigma * w_sigma));
    }
    double abc[3] = { ae,be,ce };
    //二、构建最小二乘问题
    ceres::Problem problem;
    for (int i = 0; i < N; ++i)
    {
        //向问题中添加误差项，使用自动求导，模板参数：误差类型，输出维度（残差的维度），输入维度（待优化变量的维度），维数要与前面的struct中一致，
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(new CURVE_FITTING_COST(x_data[i], y_data[i])),
            nullptr,  //核函数，这里不需要，故为空
            abc      //待估计参数
        );
    }
    //三、配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;//增量方程如何求解
    options.minimizer_progress_to_stdout = true;  //输出到cout
    ceres::Solver::Summary summary; //优化信息
    ceres::Solve(options, &problem, &summary);//开始优化
    //输出结果
    cout << summary.BriefReport() << endl;
    cout << "estimated a,b,c=";
    for (auto a : abc) cout << a << " ";
}

void testOpenCV() {
    cv::Mat m = cv::imread("F:/work/2023年照片/20230327布置会/IMG_9179[1](1).png");
    cv::pyrDown(m, m);
    cv::pyrDown(m, m);
    std::vector<cv::KeyPoint> kps;
    cv::Ptr<cv::xfeatures2d::SiftFeatureDetector> sift = cv::xfeatures2d::SiftFeatureDetector::create();
    sift->detect(m, kps);
    cv::Ptr<cv::LineSegmentDetector> lsd = cv::createLineSegmentDetector();
    std::vector<cv::Vec4f> vecLines;
    cv::Mat gray;
    cv::cvtColor(m, gray, cv::COLOR_RGB2GRAY);
    lsd->detect(gray, vecLines);
    cv::Mat outml = m.clone();
    lsd->drawSegments(outml, vecLines);
    cv::Mat outm;
    cv::drawKeypoints(m, kps, outm);
    cv::imshow("test2", m);
    cv::imshow("line", outml);
    cv::imshow("kp", outm);
    cv::waitKey();
}

void testEigen() {
    Eigen::Vector2d x;
    x.setZero();
    std::cout << x << std::endl;
    Eigen::Matrix2d m;
    m << 0, 1, -1, 0;
    std::cout << m << std::endl;
}

void testVulkan() {
    VkInstance instance;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device;
    VkApplicationInfo appInfo{};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "Hello Vulkan";
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName = "No Engine";
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.apiVersion = VK_API_VERSION_1_0;

    VkInstanceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    createInfo.pApplicationInfo = &appInfo;

    uint32_t glfwCnt = 0;
    const char** glfwExtensions;
    glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwCnt);

    createInfo.enabledExtensionCount = glfwCnt;
    createInfo.ppEnabledExtensionNames = glfwExtensions;
    createInfo.enabledLayerCount = 0;

    if (vkCreateInstance(&createInfo, nullptr, &instance) != VK_SUCCESS) {
        throw std::runtime_error("failed to create instance!");
    }
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
    //std::vector<int> dd{ 1,2,2,34,5 };
    //std::vector<int> dd2;
    //dd2 = std::move(dd);
    //return;
    //int a = 1;
    //a <<= 31;
    //a >>= 31;
    //std::cout << a << std::endl; // -1,右移保持符号位
    //return;
    ////std::pair<int, int>* pp = new std::pair<int, int>[10];
    //Eigen::SparseMatrix<double> sm;
    //sm.resize(10, 10);
    //Eigen::Matrix<double, 10, 10> dm = sm.toDense();
    //Eigen::SparseMatrix<double> smb = sm.block(0, 0, 2, 2);
    //Eigen::Matrix2d dmb = smb.toDense();
    //std::cout << "dm: \n" << dm << std::endl;
    //std::cout << "dmb: \n" << dmb << std::endl;
    //return;
    //class testPtr {
    //public:
    //    testPtr(double* _ptr, int _size) :m_ptr(_ptr), m_size(_size) {}
    //    testPtr(testPtr&& _obj) noexcept {
    //        m_ptr = _obj.m_ptr;
    //        m_size = _obj.m_size;
    //        _obj.m_ptr = nullptr;
    //        _obj.m_size = -1;
    //    }
    //    testPtr(std::vector<double>&& _obj) {
    //        m_ptr = _obj.data();
    //        m_size = (int)_obj.size();
    //        _obj.swap(std::vector<double>());
    //    }
    //    ~testPtr() {
    //        if (m_ptr) {
    //            delete m_ptr;
    //            std::cout << "release" << std::endl;
    //        }
    //    }
    //private:
    //    double* m_ptr = nullptr;
    //    int m_size = -1;
    //};
    //std::vector<double> data{ 1,2,23,45,776 };
    //{
    //    //testPtr tPtr(data.data(), (int)data.size());
    //    testPtr tPtr(std::move(data));
    //    {
    //        testPtr tPtr2 = std::move(tPtr);
    //    }
    //}
    //std::cout << data.size() << std::endl;
    //std::cout << data[0] << std::endl;
}

int main(int argc, char* argv[]){
    //test();
    //testCeres();
    //testOpenCV();
    //testEigen();
    //testVulkan();
    //testGrowTemplate();
    std::cout<<"Hello SLAM_LYJ!" <<std::endl;
    std::cout<< "Current version is: " << SLAM_LYJ::getVersion() << std::endl;
    //SLAM_LYJ::testCeres();
    //SLAM_LYJ::testEigen();
    //SLAM_LYJ::testOpenCV();
    //SLAM_LYJ::testThreadPool();
    SLAM_LYJ::testVulkan();
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
    return 0;
}