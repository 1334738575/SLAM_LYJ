#include <iostream>
#include "SLAM_LYJ.h"
#include "QT_LYJ.h"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/line_descriptor.hpp>
#include <ceres/ceres.h>
#include <vulkan/vulkan.hpp>
#include <glfw/glfw3.h>

 using namespace std;


#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

 using namespace Eigen;
 using namespace std;

 template <typename T>
 void AngleAxisToRotationMatrix(const T* angle_axis, T* R) {
     using EigenVector = Eigen::Matrix<T, 3, 1>;
     using EigenMap = Eigen::Map<Eigen::Matrix<T, 3, 3, Eigen::RowMajor>>;

     Eigen::AngleAxis<T> aa(EigenVector(angle_axis).norm(),
         EigenVector(angle_axis).normalized());
     Matrix<T, 3, 3> r = aa.toRotationMatrix();
     memcpy((void*)R, (void*)r.data(), sizeof(T) * 9);
     //EigenMap(R) = aa.toRotationMatrix();
 }

 // 重投影误差模型
 struct ReprojectionError {
     ReprojectionError(const Vector2d& observed_uv, const Matrix3d& K)
         : observed_uv(observed_uv), K(K) {
     }

     template <typename T>
     bool operator()(const T* const camera_pose, // 6D: [angle_axis(3), translation(3)]
         const T* const point3d,    // 3D点坐标
         T* residual) const {
         // 解析旋转向量和平移
         const T* angle_axis = camera_pose;
         const T* translation = camera_pose + 3;

         // 将旋转向量转换为旋转矩阵
         Matrix<T, 3, 3> R;
         AngleAxisToRotationMatrix<T>(angle_axis, R.data());

         // 将3D点转换到相机坐标系
         Matrix<T, 3, 1> p_cam = R * Matrix<T, 3, 1>(point3d[0], point3d[1], point3d[2])
             + Matrix<T, 3, 1>(translation[0], translation[1], translation[2]);

         // 投影到图像平面
         p_cam /= p_cam[2];
         Matrix<T, 3, 1> uv_hom = K.cast<T>() * p_cam;

         // 计算残差
         residual[0] = uv_hom[0] - T(observed_uv.x());
         residual[1] = uv_hom[1] - T(observed_uv.y());
         return true;
     }

     const Vector2d observed_uv;
     const Matrix3d K;
 };

 // 生成测试数据（与之前代码保持一致）
 void generateTestData(vector<Vector3d>& points3d,
     vector<pair<Vector2d, Vector2d>>& matches,
     Matrix3d& K,
     Matrix4d& T1,
     Matrix4d& T2) {
     // 相机内参
     K << 500, 0, 320,
         0, 500, 240,
         0, 0, 1;

     // 生成相机位姿
     T1 = Matrix4d::Identity();

     T2 = Matrix4d::Identity();
     T2.block<3, 3>(0, 0) = AngleAxisd(0.2, Vector3d::UnitX()).toRotationMatrix()
         * AngleAxisd(0.1, Vector3d::UnitY()).toRotationMatrix();
     T2.block<3, 1>(0, 3) = Vector3d(0.5, -0.2, 0.3);

     // 生成3D点
     const int num_points = 50;
     points3d.resize(num_points);
     for (int i = 0; i < num_points; ++i) {
         points3d[i] = Vector3d::Random().cwiseProduct(Vector3d(1, 1, 3))
             + Vector3d(0, 0, 2);
     }

     // 生成匹配点对
     auto project = [&K](const Vector3d& p, const Matrix4d& T) {
         Matrix3d R = T.block<3, 3>(0, 0);
         Vector3d t = T.block<3, 1>(0, 3);
         Vector3d p_cam = R * p + t;
         Vector3d uv = K * p_cam;
         return Vector2d(uv[0] / uv[2], uv[1] / uv[2]);
         };

     matches.clear();
     for (const auto& p : points3d) {
         matches.emplace_back(project(p, T1), project(p, T2));
     }
 }

 int main2() {
     // 生成测试数据
     vector<Vector3d> points3d_true;
     vector<pair<Vector2d, Vector2d>> matches;
     Matrix3d K;
     Matrix4d T1_true, T2_true;
     generateTestData(points3d_true, matches, K, T1_true, T2_true);

     // 添加噪声创建初始猜测
     vector<Vector3d> points3d_est = points3d_true;
     for (auto& p : points3d_est) {
         p += Vector3d::Random() * 0.1; // 添加0.1m噪声
     }

     // 转换T2为旋转向量+平移的初始猜测
     Vector3d angle_axis_est =
         AngleAxisd(Matrix3d(T2_true.block<3, 3>(0, 0))).angle()
         * AngleAxisd(Matrix3d(T2_true.block<3, 3>(0, 0))).axis();
     Vector3d translation_est = T2_true.block<3, 1>(0, 3) + Vector3d::Random() * 0.05;
     double T2_est[6] = { angle_axis_est[0], angle_axis_est[1], angle_axis_est[2],
                        translation_est[0], translation_est[1], translation_est[2] };

     // 构建优化问题
     ceres::Problem problem;

     // 添加所有残差项
     std::vector<double> T1_fixed(6, 0); // 固定第一个相机位姿
     T1_fixed[0] = 1;
     for (size_t i = 0; i < matches.size(); ++i) {
         // 第一相机的观测（固定T1）
         {
             ceres::CostFunction* cost_function =
                 new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(
                     new ReprojectionError(matches[i].first, K));
             problem.AddResidualBlock(cost_function, nullptr, T1_fixed.data(),
                 points3d_est[i].data());
             problem.SetParameterBlockConstant(T1_fixed.data()); // 固定第一个相机
         }

         // 第二相机的观测
         {
             ceres::CostFunction* cost_function =
                 new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(
                     new ReprojectionError(matches[i].second, K));
             problem.AddResidualBlock(cost_function, nullptr, T2_est,
                 points3d_est[i].data());
         }
     }

     // 配置优化器
     ceres::Solver::Options options;
     options.linear_solver_type = ceres::SPARSE_SCHUR;
     options.minimizer_progress_to_stdout = true;
     options.max_num_iterations = 100;

     // 运行优化
     ceres::Solver::Summary summary;
     ceres::Solve(options, &problem, &summary);
     std::cout << summary.FullReport() << std::endl;

     // 结果验证
     auto evaluateError = [&]() {
         double total_error = 0;
         for (size_t i = 0; i < points3d_est.size(); ++i) {
             Vector3d p = points3d_est[i];
             auto fff = ReprojectionError(matches[i].second, K);
             Vector2d uv;
             fff(T2_est, p.data(), uv.data());
             total_error += uv.norm();
         }
         return total_error / points3d_est.size();
         };

     std::cout << "\n优化结果验证:"
         << "\n平均重投影误差: " << evaluateError() << " pixels"
         << "\n平移误差: " << (Map<Vector3d>(T2_est + 3) - T2_true.block<3, 1>(0, 3)).norm()
         << " meters"
         << "\n3D点平均误差: " << [&]() {
         double err = 0;
         for (size_t i = 0; i < points3d_est.size(); ++i)
             err += (points3d_est[i] - points3d_true[i]).norm();
         return err / points3d_est.size();
         }() << " meters" << std::endl;

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
    //std::vector<int> dd{ 1,2,2,34,5 };
    //std::vector<int> dd2;
    //dd2 = std::move(dd);
    //return;
    //int a = 1;
    //a <<= 31;
    //a >>= 31;
    //std::cout << a << std::endl; // -1,���Ʊ��ַ���λ
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
    //testGrowTemplate();
    main2();
    return 0;
    std::cout<<"Hello SLAM_LYJ!" <<std::endl;
    std::cout<< "Current version is: " << SLAM_LYJ::getVersion() << std::endl;
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
    //QT_LYJ::testQT();
    //SLAM_LYJ::testPatchMatch();
    //SLAM_LYJ::testDiffuser();
    //SLAM_LYJ::testPolarGrid();
    //QT_LYJ::testOpenGLOnly();
    //SLAM_LYJ::testOcTreeAndQuadTree();
    //SLAM_LYJ::testCUDA();
    return 0;
}