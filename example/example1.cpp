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
    QT_LYJ::testQT();
    //SLAM_LYJ::testPatchMatch();
    //SLAM_LYJ::testDiffuser();
    //SLAM_LYJ::testPolarGrid();
    return 0;
}