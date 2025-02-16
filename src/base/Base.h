#ifndef SLAM_LYJ_BASE_H
#define SLAM_LYJ_BASE_H

//stl
#include <iostream>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>
#include <queue>
#include <deque>
#include <list>
#include <stack>
#include <fstream>
#include <memory>
#include <random>
#include <algorithm>

//eigen
#include <Eigen/Eigen>
#include <Eigen/Core>


typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;
typedef Eigen::Matrix<float, 3, 4> Matrix3x4f;
#define PI 3.1415926
#define RAD2DEG 57.32484076
#define DEG2RAD 0.01744444

// //opencv
// #include <opencv2/opencv.hpp>

//boost
#ifdef USEBOOST
//#include <boost/serialization/vector.hpp>
//#include <boost/serialization/map.hpp>
//#include <boost/serialization/list.hpp>
//#include <boost/serialization/set.hpp>
//#include <boost/serialization/unordered_map.hpp>
//#include <boost/serialization/unordered_set.hpp>
//#include <boost/serialization/queue.hpp>
//#include <boost/serialization/deque.hpp>
//#include <boost/serialization/base_object.hpp>
#endif //USEBOOST


//export
#ifdef WIN32
    #ifdef _MSC_VER
        #define SLAM_LYJ_API __declspec(dllexport)
    #else
        #define SLAM_LYJ_API
    #endif
#else
    #define SLAM_LYJ_API
#endif


//namespace
#define NSP_SLAM_LYJ_BEGIN namespace SLAM_LYJ {
#define NSP_SLAM_LYJ_END }

#define NSP_SLAM_LYJ_MATH_BEGIN namespace SLAM_LYJ { \
                                    namespace SLAM_LYJ_MATH {
#define NSP_SLAM_LYJ_MATH_END } \
                            }


#define NSP_SLAM_LYJ_DEBUGGER_BEGIN namespace SLAM_LYJ { \
                                    namespace SLAM_LYJ_DEBUGGER {
#define NSP_SLAM_LYJ_DEBUGGER_END } \
                            }

//base class
NSP_SLAM_LYJ_BEGIN
class BaseLYJ
{
private:
    /* data */
public:
    BaseLYJ(/* args */){};
    ~BaseLYJ(){};

    virtual void write_binary(std::ofstream& os) = 0;
    virtual void read_binary(std::ifstream& os) = 0;
};

class LYJBuffer
{
public:
    class Src
    {
    public:
        Src() {}
        ~Src() { 
            //std::cout << "release Src." << std::endl; 
        }

        virtual void forInherit() {}
    };
    template<typename T>
    class Property : public Src
    {
    public:
        Property() {
        }
        ~Property() { 
            //std::cout << "release Property." << std::endl; 
        }
        T obj_;
    };
    LYJBuffer() {};
    ~LYJBuffer() {};

    template<typename T>
    T* get(const std::string _name) {
        if (cache_.count(_name) == 0 || cache_[_name].second != typeid(T).name())
            return nullptr;
        Property<T>* pro = dynamic_cast<Property<T>*>(cache_[_name].first.get());
        return &pro->obj_;
    }

    template<typename T>
    T* alloc(const std::string _name) {
        auto ret = get<T>(_name);
        if (ret)
            return ret;
        if (cache_.count(_name) != 0)
            return nullptr;
        Property<T>* pro = new Property<T>();
        cache_[_name].first.reset((Src*)pro);
        cache_[_name].second = typeid(T).name();
        return &pro->obj_;
    }


private:
    std::unordered_map<std::string, std::pair<std::shared_ptr<Src>, std::string>> cache_;
};

NSP_SLAM_LYJ_END

#endif //SLAM_LYJ_BASE_H