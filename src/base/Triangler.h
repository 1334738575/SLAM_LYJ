#ifndef SLAM_LYJ_TRIANGLE_H
#define SLAM_LYJ_TRIANGLE_H

#include "base/Base.h"
#include "base/Pose.h"
#include "base/CameraModule.h"

NSP_SLAM_LYJ_BEGIN

struct TriangleOption
{
    double ransacPriorInlineRatio = 0.5;
    double ransacDstRatioTh = 0.99;
    double ransacStopRatio = 1;
    int ransacMinInlineNum = 2;
    int ransacMaxItrtNum = 20;
    double inlineErrTh = 5;
};

template<typename TYPE>
class Triangler
{
private:
    /* data */
    TriangleOption m_option;
public:

    using TemVec2 = Eigen::Matrix<TYPE, 2, 1>;
    using TemVec3 = Eigen::Matrix<TYPE, 3, 1>;
    using TemVec4 = Eigen::Matrix<TYPE, 4, 1>;
    using TemMat22 = Eigen::Matrix<TYPE, 2, 2>;
    using TemMat23 = Eigen::Matrix<TYPE, 2, 3>;
    using TemMat33 = Eigen::Matrix<TYPE, 3, 3>;
    using TemMat34 = Eigen::Matrix<TYPE, 3, 4>;
    using TemMat44 = Eigen::Matrix<TYPE, 4, 4>;


    Triangler(const TriangleOption& _option)
        :m_option(_option)
    {}
    ~Triangler() {}

    bool runDirect(const std::vector<TemVec2>& _uvs, const std::vector<TemMat34>& _Tcws, const std::vector<TemMat33>& _Ks,
        TemVec3& _Pw) {
        //construct
        TemMat44 A = TemMat44::Zero();
        for (size_t i = 0; i < _uvs.size(); i++)
        {
            TemVec3 point = _Ks[i].inverse() * TemVec3{ _uvs[i](0), _uvs[i](1), 1 };
            point.normalize();
            const TemMat34 term = _Tcws[i] - point * point.transpose() * _Tcws[i];
            A += term.transpose() * term;
        }
        //solve
        Eigen::SelfAdjointEigenSolver<TemMat44> eigen_solver(A);
        _Pw = eigen_solver.eigenvectors().col(0).hnormalized();
        //check
        for (size_t i = 0; i < _uvs.size(); ++i) {
            TemVec3 Pc = _Tcws[i].block(0, 0, 3, 3) * _Pw + _Tcws[i].block(0, 3, 3, 1);
            if (Pc(2) <= 0)
                return false;
        }
        return true;
    }
    bool runDirect(const std::vector<TemVec2>& _uvs, const std::vector<Pose3D>& _Tcws, const std::vector<PinholeCmera>& _cams,
        TemVec3& _Pw) {
        //construct
        TemMat44 A = TemMat44::Zero();
        for (size_t i = 0; i < _uvs.size(); i++)
        {
            TemVec3 point;
            _cams[i].image2World(_uvs[i], 1, point);
            point.normalize();
            TemMat34 Tcw;
            Tcw.block(0, 0, 3, 3) = _Tcws[i].getR();
            Tcw.block(0, 3, 3, 1) = _Tcws[i].gett();
            const TemMat34 term = Tcw - point * point.transpose() * Tcw;
            A += term.transpose() * term;
        }
        //solve
        Eigen::SelfAdjointEigenSolver<TemMat44> eigen_solver(A);
        _Pw = eigen_solver.eigenvectors().col(0).hnormalized();
        //check
        for (size_t i = 0; i < _uvs.size(); ++i) {
            TemVec3 Pc = _Tcws[i] * _Pw;
            if (Pc(2) <= 0)
                return false;
        }
        return true;
    }
    bool runWithRANSAC(const std::vector<TemVec2>& _uvs, const std::vector<Pose3D>& _Tcws, const std::vector<PinholeCmera>& _cams,
        TemVec3& _Pw) {
        int obSize = (int)_uvs.size();
        //typedef std::function<bool(const MODULE& mdl, const DATATYPE& data, ERRORTYPE& err)> FuncCalError;
        //typedef std::function<bool(const std::vector<const DATATYPE*>& samples, MODULE& mdl)> FuncCalModule;
        std::vector<int> datas; //索引作为数据输入
        for (int i = 0; i < obSize; ++i) {
            datas.emplace_back(i);
        }
        auto funcCalError = [&](const TemVec3& _Pw, const int& _data, TYPE& _err)->bool {
            TemVec3 Pc = _Tcws[_data] * _Pw;
            TemVec2 uv;
            _cams[_data].world2Image(Pc.cast<double>(), uv.cast<double>());
            _err = (_uvs[_data] - uv).norm();
            if (_err > static_cast<TYPE>(m_option.inlineErrTh))
                return false;
            return true;
        };
        auto funcCalModule = [&](const std::vector<const int*>& _samples, TemVec3& _Pw)->bool {
            //construct
            TemMat44 A = TemMat44::Zero();
            for (size_t i = 0; i < _samples.size(); i++)
            {
                TemVec3 point;
                _cams[_samples[i]].image2World(_uvs[_samples[i]], 1, point);
                point.normalize();
                TemMat34 Tcw;
                Tcw.block(0, 0, 3, 3) = _Tcws[_samples[i]].getR();
                Tcw.block(0, 3, 3, 1) = _Tcws[_samples[i]].gett();
                const TemMat34 term = Tcw - point * point.transpose() * Tcw;
                A += term.transpose() * term;
            }
            //solve
            Eigen::SelfAdjointEigenSolver<TemMat44> eigen_solver(A);
            _Pw = eigen_solver.eigenvectors().col(0).hnormalized();
            //check
            for (size_t i = 0; i < _uvs.size(); ++i) {
                TemVec3 Pc = _Tcws[_samples[i]] * _Pw;
                if (Pc(2) <= 0)
                    return false;
            }
            return true;
        };
        SLAM_LYJ_MATH::RANSAC<int, TYPE, TemVec3> ransac(m_option.ransacPriorInlineRatio, m_option.ransacMinInlineNum, m_option.ransacMaxItrtNum);
        ransac.setCallBack(funcCalError, funcCalModule);
        std::vector<TYPE> errs;
        std::vector<bool> bInlines;
        double inlineRatio = ransac.run(datas, errs, bInlines, _Pw);
        return true;
    }
};

typedef Triangler<float> Trianglerf;
typedef Triangler<double> Trianglerd;



NSP_SLAM_LYJ_END

#endif //SLAM_LYJ_TRIANGLE_H