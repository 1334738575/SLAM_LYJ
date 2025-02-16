#ifndef SLAM_LYJ_CAMERAMODULE_H
#define SLAM_LYJ_CAMERAMODULE_H

#include "Base.h"

NSP_SLAM_LYJ_BEGIN

enum CameraType {
    DEFAULT,
    PINHOLE,
    FISHEYE
};
class CameraModule : public BaseLYJ
{
protected:
    /* data */
    CameraType type = DEFAULT;
    std::vector<double> params;
public:
    CameraModule(const CameraType _type, const std::vector<double>& _params)
        :type(_type), params(_params) {}
    ~CameraModule() {}

    //物理到图像
    virtual void world2Image(const Eigen::Vector3d& _P, Eigen::Vector2d& _p) const = 0;
    virtual void world2Image(const Eigen::Vector3d& _P, double& _u, double& _v) const = 0;
    //图像到物理
    virtual void image2World(const Eigen::Vector2d& _p, const double _d, Eigen::Vector3d& _P) const = 0;
    virtual void image2World(const double _u, const double _v, const double _d, Eigen::Vector3d& _P) const = 0;
    virtual void image2World(const Eigen::Vector3d& _p, Eigen::Vector3d& _P) const = 0; //u v d
    //内参矩阵
    virtual Eigen::Matrix3d getK() const = 0;
    virtual const double& fx() const = 0;
    virtual const double& fy() const = 0;
    virtual const double& cx() const = 0;
    virtual const double& cy() const = 0;
    inline const CameraType getType() const {
        return type;
    }
    inline const int wide() const {
        return static_cast<int>(params[0]);
    }
    inline const int height() const {
        return static_cast<int>(params[1]);
    }
    bool inImage(const int _u, const int _v) {
        if (_u < 0 || _u >= wide())
            return false;
        if (_v < 0 || _v >= height())
            return false;
        return true;
    }
};


class PinholeCmera : public CameraModule
{
public:
    PinholeCmera(const CameraType _type, const std::vector<double>& _params)
        :CameraModule(_type, _params)
    {}
    PinholeCmera(const std::string& _path)
        :CameraModule(DEFAULT, std::vector<double>())
    {
        std::ifstream f(_path);
        if (!f.is_open()) {
            std::cout << "Read pinhole camera file fail!" << std::endl;
            return;
        }
        params.resize(4);
        int t;
        f >> t >> params[2] >> params[3] >> params[4] >> params[5];
        type = t==1 ? PINHOLE : DEFAULT;
        f.close();
    }
    ~PinholeCmera() {}

    //物理到图像
    //override
    void world2Image(const Eigen::Vector3d& _P, Eigen::Vector2d& _p) const {
        world2Image(_P, _p(0), _p(1));
    }
    //override
    void world2Image(const Eigen::Vector3d& _P, double& _u, double& _v) const {
        _u = params[2] * _P(0) / _P(2) + params[4];
        _v = params[3] * _P(1) / _P(2) + params[5];
    }
    //图像到物理
    //override
    void image2World(const Eigen::Vector2d& _p, const double _d, Eigen::Vector3d& _P) const {
        image2World(_p(0), _p(1), _d, _P);
    }
    //override
    void image2World(const double _u, const double _v, const double _d, Eigen::Vector3d& _P) const {
        _P(0) = 1.0 / params[2] * (_u - params[4]);
        _P(1) = 1.0 / params[3] * (_v - params[5]);
        _P(2) = _d;
    }
    //override
    void image2World(const Eigen::Vector3d& _p, Eigen::Vector3d& _P) const {
        image2World(_p(0), _p(1), _p(2), _P);
    }
    //内参矩阵
    //override
    Eigen::Matrix3d getK() const {
        Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
        K(0, 0) = params[2];
        K(1, 1) = params[3];
        K(0, 2) = params[4];
        K(1, 2) = params[5];
        return K;
    }
    const double& fx() const {
        return params[2];
    }
    const double& fy() const {
        return params[3];
    }
    const double& cx() const {
        return params[4];
    }
    const double& cy() const {
        return params[5];
    }


    void write_binary(std::ofstream& os) override {
        os.write(reinterpret_cast<const char*>(&type), sizeof(int) * 1);
        os.write(reinterpret_cast<const char*>(params.data()), sizeof(double) * 4);
    }
    void read_binary(std::ifstream& is) override {
        is.read(reinterpret_cast<char*>(&type), sizeof(int) * 1);
        is.read(reinterpret_cast<char*>(params.data()), sizeof(double) * 4);
    }
    friend std::ostream& operator<< (std::ostream& os, const PinholeCmera& cls) {
        std::cout << cls.getType() << std::endl;
        std::cout << cls.getK();
        return os;
    }

private:

};



NSP_SLAM_LYJ_END

#endif //SLAM_LYJ_CAMERAMODULE_H