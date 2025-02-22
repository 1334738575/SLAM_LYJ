#ifndef SLAM_LYJ_POSE_H
#define SLAM_LYJ_POSE_H

#include "Base.h"
#include "common/CommonAlgorithm.h"


NSP_SLAM_LYJ_BEGIN

class Pose3D : public BaseLYJ
{
private:
    /* data */
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
public:
    Pose3D(/* args */) {}
    Pose3D(const Eigen::Matrix3d& _R, const Eigen::Vector3d& _t)
        :R(_R), t(_t)
    {}
    Pose3D(const Matrix3x4d& _T)
        :R(_T.block(0, 0, 3, 3)), t(_T.block(0, 3, 3, 1))
    {}
    Pose3D(const Eigen::Vector3d& _c1, const Eigen::Vector3d& _n1,
        const Eigen::Vector3d& _c2, const Eigen::Vector3d& _n2)
    {
		Eigen::Vector3d axis = _n1.cross(_n2);
		double angle = std::acos(_n1.dot(_n2));
        Eigen::Matrix3d R = SLAM_LYJ_MATH::Rodrigues2RotMatrix(axis, angle);
		Eigen::Vector3d t = _c2 - R * _c1;
		this->setR(R);
		this->sett(t);
    }
    ~Pose3D(){}

    void write_binary(std::ofstream& os) {
        os.write(reinterpret_cast<const char*>(R.data()), sizeof(double) * 9);
        os.write(reinterpret_cast<const char*>(t.data()), sizeof(double) * 3);
    }
    void read_binary(std::ifstream& is) {
        is.read(reinterpret_cast<char*>(R.data()), sizeof(double) * 9);
        is.read(reinterpret_cast<char*>(t.data()), sizeof(double) * 3);
    }
    friend std::ostream& operator<< (std::ostream& os, const Pose3D cls) {
        std::cout << cls.getR() << std::endl;
        std::cout << cls.gett();
        return os;
    }

    const Eigen::Matrix3d& getR() const {
        return R;
    }
    const Eigen::Vector3d& gett() const {
        return t;
    }
    void setR(const Eigen::Matrix3d& _R) {
        R = _R;
    }
    void sett(const Eigen::Vector3d& _t) {
        t = _t;
    }

    Pose3D inversed() {
        Pose3D pose{};
        pose.setR(R.transpose());
        pose.sett(-1 * pose.getR() * t);
        return pose;
    }
    //注意eigen的延迟更新
    void inverse() {
        R = R.transpose();
        t = -1 * R * t;
    }

    Eigen::Vector3d operator* (const Eigen::Vector3d& _P) const {
        return R * _P + t;
    }
    Eigen::Vector3d transformNormal(const Eigen::Vector3d& _n) const {
        return R * _n;
    }
    Pose3D operator* (const Pose3D _pose) {
        Pose3D pose{};
        pose.setR(R * _pose.getR());
        pose.sett(t + R * _pose.gett());
        return pose;
    }
    //注意eigen的延迟更新
    void operator*= (const Pose3D _pose) {
        t = t + R * _pose.gett();
        R = R * _pose.getR();
    }

};

//原点坐标 -R*t
static Eigen::Vector3d center3D(const Pose3D& _Tcw) {
    return -_Tcw.getR().transpose() * _Tcw.gett();
}


class Pose2D : public BaseLYJ
{
public:
    Pose2D() {};
    Pose2D(const double _thera, const Eigen::Vector2d& _t)
        :thera(_thera), t(_t)
    {}
    Pose2D(const Eigen::Matrix2d& _R, const Eigen::Vector2d& _t) {
        thera = std::atan2(_R(1, 0), _R(0, 0));
        t = _t;
    }
    Pose2D(const Eigen::Matrix<double, 2 ,3>& _R) {
        thera = std::atan2(_R(1, 0), _R(0, 0));
        t = _R.block(0,2,2,1);
    }
    ~Pose2D() {};

    void write_binary(std::ofstream& os) {
        os.write(reinterpret_cast<const char*>(&thera), sizeof(double) * 1);
        os.write(reinterpret_cast<const char*>(t.data()), sizeof(double) * 2);
    }
    void read_binary(std::ifstream& is) {
        is.read(reinterpret_cast<char*>(&thera), sizeof(double) * 1);
        is.read(reinterpret_cast<char*>(t.data()), sizeof(double) * 2);
    }
    friend std::ostream& operator<< (std::ostream& os, const Pose2D& cls) {
        std::cout << cls.getThera() << std::endl;
        std::cout << cls.gett();
        return os;
    }

    Eigen::Matrix2d getR() const {
        Eigen::Matrix2d R;
        R << std::cos(thera), -1 * std::sin(thera), std::sin(thera), std::cos(thera);
        return R;
    }
    const double& getThera() const {
        return thera;
    }
    const Eigen::Vector2d& gett() const {
        return t;
    }
    void setThera(const double _thera) {
        thera = _thera;
    }
    void setR(const Eigen::Matrix2d& _R) {
        thera = std::atan2(_R(1, 0), _R(0, 0));
    }
    void sett(const Eigen::Vector2d& _t) {
        t = _t;
    }

    Pose2D inversed() {
        Pose2D pose{};
        pose.setR(this->getR().transpose());
        pose.sett(-1 * pose.getR() * t);
        return pose;
    }
    //注意eigen的延迟更新
    void inverse() {
        this->setR(this->getR().transpose());
        t = -1 * this->getR() * t;
    }

    Eigen::Vector2d operator* (const Eigen::Vector2d& _p) const {
        return this->getR() * _p + t;
    }
    Eigen::Vector2d transformNormal(const Eigen::Vector2d& _n) {
        return this->getR() * _n;
    }
    Pose2D operator* (const Pose2D _pose) {
        Pose2D pose{};
        Eigen::Matrix2d R = this->getR();
        pose.setR(R * _pose.getR());
        pose.sett(t + R * _pose.gett());
        return pose;
    }
    //注意eigen的延迟更新
    void operator*= (const Pose2D _pose) {
        Eigen::Matrix2d R = this->getR();
        t = t + R * _pose.gett();
        this->setR(R * _pose.getR());
    }

private:
    double thera = 0;
    Eigen::Vector2d t = Eigen::Vector2d::Zero();
};

//原点坐标 -R*t
static Eigen::Vector2d center2D(const Pose2D& _Tcw) {
    return -_Tcw.getR().transpose() * _Tcw.gett();
}


NSP_SLAM_LYJ_END

#endif //SLAM_LYJ_POSE_H