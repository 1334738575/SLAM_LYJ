#ifndef SLAM_LYJ_LINE_H
#define SLAM_LYJ_LINE_H

#include "base/Base.h"
#include "base/Pose.h"


NSP_SLAM_LYJ_MATH_BEGIN
    
template<typename TYPE>
struct Line2 {

    //using TYPE = double;
    using TemVec2 = Eigen::Matrix<TYPE, 2, 1>;
    using TemVec3 = Eigen::Matrix<TYPE, 3, 1>;
    using TemVec4 = Eigen::Matrix<TYPE, 4, 1>;
    using TemMat22 = Eigen::Matrix<TYPE, 2, 2>;
    using TemMat23 = Eigen::Matrix<TYPE, 2, 3>;
    using TemMat33 = Eigen::Matrix<TYPE, 3, 3>;
    using TemMat34 = Eigen::Matrix<TYPE, 3, 4>;
    using TemMat44 = Eigen::Matrix<TYPE, 4, 4>;

    //construct
    Line2() {}
    Line2(const TemVec3& _params) {
        update(_params(0), _params(1), _params(2));
    }
    Line2(const TYPE _a, const TYPE _b, const TYPE _c) {
        update(_a, _b, _c);
    }
    Line2(const TemVec2& _v1, const TemVec2& _v2, const bool _useDir=false) {
        if(_useDir)
            update(TemVec4(_v1(0), _v1(0), _v1(0)+_v2(0), _v1(1)+_v2(1)));
        else
            update(TemVec4(_v1(0), _v1(1), _v2(0), _v2(1)));
    }
    Line2(const TemVec4& _ps) {
        update(_ps);
    }

    void update(const TYPE _a, const TYPE _b, const TYPE _c) {
        TYPE norm = std::sqrt(_a * _a + _b * _b);
        ps(0) = 0;
        ps(2) = 1;
        if (std::abs(_b) > 1e-3) {
            ps(1) = -_c / _b;
            ps(3) = -_a / _b * - _c / _b;
        }
        else {
            ps(1) = 0;
            ps(0) = -_c / _a;
            ps(3) = 1;
            ps(2) = -_c / _a;
        }
        params(0) = _a / norm;
        params(1) = _b / norm;
        params(2) = _c / norm;
        length = std::sqrt((ps(0) - ps(2)) * (ps(0) - ps(2)) + (ps(1) - ps(3)) * (ps(1) - ps(3)));
    }
    void update(const TemVec4& _ps) {
        ps = _ps;
        if (ps(0) > ps(2)) {
            ps(0) = _ps(2);
            ps(1) = _ps(3);
            ps(2) = _ps(0);
            ps(3) = _ps(1);
        }
        TemVec2 dir(ps(2) - ps(0), ps(3) - ps(1));
        length = dir.norm();
        if (length < 1e-3) {
            length = 0;
            ps.setZero();
            params.setZero();
            return;
        }
        dir.normalize();
        params(0) = -dir(1);
        params(1) = dir(0);
        params(2) = -params(0) * ps(0) - params(1) * ps(1);
    }

    TemVec2 dir() const {
        return TemVec2(params(1), -params(0));
    }
    //[-pi/2, pi/2]
    TYPE angle() const {
        return std::atan2(static_cast<double>(ps(3) - ps(1)), static_cast<double>(ps(2) - ps(0)));
    }
    TYPE dist() const {
        return -params(2);
    }
    //点到直线的距离，带符号
    TYPE distP2L(const TemVec2& _p) const {
        return _p(0) * params(0) + _p(1) * params(1) + params(2);
    }
    //点到直线的投影点
    TemVec2 pointInL(const TemVec2& _p) const {
        TYPE d = distP2L(_p);
        return _p - d * params.block(0,0,2,1);
    }

    //直线夹角,[0, pi/2]
    static TYPE angleL2L(const Line2<TYPE>& _l1, const Line2<TYPE>& _l2) {
        return detAgnle<TYPE>(_l1.angle(), _l2.angle());
    }
    //直线交点
    static bool interestL2L(const Line2<TYPE>& _l1, const Line2<TYPE>& _l2, TemVec2& _p) {
        if (angleL2L(_l1, _l2) < 3 * DEG2RAD) {
            return false;
        }
        TemMat22 m;
        m << _l1.params(0), _l1.params(1), _l2.params(0), _l2.params(1);
        TemVec2 b(-_l1.params(2), -_l2.params(2));
        _p = m.inverse() * b;
        return true;
    }
    //直线相加
    static Line2<TYPE> gatherLines(const std::vector<Line2<TYPE>>& _lines) {
        int lSize = _lines.size();
        assert(lSize > 0);
        TemVec4 ps = TemVec4::Zero();
        for (const auto& line : _lines) {
            ps(0) += line.ps(0);
            ps(1) += line.ps(1);
            ps(2) += line.ps(2);
            ps(3) += line.ps(3);
        }
        ps /= lSize;
        return Line2<TYPE>(ps);
    }
    //直线距离
    static TYPE distL2L(const Line2<TYPE>& _l1, const Line2<TYPE>& _l2) {
        if (angleL2L(_l1, _l2) < 3 * DEG2RAD) {
            return std::abs(_l1.dist() - _l2.dist());
        }
        TYPE d11 = _l1.distP2L(TemVec2(_l2.params(0), _l2.params(1)));
        TYPE d12 = _l1.distP2L(TemVec2(_l2.params(2), _l2.params(3)));
        TYPE d21 = _l2.distP2L(TemVec2(_l1.params(0), _l1.params(1)));
        TYPE d22 = _l2.distP2L(TemVec2(_l1.params(2), _l1.params(3)));
        return std::min( std::min(d11, d12), std::min(d21, d22) );
    }
    //重叠率
    static TYPE overlapL2L(const Line2<TYPE>& _l1, const Line2<TYPE>& _l2) {
        TemVec2 p11 = _l1.pointInL(TemVec2(_l2.params(0), _l2.params(1)));
        TemVec2 p12 = _l1.pointInL(TemVec2(_l2.params(2), _l2.params(3)));
        TemVec2 p21 = _l2.pointInL(TemVec2(_l1.params(0), _l1.params(1)));
        TemVec2 p22 = _l2.pointInL(TemVec2(_l1.params(2), _l1.params(3)));
        TYPE rat1 = (p11 - p12).norm() / _l1.length;
        TYPE rat2 = (p21 - p22).norm() / _l2.length;
        return std::max(rat1, rat2);
    }
    //端点位姿变换
    static Line2<TYPE> transformPoint(const Pose2D& _T, const Line2<TYPE>& _l) {
        Eigen::Vector2d sp = _l.ps.block(0, 0, 2, 1).cast<double>();
        Eigen::Vector2d spN = _T * sp;
        Eigen::Vector2d ep = _l.ps.block(2, 0, 2, 1).cast<double>();
        Eigen::Vector2d epN = _T * ep;
        return Line2(spN.cast<TYPE>(), epN.cast<TYPE>());
    }
    //2D线位姿变换

    //2D线端点转普吕克

    //2D线端点转正交

    //2D线普吕克转正交

    //2D线正交转普吕克


    void write_binary(std::ofstream& os) {
        os.write(reinterpret_cast<const char*>(params), sizeof(TYPE) * 3);
        os.write(reinterpret_cast<const char*>(ps), sizeof(TYPE) * 4);
        os.write(reinterpret_cast<const char*>(length), sizeof(TYPE) * 1);
    }
    void read_binary(std::ifstream& is) {
        is.read(reinterpret_cast<char*>(params), sizeof(TYPE) * 3);
        is.read(reinterpret_cast<char*>(ps), sizeof(TYPE) * 4);
        is.read(reinterpret_cast<char*>(length), sizeof(TYPE) * 1);
    }
    friend std::ostream& operator<< (std::ostream& os, const Line2<TYPE>& cls) {
        std::cout << "line a b c:" << cls.params(0) << " " << cls.params(1) << " " << cls.params(2) << std::endl;
        std::cout << "sp ep:" << cls.ps(0) << " " << cls.ps(1) << " " << cls.ps(2) << " " << cls.ps(3) << std::endl;
        std::cout << "length:" << cls.length;
        return os;
    }

    TemVec3 params = TemVec3::Zero();//ax+by+c=0, a^2+b^2=1, c = -1 * origin2Line
    TemVec4 ps = TemVec4::Zero();//sp, ep
    TYPE length = 0;
};
typedef Line2<double> Line2d;
typedef Line2<float> Line2f;

//光栅化 bresenham
static void bresenhamLine(int x0, int y0, int x1, int y1, std::vector<Eigen::Vector2i>& _ps)
{
    // 确保起点x0 < x1
    if (x0 > x1)
    {
        int tmpX = x0;
        x0 = x1;
        x1 = tmpX;

        int tmpy = y0;;
        y0 = y1;
        y1 = tmpy;
    }
    int dx = x1 - x0, sx = 1;
    int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int e_xy = dx - dy; // 这里的e_xy 其实是F(x+1,y+sy)
    while (true)
    {
        _ps.emplace_back(x0, y0);
        int e2 = 2 * e_xy;
        if (e2 >= -dy)
        {   // 等价于判断 F(x+1,y+sy) + F(x,y+sy) > 0; F(x,y+sy) = e_xy + dy
            if (x0 == x1)
                break;
            e_xy += -dy; x0 += sx;
        }
        if (e2 <= dx)
        {	// 等价于判断 F(x+1,y+sy) + F(x+1,y) > 0; F(x+1,y) = e_xy - dx
            if (y0 == y1)
                break;
            e_xy += dx; y0 += sy;
        }
    }
}



template<typename TYPE>
struct Line3 {

    using TemVec2 = Eigen::Matrix<TYPE, 2, 1>;
    using TemVec3 = Eigen::Matrix<TYPE, 3, 1>;
    using TemVec4 = Eigen::Matrix<TYPE, 4, 1>;
    using TemVec6 = Eigen::Matrix<TYPE, 6, 1>;
    using TemVec8 = Eigen::Matrix<TYPE, 8, 1>;
    using TemMat22 = Eigen::Matrix<TYPE, 2, 2>;
    using TemMat23 = Eigen::Matrix<TYPE, 2, 3>;
    using TemMat33 = Eigen::Matrix<TYPE, 3, 3>;
    using TemMat34 = Eigen::Matrix<TYPE, 3, 4>;
    using TemMat44 = Eigen::Matrix<TYPE, 4, 4>;
    using TemMat66 = Eigen::Matrix<TYPE, 6, 6>;

    Line3() {}
    Line3(const TemVec3& _v1, const TemVec3& _v2, bool _useDir=false) {
        if (_useDir)
            update(_v1, _v1 + _v2);
        else
            update(_v1, _v2);
    }
    void update(const TemVec3& _p1, const TemVec3& _p2) {
        sp = _p1;
        ep = _p2;
        if (_p1(0) > _p2(0)) {
            sp = _p2;
            ep = _p1;
        }
        dir = ep - sp;
        length = dir.norm();
        if (length < 1e-3) {
            sp.setZero();
            ep.setZero();
            dir.setZero();
            length = 0;
            return;
        }
        dir /= length;
        dir.normalize();
    }
    //点到直线的距离，带符号
    TYPE distP2L(const TemVec3& _p) const {
        return (_p - pointInL(_p)).norm();
    }
    //点到直线的投影点
    TemVec3 pointInL(const TemVec3& _p) const {
        return sp + ((_p - sp).dot(dir)) * dir;
    }

    //直线夹角 [0,pi/2]
    static TYPE angleL2L(const Line3<TYPE>& _l1, const Line3<TYPE>& _l2) {
        TYPE ang = std::acos(_l1.dir.dot(_l2.dir));
        if (ang > PI / 2)
            ang = PI - ang;
        return ang;
    }
    //直线交点，未验证
    static bool interestL2L(const Line3<TYPE>& _l1, const Line3<TYPE>& _l2, TemVec3& _p) {
        if (angleL2L(_l1, _l2) < 3 * DEG2RAD) {
            return false;
        }
        TemVec3 croDir = _l1.dir.cross(_l2.dir);
        TYPE croD = croDir.norm();
        if (croD < 1e-3)
            return false;
        TemMat33 m;
        m << _l1.dir, _l2.dir, croDir;
        TemVec3 L = _l1.sp - _l2.sp;
        TemVec3 par = m.inverse() * L;
        return _l1.sp + par(0) * _l1.dir + par(2) * croDir / 2;
    }
    //直线相加
    static Line3<TYPE> gatherLines(const std::vector<Line3<TYPE>>& _lines) {
        int lSize = _lines.size();
        assert(lSize > 0);
        TemVec3 p1 = TemVec3::Zero();
        TemVec3 p2 = TemVec3::Zero();
        for (const auto& line : _lines) {
            p1(0) += line.sp(0);
            p1(1) += line.sp(1);
            p1(2) += line.sp(2);
            p2(0) += line.ep(0);
            p2(1) += line.ep(1);
            p2(2) += line.ep(2);
        }
        p1 /= lSize;
        p2 /= lSize;
        return Line3<TYPE>(p1, p2);
    }
    //直线距离，不平行建议配合重叠率使用
    static TYPE distL2L(const Line3<TYPE>& _l1, const Line3<TYPE>& _l2) {
        if (angleL2L(_l1, _l2) < 3 * DEG2RAD) {
            TYPE d11 = _l1.distP2L(_l2.sp);
            TYPE d12 = _l1.distP2L(_l2.ep);
            TYPE d21 = _l2.distP2L(_l1.sp);
            TYPE d22 = _l2.distP2L(_l1.ep);
            return std::min(std::min(d11, d12), std::min(d21, d22));
        }
        TemVec3 croDir = _l1.dir.cross(_l2.dir);
        TemMat33 m;
        m << _l1.dir, _l2.dir, croDir;
        TemVec3 L = _l1.sp - _l2.sp;
        TemVec3 par = m.inverse() * L;
        return par(2) * corDir.norm();
    }
    //重叠率
    static TYPE overlapL2L(const Line3<TYPE>& _l1, const Line3<TYPE>& _l2) {
        TemVec3 p11 = _l1.pointInL(_l2.sp);
        TemVec3 p12 = _l1.pointInL(_l2.ep);
        TemVec3 p21 = _l2.pointInL(_l1.sp);
        TemVec3 p22 = _l2.pointInL(_l1.ep);
        TYPE rat1 = (p11 - p12).norm() / _l1.length;
        TYPE rat2 = (p21 - p22).norm() / _l2.length;
        return std::max(rat1, rat2);
    }
    //端点位姿变换
    static Line3<TYPE> transformLineByPoint(const Pose3D& _T, const Line3<TYPE>& _l) {
        TemVec3 p1 = _T * _l.sp;
        TemVec3 p2 = _T * _l.ep;
        return Line3<TYPE>(p1, p2);
    }


//private:
    /**************************************************************************/
    //点向转普吕克,n需要归一化
    static TemVec6 linePN_to_plk(const TemVec3& p, const TemVec3& v)
    {
        TemVec6 plk;
        TemVec3 vNorm = v.normalize();
        plk.block(0, 0, 3, 1) = p.cross(vNorm);
        plk.block(3, 0, 3, 1) = vNorm;
        return plk;
    }
    static TemVec6 linePP_to_plk(const TemVec3& p1, const TemVec3& p2)
    {
        return linePN_to_plk(p1, p2 - p1);
    }
    static TemVec6 line_to_plk(const TemVec6& line)
    {
        return linePN_to_plk(line.block(0, 0, 3, 1), line.block(3, 0, 3, 1));
    }

    //点向与正交转换
    static TemVec4 linePN_to_orth(const TemVec3& p, const TemVec3& v)
    {
        TemVec4 orth;
        TemVec3 n = p.cross(v);
        TemVec3 u1 = n / n.norm();
        TemVec3 u2 = v / v.norm();
        TemVec3 u3 = u1.cross(u2);
        orth[0] = std::atan2(u2(2), u3(2));
        orth[1] = std::asin(-u1(2));
        orth[2] = std::atan2(u1(1), u1(0));

        TemVec2 w(n.norm(), v.norm());
        w = w / w.norm();
        orth[3] = std::asin(w(1));

        return orth;
    }
    static TemVec4 line_to_orth(const TemVec6& line)
    {
        return linePN_to_orth(line.block(0, 0, 3, 1), line.block(3, 0, 3, 1));
    }
    static TemVec6 orth_to_line(const TemVec4& orth)
    {
        TemVec6 line;

        TemVec3 theta = orth.head(3);
        TYPE phi = orth[3];
        // todo:: SO3
        TYPE s1 = std::sin(theta[0]);
        TYPE c1 = std::cos(theta[0]);
        TYPE s2 = std::sin(theta[1]);
        TYPE c2 = std::cos(theta[1]);
        TYPE s3 = std::sin(theta[2]);
        TYPE c3 = std::cos(theta[2]);
        TemMat33 R;
        R <<
            c2 * c3, s1* s2* c3 - c1 * s3, c1* s2* c3 + s1 * s3,
            c2* s3, s1* s2* s3 + c1 * c3, c1* s2* s3 - s1 * c3,
            -s2, s1* c2, c1* c2;

        TYPE w1 = std::cos(phi);
        TYPE w2 = std::sin(phi);
        TYPEv d = w1 / w2;      // 原点到直线的距离

        line.head(3) = -R.col(2) * d;
        line.tail(3) = R.col(1);

        return line;


    }

    //普吕克与正交转换
    static TemVec4 plk_to_orth(const TemVec3& n, const TemVec3& v)
    {
        TemVec4 orth;
        TemVec3 u1 = n / n.norm();
        TemVec3 u2 = v / v.norm();
        TemVec3 u3 = u1.cross(u2);
        // todo:: use SO3
        orth[0] = std::atan2(u2(2), u3(2));
        orth[1] = std::asin(-u1(2));
        orth[2] = std::atan2(u1(1), u1(0));

        TemVec2 w(n.norm(), v.norm());
        w = w / w.norm();
        orth[3] = std::asin(w(1));

        return orth;
    }
    static TemVec4 plk_to_orth(const TemVec6& plk)
    {
        return plk_to_orth(plk.block(0, 0, 3, 1), plk.block(3, 0, 3, 1));

    }
    static TemVec6 orth_to_plk(const TemVec4& orth)
    {
        TemVec6 plk;

        TemVec3 theta = orth.head(3);
        TYPE phi = orth[3];
        TYPE s1 = std::sin(theta[0]);
        TYPE c1 = std::cos(theta[0]);
        TYPE s2 = std::sin(theta[1]);
        TYPE c2 = std::cos(theta[1]);
        TYPE s3 = std::sin(theta[2]);
        TYPE c3 = std::cos(theta[2]);
        TemMat33 R;
        R <<
            c2 * c3, s1* s2* c3 - c1 * s3, c1* s2* c3 + s1 * s3,
            c2* s3, s1* s2* s3 + c1 * c3, c1* s2* s3 - s1 * c3,
            -s2, s1* c2, c1* c2;

        TYPE w1 = std::cos(phi);
        TYPE w2 = std::sin(phi);
        TYPE d = w1 / w2;      // 原点到直线的距离

        TemVec3 u1 = R.col(0);
        TemVec3 u2 = R.col(1);

        TemVec3 n = w1 * u1;
        TemVec3 v = w2 * u2;

        plk.head(3) = n;
        plk.tail(3) = v;

        //TemVec3 Q = -R.col(2) * d;
        //plk.head(3) = Q.cross(v);
        //plk.tail(3) = v;
        return plk;
    }

    // 两平面相交得到直线的plucker 坐标
    static TemVec6 pipi_plk(const TemVec4& pi1, const TemVec4& pi2) {
        TemVec6 plk;
        TemMat44 dp = pi1 * pi2.transpose() - pi2 * pi1.transpose();
        plk << dp(0, 3), dp(1, 3), dp(2, 3), -dp(1, 2), dp(0, 2), -dp(0, 1);
        return plk;
    }
    // 获取光心到直线的垂直点
    static TemVec3 plucker_origin(const TemVec3& n, const TemVec3& v) {
        return v.cross(n) / v.dot(v);
    }

    //3D线位姿变换
    static TemVec6 line_to_pose(const TemVec6& line_w, const TemMat33& Rcw, const TemVec3& tcw) {
        TemVec6 line_c;
        TemVec3 cp_w, dv_w;
        cp_w = line_w.head(3);
        dv_w = line_w.tail(3);
        TemVec3 cp_c = Rcw * cp_w + tcw;
        TemVec3 dv_c = Rcw * dv_w;
        line_c.head(3) = cp_c;
        line_c.tail(3) = dv_c;
        return line_c;
    }
    static TemVec6 line_from_pose(const TemVec6& line_c, const TemMat33& Rcw, const TemVec3& tcw) {
        TemMat33 Rwc = Rcw.transpose();
        TemVec3 twc = -Rwc * tcw;
        return line_to_pose(line_c, Rwc, twc);
    }
    static TemVec6 plk_to_pose(const TemVec6& plk_w, const TemMat33& Rcw, const TemVec3& tcw) {
        TemVec3 nw = plk_w.head(3);
        TemVec3 vw = plk_w.tail(3);
        TemVec3 nc = Rcw * nw + skewSymmetric<TYPE>(tcw) * Rcw * vw;
        TemVec3 vc = Rcw * vw;
        TemVec6 plk_c;
        plk_c.head(3) = nc;
        plk_c.tail(3) = vc;
        return plk_c;
    }
    static TemVec6 plk_from_pose(const TemVec6& plk_c, const TemMat33& Rcw, const TemVec3& tcw) {
        TemMat33 Rwc = Rcw.transpose();
        TemVec3 twc = -Rwc * tcw;
        return plk_to_pose(plk_c, Rwc, twc);
    }


public:
    void write_binary(std::ofstream& os) {
        os.write(reinterpret_cast<const char*>(dir.data()), sizeof(TYPE) * 3);
        os.write(reinterpret_cast<const char*>(sp.data()), sizeof(TYPE) * 3);
        os.write(reinterpret_cast<const char*>(ep.data()), sizeof(TYPE) * 3);
        os.write(reinterpret_cast<const char*>(&length), sizeof(TYPE) * 1);
    }
    void read_binary(std::ifstream& is) {
        is.read(reinterpret_cast<char*>(dir.data()), sizeof(TYPE) * 3);
        is.read(reinterpret_cast<char*>(sp.data()), sizeof(TYPE) * 3);
        is.read(reinterpret_cast<char*>(ep.data()), sizeof(TYPE) * 3);
        is.read(reinterpret_cast<char*>(&length), sizeof(TYPE) * 1);
    }
    friend std::ostream& operator<< (std::ostream& os, const Line3<TYPE>& cls) {
        std::cout << "dir: " << dir(0) << " " << dir(1) << " " << dir(2) << std::endl;
        std::cout << "sp: " << sp(0) << " " << sp(1) << " " << sp(2) << std::endl;
        std::cout << "ep: " << ep(0) << " " << ep(1) << " " << ep(2) << std::endl;
        std::cout << "length: " << length;
        return os;
    }

    TemVec3 dir = TemVec3::Zero();
    TemVec3 sp = TemVec3::Zero();
    TemVec3 ep = TemVec3::Zero();
    TYPE length = 0;
};
typedef Line3<double> Line3d;
typedef Line3<float> Line3f;



NSP_SLAM_LYJ_MATH_END


#endif //SLAM_LYJ_LINE_H