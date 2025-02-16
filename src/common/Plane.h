#ifndef SLAM_LYJ_PLANE_H
#define SLAM_LYJ_PLANE_H
#include "base/Base.h"
#include "base/Pose.h"
#include "common/line.h"

NSP_SLAM_LYJ_MATH_BEGIN


template<typename TYPE>
struct Plane3
{

	using TemVec2 = Eigen::Matrix<TYPE, 2, 1>;
	using TemVec3 = Eigen::Matrix<TYPE, 3, 1>;
	using TemVec4 = Eigen::Matrix<TYPE, 4, 1>;
	using TemVec6 = Eigen::Matrix<TYPE, 6, 1>;
	using TemMat22 = Eigen::Matrix<TYPE, 2, 2>;
	using TemMat23 = Eigen::Matrix<TYPE, 2, 3>;
	using TemMat33 = Eigen::Matrix<TYPE, 3, 3>;
	using TemMat34 = Eigen::Matrix<TYPE, 3, 4>;
	using TemMat44 = Eigen::Matrix<TYPE, 4, 4>;

	Plane3() {};
	~Plane3() {};
	Plane3(const TemVec3& _n, const TemVec3& _c) {
		update(_n, _c);
	}
	Plane3(const TemVec3& _P0, const TemVec3& _P1, const TemVec3& _P2) {
		TemVec3 n = (_P1 - _P0).cross(_P2 - _P0);
		TemVec3 c = (_P0 + _P1 + _P2) / 3;
		update(n, c);
	}
	///*
	// 三点确定一个平面 a(x-x0)+b(y-y0)+c(z-z0)=0  --> ax + by + cz + d = 0   d = -(ax0 + by0 + cz0)
	// 平面通过点（x0,y0,z0）以及垂直于平面的法线（a,b,c）来得到
	// (a,b,c)^T = vector(AO) cross vector(BO)
	// d = O.dot(cross(AO,BO))
	// */
	//Plane3(const TemVec3& x1, const TemVec3& x2, const TemVec3& x3) {
	//	params << (x1 - x3).cross(x2 - x3), -x3.dot(x1.cross(x2)); // d = - x3.dot( (x1-x3).cross( x2-x3 ) ) = - x3.dot( x1.cross( x2 ) )
	//	TemVec3 c = (x1 + x2 + x3) / 3;
	//}
	void update(TemVec3 _n, const TemVec3& _c) {
		_n.normalize();
		params(3) = -1 * _c.dot(_n);
		params(0) = _n(0);
		params(1) = _n(1);
		params(2) = _n(2);
		center = _c;
	}

	TemVec3 dir() {
		return params.block(0, 0, 3, 1);
	}
	//原点到平面距离
	TYPE dist() {
		return -params(0);
	}
	//点到平面距离，带符号
	TYPE dist2Point(TemVec3& _p) {
		return _p(0) * params(0) + _p(1) * params(1) + _p(2) * params(2) + params(3);
	}
	//直线与平面交点
	bool interPointWithLine(const Line3<TYPE>& _l, TemVec3& _p) {
		TYPE d = std::abs(dist2Point(_l.sp));
		const TemVec3& lDir = _l.dir;
		TYPE cs = lDir.dot(dir());
		int s = 1;
		if (std::abs(cs) < 1e-6)
			return false;
		else if (cs < -1 * 1e-6)
			s = -1;
		_p = d / cs * lDir + _l.sp;
		return true;
	}
	//点到平面投影
	static TemVec3 pointInPlane(const TemVec3& _p) {
		TemVec3 p;
		TYPE d = std::abs(dist2Point(_p));
		p = _p - d * dir();
	}
	//投影线
	static Line3<TYPE> lineInPlane(const Line3<TYPE>& _l) {
		TYPE d1 = std::abs(dist2Point(_l.sp));
		TYPE d2 = std::abs(dist2Point(_l.ep));
		if (std::abs(d1) < 1e-3 && std::abs(d2) < 1e-3)
			return _l;
		TemVec3 pInter1 = pointInPlane(_l.sp);
		TemVec3 pInter2 = pointInPlane(_l.ep);
		return Line3<TYPE>(pInter1, pInter2);
	}

	//平面与平面夹角
	static Line3<TYPE> anglePlane2Plane(const Plane3<TYPE>& _plane1, const Plane3<TYPE>& _plane2) {
		TYPE ang = std::acos(_plane1.dir().dot(_plane2.dir()));
		if (ang > PI / 2)
			ang = PI - ang;
		return ang;
	}
	//平面与平面交线
	static bool interLine(const Plane3<TYPE>& _plane1, const Plane3<TYPE>& _plane2, Line3<TYPE>& _l) {
		if (anglePlane2Plane(_plane1, _plane2) < 3 * DEG2RAD)
			return false;
		TemVec3 lDir = _plane1.dir().dot(_plane2.dir());
		TemMat33 m;
		m << _plane1.dir(), _plane2.dir(), lDir;
		TemVec3 b(_plane1.params(3), _plane2.params(3), 0);
		TemVec3 p = m.inverse() * b;
		_l.update(p, p + lDir);
	}
	// 两平面相交得到直线的plucker 坐标
	static TemVec6 interLinePLK(const Plane3<TYPE>& pi1, const Plane3<TYPE>& pi2) {
		TemVec6 plk;
		TemMat44 dp = pi1.params * pi2.params.transpose() - pi2.params * pi1.params.transpose();
		plk << dp(0, 3), dp(1, 3), dp(2, 3), -dp(1, 2), dp(0, 2), -dp(0, 1);
		return plk;
	}


	void write_binary(std::ofstream& os) {
		os.write(reinterpret_cast<const char*>(params.data()), sizeof(TYPE) * 4);
		os.write(reinterpret_cast<const char*>(canter.data()), sizeof(TYPE) * 3);
	}
	void read_binary(std::ifstream& is) {
		is.read(reinterpret_cast<char*>(params.data()), sizeof(TYPE) * 4);
		is.read(reinterpret_cast<char*>(center.data()), sizeof(TYPE) * 3);
	}
	friend std::ostream& operator<< (std::ostream& os, const Plane3<TYPE>& cls) {
		std::cout << "params: " << cls.params(0) << " " << cls.params(1) << " " << cls.params(2) << " " << cls.params(3) << std::endl;
		std::cout << "center: " << cls.center(0) << " " << cls.center(1) << " " << cls.center(2);
		return os;
	}
	
	//3D平面位姿变换
	static Plane3<TYPE> transformPlane(const Pose3D& _T, const Plane3<TYPE>& _plane) {
		TemVec3 n = _T.transformNormal(_plane.params.block(0, 0, 3, 1));
		TemVec3 c = _T * _plane.center;
		return Plane3<TYPE>(n, c);
	}

	TemVec4 params = TemVec4::Zero();//ax+by+cz+d=0, a^2+b^2+c^2=1, d = -1 * origin2Plane
	TemVec3 center = TemVec3::Zero();
};
typedef Plane3<double> Plane3d;
typedef Plane3<float> Plane3f;



NSP_SLAM_LYJ_MATH_END



#endif //SLAM_LYJ_PLANE_H