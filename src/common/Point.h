#ifndef SLAM_LYJ_POINT_H
#define SLAM_LYJ_POINT_H
#include "base/Base.h"

NSP_SLAM_LYJ_MATH_BEGIN

template<typename TYPE, int DIM>
struct Point : public BaseLYJ
{
	Point() {
		memset(data, 0, DIM * sizeof(TYPE));
	}
	Point(const TYPE* _data) {
		memcpy(data, _data, DIM * sizeof(TYPE));
	}
	//Point(TYPE... _datas) {
	//	std::vector<TYPE> d = std::vector<TYPE>{ _datas... };
	//	if (DIM != (int)d.size()) {
	//		std::cout << "input dim error!" << std::endl;
	//		return;
	//	}
	//	memcpy(data, d.data(), DIM * sizeof(TYPE));
	//}
	Eigen::Matrix<TYPE, DIM, 1> toEigen() {
		Eigen::Matrix<TYPE, DIM, 1> v;
		memcpy(v.data(), data, sizeof(TYPE) * DIM);
	}
	void setByEigen(const Eigen::Matrix<TYPE, DIM, 1>& _v) {
		memcpy(data, _v.data(), sizeof(TYPE) * DIM);
	}
	double norm() {
		if (nm != 0)
			return nm;
		nm = 0;
		for (int i = 0; i < DIM; ++i) {
			nm += static_cast<double>(data[i] * data[i]);
		}
		return std::sqrt(nm);
	}
	void normalize() {
		TYPE norm = static_cast<TYPE>(nm);
		if (norm == 0) {
			std::cout << "norm is 0." << std::endl;
			return;
		}
		for (int i = 0; i < DIM; ++i) {
			data[i] /= norm;
		}
	}
	Point<TYPE, DIM> normalized() {
		Point<TYPE, DIM> res;
		TYPE norm = static_cast<TYPE>(nm);
		if (norm == 0) {
			std::cout << "norm is 0." << std::endl;
			return;
		}
		for (int i = 0; i < DIM; ++i) {
			res.data[i] /= norm;
		}
		return res;
	}
	TYPE operator[] (const int i) const {
		return data[i];
	}
	TYPE& operator[] (const int i) {
		return data[i];
	}
	TYPE operator() (const int i) const {
		return data[i];
	}
	TYPE& operator() (const int i) {
		return data[i];
	}
	Point<TYPE, DIM> operator+ (const Point<TYPE, DIM>& p) {
		Point<TYPE, DIM> res;
		for (int i = 0; i < DIM; ++i) {
			res.data[i] = data[i] + p.data[i];
		}
		return res;
	}
	Point<TYPE, DIM> operator- (const Point<TYPE, DIM>& p) {
		Point<TYPE, DIM> res;
		for (int i = 0; i < DIM; ++i) {
			res.data[i] = data[i] - p.data[i];
		}
		return res;
	}
	Point<TYPE, DIM> operator* (const int num) const {
		Point<TYPE, DIM> res;
		for (int i = 0; i < DIM; ++i) {
			res.data[i] = data[i] * num;
		}
		return res;
	}
	friend Point<TYPE, DIM> operator* (const int num, const Point<TYPE, DIM>& p) {
		Point<TYPE, DIM> res;
		for (int i = 0; i < DIM; ++i) {
			res.data[i] = num * p.data[i];
		}
		return res;
	}
	Point<TYPE, DIM> operator/ (const int num) const {
		Point<TYPE, DIM> res;
		for (int i = 0; i < DIM; ++i) {
			res.data[i] = data[i] / num;
		}
		return res;
	}
	friend Point<TYPE, DIM> operator/ (const int num, const Point<TYPE, DIM>& p) {
		Point<TYPE, DIM> res;
		for (int i = 0; i < DIM; ++i) {
			res.data[i] = num / p.data[i];
		}
		return res;
	}
	bool operator== (const Point<TYPE, DIM>& p) {
		for (int i = 0; i < DIM; ++i) {
			if (data[i] != p.data[i])
				return false;
		}
		return true;
	}
	void operator+= (const Point<TYPE, DIM>& p) {
		for (int i = 0; i < DIM; ++i) {
			data[i] += p.data[i];
		}
	}
	void operator-= (const Point<TYPE, DIM>& p) {
		for (int i = 0; i < DIM; ++i) {
			data[i] -= p.data[i];
		}
	}
	void operator*= (const int num) {
		for (int i = 0; i < DIM; ++i) {
			data[i] *= num;
		}
	}
	void operator/= (const int num) {
		for (int i = 0; i < DIM; ++i) {
			data[i] /= num;
		}
	}
	friend std::ostream& operator<< (std::ostream& os, const Point<TYPE, DIM>& p) {
		std::cout << "(";
		for (int i = 0; i < DIM; ++i) {
			if(i == DIM-1)
				std::cout << p.data[i];
			else
				std::cout << p.data[i] << ",";
		}
		std::cout << ")";
		return os;
	}
	void write_binary(std::ofstream& os) {
		os.write(reinterpret_cast<const char*>(data), sizeof(TYPE) * DIM);
		os.write(reinterpret_cast<const char*>(&nm), sizeof(double));
	}
	void read_binary(std::ifstream& is) {
		is.read(reinterpret_cast<char*>(data), sizeof(TYPE) * DIM);
		is.read(reinterpret_cast<char*>(&nm), sizeof(double));
	}
	TYPE data[DIM];
	double nm = -1;
};

template<typename TYPE>
struct Point2 : public Point<TYPE, 2>
{
	Point2() {
	}
	//Point2(TYPE... _datas) :Point(_datas...) {}
	Point2(const TYPE _x, const TYPE _y) {
		data[0] = _x;
		data[1] = _y;
	}
	inline TYPE x() const { return data[0]; }
	inline TYPE& x() { return data[0]; }
	inline TYPE y() const { return data[1]; }
	inline TYPE& y() { return data[1]; }
	Point2<TYPE> normalized() {
		Point2<TYPE> res;
		TYPE norm = static_cast<TYPE>(nm);
		if (norm == 0) {
			std::cout << "norm is 0." << std::endl;
			return;
		}
		for (int i = 0; i < 2; ++i) {
			res.data[i] /= norm;
		}
		return res;
	}
	Point2<TYPE> operator+ (const Point2<TYPE>& p) {
		Point2<TYPE> res;
		for (int i = 0; i < 2; ++i) {
			res.data[i] = data[i] + p.data[i];
		}
		return res;
	}
	Point2<TYPE> operator- (const Point2<TYPE>& p) {
		Point2<TYPE> res;
		for (int i = 0; i < 2; ++i) {
			res.data[i] = data[i] - p.data[i];
		}
		return res;
	}
	Point2<TYPE> operator* (const int num) const {
		Point2<TYPE> res;
		for (int i = 0; i < 2; ++i) {
			res.data[i] = data[i] * num;
		}
		return res;
	}
	friend Point2<TYPE> operator* (const int num, const Point2<TYPE>& p) {
		Point2<TYPE> res;
		for (int i = 0; i < 2; ++i) {
			res.data[i] = num * p.data[i];
		}
		return res;
	}
	Point2<TYPE> operator/ (const int num) const {
		Point2<TYPE> res;
		for (int i = 0; i < 2; ++i) {
			res.data[i] = data[i] / num;
		}
		return res;
	}
	friend Point2<TYPE> operator/ (const int num, const Point2<TYPE>& p) {
		Point2<TYPE> res;
		for (int i = 0; i < 2; ++i) {
			res.data[i] = num / p.data[i];
		}
		return res;
	}
};
typedef Point2<double> Point2d;
typedef Point2<float> Point2f;
typedef Point2<unsigned char> Point2c;

template<typename TYPE>
struct Point3 : public Point<TYPE, 3>
{
	Point3() {
	}
	Point3(const TYPE _x, const TYPE _y, const TYPE _z) {
		data[0] = _x;
		data[1] = _y;
		data[2] = _z;
	}
	inline TYPE x() const { return data[0]; }
	inline TYPE& x() { return data[0]; }
	inline TYPE y() const { return data[1]; }
	inline TYPE& y() { return data[1]; }
	inline TYPE z() const { return data[2]; }
	inline TYPE& z() { return data[2]; }
	Point3<TYPE> normalized() {
		Point3<TYPE> res;
		TYPE norm = static_cast<TYPE>(nm);
		if (norm == 0) {
			std::cout << "norm is 0." << std::endl;
			return;
		}
		for (int i = 0; i < 3; ++i) {
			res.data[i] /= norm;
		}
		return res;
	}
	Point3<TYPE> operator+ (const Point3<TYPE>& p) {
		Point3<TYPE> res;
		for (int i = 0; i < 3; ++i) {
			res.data[i] = data[i] + p.data[i];
		}
		return res;
	}
	Point3<TYPE> operator- (const Point3<TYPE>& p) {
		Point3<TYPE> res;
		for (int i = 0; i < 3; ++i) {
			res.data[i] = data[i] - p.data[i];
		}
		return res;
	}
	Point3<TYPE> operator* (const int num) const {
		Point3<TYPE> res;
		for (int i = 0; i < 3; ++i) {
			res.data[i] = data[i] * num;
		}
		return res;
	}
	friend Point3<TYPE> operator* (const int num, const Point3<TYPE>& p) {
		Point3<TYPE> res;
		for (int i = 0; i < 2; ++i) {
			res.data[i] = num * p.data[i];
		}
		return res;
	}
	Point3<TYPE> operator/ (const int num) const {
		Point3<TYPE> res;
		for (int i = 0; i < 3; ++i) {
			res.data[i] = data[i] / num;
		}
		return res;
	}
	friend Point3<TYPE> operator/ (const int num, const Point3<TYPE>& p) {
		Point3<TYPE> res;
		for (int i = 0; i < 2; ++i) {
			res.data[i] = num * p.data[i];
		}
		return res;
	}
};
typedef Point3<double> Point3d;
typedef Point3<float> Point3f;
typedef Point3<unsigned char> Point3c;

template<typename TYPE>
struct Point4 : public Point<TYPE, 4>
{
	Point4() {
	}
	Point4(const TYPE _x, const TYPE _y, const TYPE _z, const TYPE _w) {
		data[0] = _x;
		data[1] = _y;
		data[2] = _z;
		data[3] = _w;
	}
	inline TYPE x() const { return data[0]; }
	inline TYPE& x() { return data[0]; }
	inline TYPE y() const { return data[1]; }
	inline TYPE& y() { return data[1]; }
	inline TYPE z() const { return data[2]; }
	inline TYPE& z() { return data[2]; }
	inline TYPE w() const { return data[3]; }
	inline TYPE& w() { return data[3]; }
	Point4<TYPE> normalized() {
		Point4<TYPE> res;
		TYPE norm = static_cast<TYPE>(nm);
		if (norm == 0) {
			std::cout << "norm is 0." << std::endl;
			return;
		}
		for (int i = 0; i < 4; ++i) {
			res.data[i] /= norm;
		}
		return res;
	}
	Point4<TYPE> operator+ (const Point4<TYPE>& p) {
		Point4<TYPE> res;
		for (int i = 0; i < 4; ++i) {
			res.data[i] = data[i] + p.data[i];
		}
		return res;
	}
	Point4<TYPE> operator- (const Point4<TYPE>& p) {
		Point4<TYPE> res;
		for (int i = 0; i < 4; ++i) {
			res.data[i] = data[i] - p.data[i];
		}
		return res;
	}
	Point4<TYPE> operator* (const int num) const {
		Point4<TYPE> res;
		for (int i = 0; i < 4; ++i) {
			res.data[i] = data[i] * num;
		}
		return res;
	}
	friend Point4<TYPE> operator* (const int num, const Point4<TYPE>& p) {
		Point4<TYPE> res;
		for (int i = 0; i < 2; ++i) {
			res.data[i] = num * p.data[i];
		}
		return res;
	}
	Point4<TYPE> operator/ (const int num) const {
		Point4<TYPE> res;
		for (int i = 0; i < 4; ++i) {
			res.data[i] = data[i] / num;
		}
		return res;
	}
	friend Point4<TYPE> operator/ (const int num, const Point4<TYPE>& p) {
		Point4<TYPE> res;
		for (int i = 0; i < 2; ++i) {
			res.data[i] = num / p.data[i];
		}
		return res;
	}
};
typedef Point4<double> Point4d;
typedef Point4<float> Point4f;
typedef Point4<unsigned char> Point4c;

typedef Point2<int> Point2i;
typedef Point3<int> Point3i;
typedef Point4<int> Point4i;



NSP_SLAM_LYJ_MATH_END



#endif //SLAM_LYJ_POINT_H