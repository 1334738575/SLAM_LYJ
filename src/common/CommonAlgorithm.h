#ifndef SLAM_LYJ_COMMON_ALGORITHM_H
#define SLAM_LYJ_COMMON_ALGORITHM_H

#include "base/Base.h"


NSP_SLAM_LYJ_MATH_BEGIN

//Rodrigues
template<typename T>
static Eigen::Matrix<T, 3, 3> Rodrigues2RotMatrix(const Eigen::Matrix<T, 3, 1>& _r, const T _theta) {
	if (_theta < 1e-10) {
		return Eigen::Matrix<T, 3, 3>::Identity();
	}
	Eigen::Matrix<T, 3, 3> R = Eigen::Matrix<T, 3, 3>::Identity();
	Eigen::Matrix<T, 3, 3> r_hat = skewSymmetric(_r);
	R += r_hat * sin(_theta);
	R += r_hat * r_hat * (1 - cos(_theta));
	return R;
}

//incremental avg and var
template<typename T, int DIM>
static void calculateAgvAndVar(int _lastCnt, const T* _curData, T* _agv, T* _var, const bool _updateVar = true) {
	++_lastCnt;
	T det = 0;
	for (int i = 0; i < DIM; ++i) {
		det = _curData[i] - _agv[i];
		_agv[i] += det / _lastCnt;
		if (_lastCnt > 1)
			_var[i] = (_var[i] * (_lastCnt - 1) + det * (_curData[i] - _agv[i])) / _lastCnt;
	}
}

//临时vector，不进行数据拷贝，无法改变内部大小，可以修改数据，注意生命周期要在外部数据内
template<typename T>
class RefVector
{
public:
	RefVector(std::vector<T>& _data) {
		m_data = _data.data();
		m_size = (int)_data.size();
	}
	inline T* data() { return m_data; }
	inline int size() { return m_size; }
	T& operator[](int _i) { return m_data[_i]; }
	const T& operator[](int _i) const { return m_data[_i]; }
private:
	T* m_data = nullptr;
	int m_size = 0;
};

//compress vector<vector<T>> to vector<T>
template<typename T>
class CompressVV2V
{
public:
	CompressVV2V() = delete;
	CompressVV2V(const std::vector<std::vector<T>>& _data) {
		int indSize = (int)_data.size();
		m_indexs.resize(indSize + 1, 0);
		for (int i = 0; i < indSize; i++)
			m_indexs[i + 1] = m_indexs[i] + (int)_data[i].size();
		m_data.resize(dataSize());
		for (int i = 0; i < indSize; i++)
			memcpy(m_data + m_indexs[i], _data[i].data(), sizeof(T) * _data[i].size());
	}
	CompressVV2V(std::vector<T>&& _data, std::vector<int>&& _indexs) {
		m_data = std::move(_data);
		m_indexs = std::move(_indexs);
	}
	~CompressVV2V() {}

	inline bool empty() { return m_data.empty(); }
	inline int dataSize() {
		return m_data.size();
	}
	inline int indexSize() {
		return m_indexs.size();
	}
	inline int subSize(int _ind) {
		return m_indexs[_ind + 1] - m_indexs[_ind];
	}
	inline T& data(int _ind, int _subInd) {
		return m_data[m_indexs[_ind] + _subInd];
	}
	inline const T& data(int _ind, int _subInd) const {
		return m_data[m_indexs[_ind] + _subInd];
	}
	inline T* dataPtr() { return m_data.data(); }
	inline int* indexPtr() { return m_indexs.data(); }

private:
	std::vector<T> m_data;
	std::vector<int> m_indexs;
};

//用uchar压缩bool类型vector
class BitFlagVec {
public:
	BitFlagVec(int _size) : m_size(_size)
	{
		int s = _size >> 3;
		m_flags.resize(s + 1, 0);
	}

	inline void reset(){ m_flags.assign(m_flags.size(), 0); }
	inline const int size() { return m_size; }
	bool operator[](int _i) {
		int s = _i >> 3;
		int rs = _i % 8;
		return (m_flags[s] >> rs) & 1;
	}
	void setFlag(int _i, bool _b) {
		int s = _i >> 3;
		int rs = _i % 8;
		if (_b)
			m_flags[s] |= (1 << rs);
		else
			m_flags[s] &= ~(1 << rs);
	}

private:
	std::vector<unsigned char> m_flags;
	int m_size = 0;
};

//阶乘 n!
static uint64_t factorial(const int n, const int nSt = 1) {
	if (n < 0) {
		return 0;
	}
	else if (n == 0) {
		return 1;
	}
	uint64_t res = 1;
	for (uint64_t i = nSt; i <= n; ++i) {
		res *= i;
	}
	return res;
}
static uint64_t pernutateNum(const int m, const int n) {
	if (n <= 0)
		return 0;
	return factorial(m, m - n + 1);
}
//组合数 C(n, m)
static uint64_t selectNum(const int m, const int n) {
	if (n <= 0 || m <= 0)
		return 0;
	else if (n >= m)
		return 1;
	uint64_t c1 = factorial(n);
	uint64_t c2 = factorial(m, m - n + 1);
	uint64_t res = c2 / c1;
	return res;
}

static void permutateGen(const std::vector<int>& _datas, const int& _size, const int& _eveSize, const int& _ind, uint64_t& _resInd,
	std::vector<std::vector<int>>& _results, const std::vector<int>& _oneResult) {
	for (int i = _ind; i < _size; ++i) {
		std::vector<int> oneResult = _oneResult;
		oneResult.push_back(_datas[i]);
		if ((int)oneResult.size() == _eveSize) {
			_results[_resInd++] = oneResult;
			continue;
		}
		int reDataSize = _size - i - 1;
		int reResultSize = _eveSize - (int)oneResult.size();
		if (reResultSize > reDataSize)
			return;
		else if (reResultSize == reDataSize) {
			for (int j = i+1; j < _size; ++j) {
				oneResult.push_back(_datas[j]);
			}
			_results[_resInd++] = oneResult;
			return;
		}
		else
			permutateGen(_datas, _size, _eveSize, i + 1, _resInd, _results, oneResult);
	}
}
//全组合
static void fullSelection(const int _size, const int _eveSize, std::vector<std::vector<int>>& _results) {
	if (_size <= 0 || _eveSize <= 0 || _eveSize > _size)
		return;
	std::vector<int> datas(_size);
	for (int i = 0; i < _size; ++i) {
		datas[i] = i;
	}
	uint64_t allSize = selectNum(_size, _eveSize);
	uint64_t resInd = 0;
	_results.resize(allSize, std::vector<int>(_eveSize, -1));
	std::vector<int> oneResult;
	permutateGen(datas, _size, _eveSize, 0, resInd, _results, oneResult);
}

//随机索引组
static void randIndexGroup(const int _size, const int _eveSize, const int _groupSize, std::vector<std::vector<int>>& _results) {
	if (_size <= 0 || _eveSize <= 0 || _eveSize > _size)
		return;
	_results.resize(_groupSize, std::vector<int>(_eveSize, -1));
	// 创建随机数生成器
	std::random_device rd;
	std::mt19937 gen(rd());
	for (int i = 0; i < _groupSize; ++i) {
		std::vector<int> datas(_size);
		for (int i = 0; i < _size; ++i) {
			datas[i] = i;
		}
		// 随机打乱索引数组
		std::shuffle(datas.begin(), datas.end(), gen);

		// 选择前 k 个索引作为当前组
		std::vector<int> group(datas.begin(), datas.begin() + _eveSize);
		_results[i] = group;
	}
}

//二维叉乘
static double corss2D(const Eigen::Vector2d& _v1, const Eigen::Vector2d& _v2) {
	/*
	* 0		0	y1		x2
	* 0		0	-x1	*	y2
	* -y1	x1	0		0
	*/
	return -_v1(1) * _v2(0) + _v1(0) * _v2(1);
}

//(-pi/2, pi/2]
template<typename T>
static T regularizationAngle(const T _angle) {
	T angle = _angle;
	if (std::abs(angle) > 2 * PI) {
		int t = std::floor(angle / 2 / PI);
		angle -= t * angle;
	}
	if (angle < 0)
		angle += 2 * PI;
	if (angle > PI)
		angle -= PI;
	if (angle > PI / 2)
		angle = PI / 2 - angle;
	return angle;
}

template<typename T>
static T detAgnle(const T _angle1, const T _angle2) {
	T angle = _angle1 - _angle2;
	if (std::abs(angle) > 2 * PI) {
		int t = static_cast<int>(std::floor(angle / 2 / PI));
		angle -= t * angle;
	}
	if (angle < 0)
		angle += 2 * PI;
	if (angle > PI)
		angle -= PI;
	if (angle > PI / 2)
		angle = PI - angle;
	return angle;
}



//欧拉角转旋转矩阵

//向量反对称化
template <typename T>
static Eigen::Matrix<T, 3, 3> skewSymmetric(const Eigen::Matrix<T, 3, 1>& v) {
	Eigen::Matrix<T, 3, 3> S;
	S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
	return S;
}


//calculate fundamental and essential matrix
static Eigen::Matrix3d calculateEssentialMatrix(
	const Eigen::Matrix3d& _R12, const Eigen::Vector3d& _t12
) {
	return skewSymmetric<double>(_t12) * _R12;
}
static Eigen::Matrix3d calculateEssentialMatrix(
	const Eigen::Matrix3d& _Rcw1, const Eigen::Vector3d& _tcw1,
	const Eigen::Matrix3d& _Rcw2, const Eigen::Vector3d& _tcw2
) {
	Eigen::Matrix3d R12 = _Rcw1 * _Rcw2.transpose();
	Eigen::Vector3d t12 = _tcw1 - R12 * _tcw2;
	return calculateEssentialMatrix(R12, t12);
}
static Eigen::Matrix3d calculateFundamentalMatrix(
	const Eigen::Matrix3d& _R12, const Eigen::Vector3d& _t12,
	const Eigen::Matrix3d& _K1, const Eigen::Matrix3d& _K2
) {
	Eigen::Matrix3d invK1 = _K1.inverse();
	Eigen::Matrix3d invK2 = _K2.inverse();
	return invK1.transpose() * calculateEssentialMatrix(_R12, _t12) * invK2;
}
static Eigen::Matrix3d calculateFundamentalMatrix(
	const Eigen::Matrix3d& _Rcw1, const Eigen::Vector3d& _tcw1, const Eigen::Matrix3d& _K1,
	const Eigen::Matrix3d& _Rcw2, const Eigen::Vector3d& _tcw2, const Eigen::Matrix3d& _K2
) {
	Eigen::Matrix3d invK1 = _K1.inverse();
	Eigen::Matrix3d invK2 = _K2.inverse();
	return invK1.transpose() * calculateEssentialMatrix(_Rcw1, _tcw1, _Rcw2, _tcw2) * invK2;
}




/*******************************ORBSLAM3*********************************/
static Eigen::Matrix3d NormalizeRotation(const Eigen::Matrix3d& R)
{
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
	return svd.matrixU() * svd.matrixV();
}
static Eigen::Matrix3d ExpSO3(const double x, const double y, const double z)
{
	const double d2 = x * x + y * y + z * z;
	const double d = sqrt(d2);
	Eigen::Matrix3d W;
	W << 0.0, -z, y, z, 0.0, -x, -y, x, 0.0;
	if (d < 1e-5)
	{
		Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W + 0.5 * W * W;
		return NormalizeRotation(res);
	}
	else
	{
		Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W * sin(d) / d + W * W * (1.0 - cos(d)) / d2;
		return NormalizeRotation(res);
	}
}
static Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& w)
{
	return ExpSO3(w[0], w[1], w[2]);
}



/***************************PL-VINS**********************************/
template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived>& theta)
{
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived>& q)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1),
        q(2), typename Derived::Scalar(0), -q(0),
        -q(1), q(0), typename Derived::Scalar(0);
    return ans;
}

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived>& q)
{
    //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
    //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
    //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
    //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
    return q;
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived>& q)
{
    Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
    ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
    return ans;
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived>& p)
{
    Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
    ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
    return ans;
}

static Eigen::Vector3d R2ypr(const Eigen::Matrix3d& R)
{
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / PI * 180.0;
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived>& ypr)
{
    typedef typename Derived::Scalar Scalar_t;

    Scalar_t y = ypr(0) / 180.0 * PI;
    Scalar_t p = ypr(1) / 180.0 * PI;
    Scalar_t r = ypr(2) / 180.0 * PI;

    Eigen::Matrix<Scalar_t, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0,
        sin(y), cos(y), 0,
        0, 0, 1;

    Eigen::Matrix<Scalar_t, 3, 3> Ry;
    Ry << cos(p), 0., sin(p),
        0., 1., 0.,
        -sin(p), 0., cos(p);

    Eigen::Matrix<Scalar_t, 3, 3> Rx;
    Rx << 1., 0., 0.,
        0., cos(r), -sin(r),
        0., sin(r), cos(r);

    return Rz * Ry * Rx;
}

static Eigen::Matrix3d g2R(const Eigen::Vector3d& g)
{
	Eigen::Matrix3d R0;
	Eigen::Vector3d ng1 = g.normalized();
	Eigen::Vector3d ng2{ 0, 0, 1.0 };
	R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
	double yaw = R2ypr(R0).x();
	R0 = ypr2R(Eigen::Vector3d{ -yaw, 0, 0 }) * R0;
	// R0 = ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
	return R0;
}

template <size_t N>
struct uint_
{
};

template <size_t N, typename Lambda, typename IterT>
void unroller(const Lambda& f, const IterT& iter, uint_<N>)
{
    unroller(f, iter, uint_<N - 1>());
    f(iter + N);
}

template <typename Lambda, typename IterT>
void unroller(const Lambda& f, const IterT& iter, uint_<0>)
{
    f(iter);
}

template <typename T>
static T normalizeAngle(const T& angle_degrees) {
    T two_pi(2.0 * 180);
    if (angle_degrees > 0)
        return angle_degrees -
        two_pi * std::floor((angle_degrees + T(180)) / two_pi);
    else
        return angle_degrees +
        two_pi * std::floor((-angle_degrees + T(180)) / two_pi);
};







NSP_SLAM_LYJ_MATH_END



#endif //SLAM_LYJ_COMMON_ALGORITHM_H