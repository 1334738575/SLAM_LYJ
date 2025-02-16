#ifndef SLAM_LYJ_OPTIMIZERABR_H
#define SLAM_LYJ_OPTIMIZERABR_H

#include "base/PreDefine.h"
#include "Factor.h"


NSP_SLAM_LYJ_BEGIN


template<typename T>
class OptimizerAbr
{
public:
	class OptConnect
	{
	public:
		OptConnect() {}
		OptConnect(int _num) {
			m_size = _num;
			m_connectIds = new uint64_t[_num];
			m_valids = new bool[_num];
			m_locs = new std::pair<int, int>[_num];
			reset();
		}
		~OptConnect() {
			if (m_connectIds) {
				delete m_connectIds;
				delete m_valids;
				delete m_locs;
			}
		}
		inline size() { return m_size; }
		void reset() {
			memset(m_connectIds, 1, sizeof(uint64_t) * _num);
			memset(m_valids, 0, sizeof(bool) * _num);
			memset(m_locs, 0, sizeof(std::pair<int, int>) * _num);
		}
		inline uint64_t& connectId(const int _i) { return m_connectIds[_i]; }
		inline const uint64_t& connectId(const int _i) const { return m_connectIds[_i]; }
		inline bool& valid(const int _i) { return m_valids[_i]; }
		inline const bool& valid(const int _i) { return m_valids[_i]; }
		inline std::pair<int, int>& loc(const int _i) { return m_locs[_i]; }
		inline const std::pair<int, int>& loc(const int _i) { return m_locs[_i]; }

	private:
		uint64_t* m_connectIds = nullptr;
		bool* m_valids = nullptr;
		std::pair<int, int>* m_locs = nullptr;
		int m_size = 0;
	};
	typedef OptConnect Var2Factor;
	typedef OptConnect Factor2Var;

	template<typename T>
	class MatInner
	{
	public:
		MatInner() {}
		MatInner(const int _r, const int _c) m_r(_r), m_c(_c)
		{
			m_data = new T[_r * _c];
			reset();
		}
		~MatInner() {
			if (m_data)
				delete m_data;
		}
		inline void reset() { memset(m_data, 0, sizeof(T) * m_r * m_c); }
		inline T* getData() { return m_data; }
		inline void setData(T* _data) { memcpy(m_data, _data, sizeof(T) * m_r * m_c); }
		inline const int r() { return m_r; }
		inline const int c() { return m_c; }
	private:
		T* m_data = nullptr;
		int m_r = -1;
		int m_c = -1;
	};
	class JacManager
	{
	public:
		JacManager(const uint64_t _id, const Var2Factor& _v2f,
			const std::vector<OptFactorAbr<T>>& _factors, const std::vector<Factor2Var>& _factor2Vars) {
			m_id = _id;
			m_jacs.resize(_v2f.size());
			for (int i = 0; i < _v2f.size(); i++)
			{
				const uint64_t& fId = _v2f.connectId(i);
				const Factor2Var& f2v = _factor2Vars[fId];
				const OptFactorAbr<T>& f = _factors[fId];
				const int r = f.getEDim();
				const std::vector<int> vDims = f.getVDims();
				for (int j = 0; j < f2v.size(); j++)
				{
					if (_id != f2v.connectId(j))
						continue;
					const int& c = vDims[j];
					m_jacs[i] = MatInner(r, c);
				}
			}
		}
		~JacManager() {}

		//非位姿jac，QR分解，变换位姿jac
		void QR() {

		}

	private:
		uint64_t m_id = UINT64_MAX;
		std::vector<MatInner> m_jacs;
	};

	OptimizerAbr() {}
	~OptimizerAbr() {}

	virtual bool run() {
		for (int i = 0; i < m_maxIterNum; ++i) {
			T err = -1;
			if (!generateAB(err))
				return false;
			if (!solveDetX())
				return false;
			if (!updateX())
				return false;
			if (isFinish(i, err))
				break;
		}
		return true;
	}

	virtual bool generateAB(T& _err) = 0;

	virtual bool solveDetX() = 0;

	virtual bool updateX() = 0;

	virtual bool isFinish(const int _i, const T _err) {
		m_lastErr = _err;
		if (_err <= m_minErrTh || _i >= m_maxIterNum)
			return true;
		return false;
	}

private:
	int m_maxIterNum = 30;
	T m_minErrTh = 0;
	T m_lastErr = -1;

	std::vector<OptVarAbr<T>> m_vars;
	std::vector<Var2Factor> m_var2Factors;
	std::vector<OptFactorAbr<T>> m_factors;
	std::vector<Factor2Var> m_factor2Vars;
	std::vector<JacManager> m_jacManagers;
};




NSP_SLAM_LYJ_END


#endif //SLAM_LYJ_OPTIMIZERABR_H