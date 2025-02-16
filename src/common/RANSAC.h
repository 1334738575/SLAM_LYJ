#ifndef SLAM_LYJ_RANSAC_H
#define SLAM_LYJ_RANSAC_H

#include "base/Base.h"

#include "common/CommonAlgorithm.h"

NSP_SLAM_LYJ_MATH_BEGIN

template<typename DATATYPE, typename ERRORTYPE, typename MODULE>
class RANSAC
{
public:
	typedef std::function<bool(const MODULE& mdl, const DATATYPE& data, ERRORTYPE& err)> FuncCalError;
	typedef std::function<bool(const std::vector<const DATATYPE*>& samples, MODULE& mdl)> FuncCalModule;
	//dropped
	RANSAC(const double _inlineRatioTh, const double _dstInlineRatioTh, const double _stopRatio,
		const int _maxIterNum, const int _minInlineNum, FuncCalError _funcCalErr, FuncCalModule _funcCalModule)
		:m_stopRatio(_stopRatio), m_minInlineNum(_minInlineNum), m_funcCalErr(_funcCalErr), m_funcCalModule(_funcCalModule)
	{
		double dstRat = std::clamp(_dstInlineRatioTh, 0.01, 0.99);
		double invInRatN = 1 - std::pow(_inlineRatioTh, _minInlineNum);
		double invDstRat = 1 - dstRat;
		int maxInterNum = static_cast<int>(std::log(invDstRat) / std::log(invDstRat));
		m_maxIterNum = std::min(_maxIterNum, _maxIterNum);
	};
	RANSAC(const double _inlineRatioTh, const int _minInlineNum,
		const int _maxIterNum = INT32_MAX, const double _dstInlineRatioTh = 0.99, const double _stopRatio = 1.0)
		:m_stopRatio(_stopRatio), m_minInlineNum(_minInlineNum)
	{
		double dstRat = std::clamp(_dstInlineRatioTh, 0.01, 0.99);
		double invInRatN = 1 - std::pow(_inlineRatioTh, _minInlineNum);
		double invDstRat = 1 - dstRat;
		int maxInterNum = static_cast<int>(std::log(invDstRat) / std::log(invDstRat));
		m_maxIterNum = std::min(_maxIterNum, maxInterNum);
	};
	void setCallBack(FuncCalError _funcCalErr, FuncCalModule _funcCalModule) {
		m_funcCalErr = _funcCalErr;
		m_funcCalModule = _funcCalModule;
	}

	~RANSAC() {};

	double run(const std::vector<DATATYPE>& _datas, std::vector<ERRORTYPE>& _errs, std::vector<bool>& _bInlines, MODULE& _mdl) {
		const int dataSize = (int)_datas.size();
		if (dataSize < m_minInlineNum)
			return 0.0;
		double bestRatio = m_stopRatio/2;
		std::vector<std::vector<int>> allSamples(m_maxIterNum, std::vector<int>(m_minInlineNum, -1));
		generateAllSamples(dataSize, m_minInlineNum, allSamples);
		//可以并行化，或者内部并行化
		for (int i = 0; i < m_maxIterNum; ++i) {
			const std::vector<int>& inds = allSamples[i];
			if (inds[0] == -1)
				continue;
			std::vector<ERRORTYPE> errs(dataSize);
			std::vector<bool> bInlines(dataSize, false);
			MODULE mdl;
			double rat = runOnce(_datas, inds, errs, bInlines, mdl);
			if (rat <= bestRatio)
				continue;
			bestRatio = rat;
			_mdl = mdl;
			_errs = errs;
			_bInlines = bInlines;
			if (rat >= m_stopRatio) {
				break;
			}
		}
		return bestRatio;
	}

private:
	void generateAllSamples(const int _allSize, const int _eveSize, std::vector<std::vector<int>>& _allSamples) {
		uint64_t selSize = selectNum(_allSize, _eveSize);
		if (selSize < (uint64_t)m_maxIterNum) {
			m_maxIterNum = (int)selSize;
			_allSamples.resize(selSize);
			fullSelection(_allSize, _eveSize, _allSamples);
			return;
		}
		randIndexGroup(_allSize, _eveSize, m_maxIterNum, _allSamples);
	}
	double runOnce(const std::vector<DATATYPE>& _datas, const std::vector<int>& _inds, std::vector<ERRORTYPE>& _errs, std::vector<bool>& _bInlines, MODULE& _mdl) {
		std::vector<const DATATYPE*> samples;
		for (const auto& ind : _inds) {
			samples.push_back(&_datas[ind]);
		}
		if (!m_funcCalModule(samples, _mdl)) {
			return 0.0;
		}
		int dataSize = (int)_datas.size();
		int validCnt = 0;
		for (int i = 0; i < dataSize; ++i) {
			_bInlines[i] = m_funcCalErr(_mdl, _datas[i], _errs[i]);
			if(_bInlines[i])
				++validCnt;
		}
		return static_cast<double>(validCnt)/static_cast<double>(dataSize);
	}

private:
	double m_stopRatio = 1;
	int m_maxIterNum = INT32_MAX;
	int m_minInlineNum = INT32_MAX;
	FuncCalError m_funcCalErr = nullptr;
	FuncCalModule m_funcCalModule = nullptr;
};


NSP_SLAM_LYJ_MATH_END

#endif //SLAM_LYJ_RANSAC_H