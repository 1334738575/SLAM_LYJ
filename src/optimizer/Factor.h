#ifndef SLAM_LYJ_FACTOR_H
#define SLAM_LYJ_FACTOR_H

#include "base/PreDefine.h"
#include "Variable.h"

NSP_SLAM_LYJ_BEGIN


template<typename T>
class OptFactorAbr
{
public:
    enum OptFactorType
    {
        FACTOR_DEAULT = 0,
        FACTOR_UV_T3DPOINT3D,
        FACTOR_UV_T2D,
        FACTOR_POINT3D_T3D,
        FACTOR_UV2_T3DLINE3D,
        FACTOR_UV2_T2DLINE2D,
        FACTOR_UV_T3DPOINT3D_WITH_PLANE3D,
        FACTOR_PLANE3D_T3D_WITH_UV,
        FACTOR_T3D_T3DIMU,
        FACTOR_UNDEFINE_0 = 100,
        FACTOR_UNDEFINE_1,
        FACTOR_UNDEFINE_2
    };
    OptFactorAbr(const uint64_t _id, const OptFactorType _type) : m_errId(_id), m_type(_type) {}
    ~OptFactorAbr() {
        if(m_data)
            delete m_data;
    }

    inline const uint64_t getId() { return m_errId; }
    inline T* getData() { return m_data; }
    virtual void setData(T* _data) = 0;
    virtual const int getEDim() = 0;
    inline const int getVNum() { return m_vNum; }
    virtual const std::vector<int> getVDims() = 0;

    bool checkVDims(OptVarAbr<T>* _values) {
        const auto vDims = getVDims();
        for (int i = 0; i < m_vNum; ++i) {
            if (vDims[i] != _values[i].getDim())
                return false;
        }
        return true;
    }
    virtual bool calculateErrAndJac(T* _err, T** _jacs, T _w, OptVarAbr<T>* _values) = 0;
protected:
    const uint64_t m_errId = UINT64_MAX;
    const OptFactorType m_type = FACTOR_DEAULT;
    T* m_data = nullptr;
    int m_vNum = -1;
};

template<typename T, int EDIM, int... VDIMS>
class OptFactor : public OptFactorAbr<T>
{
protected:
public:
    OptFactor(const uint64_t _id, const OptFactorType _type) : OptFactorAbr(_id, _type)
    {
        m_data = new T[EDIM];
        memset(m_data, 0, sizeof(T) * EDIM);
        std::vector<int> vDims = std::vector<int>{ VDIMS... };
        m_vNum = (int)vDims.size();
    }
    ~OptFactor() {}

    void setData(T* _data) override { memcpy(m_data, _data, sizeof(T) * EDIM); }
    const int getEDim() override { return EDIM; }
    const std::vector<int> getVDims() override { return std::vector<int>{VDIMS...}; }
};

class OptFactorP3d_P3d : public OptFactor<double, 3, 3>
{
public:
    OptFactorP3d_P3d(const uint64_t _id) : OptFactor(_id, FACTOR_UNDEFINE_0) {}
    ~OptFactorP3d_P3d() {}

    bool calculateErrAndJac(double* _err, double** _jac, double _w, OptVarAbr<double>* _values) override
    {
        if (!checkVDims(_values))
            return false;
        auto varData = _values->getData();
        if (_err == nullptr)
            return false;
        for (size_t i = 0; i < 3; i++)
        {
            _err[i] = (m_data[i] - varData[i]) * _w;
        }
        if (_jac[0]) {
            _jac[0][0] = _err[0];
            _jac[0][1] = 0;
            _jac[0][2] = 0;
            _jac[0][3] = 0;
            _jac[0][4] = _err[1];
            _jac[0][5] = 0;
            _jac[0][6] = 0;
            _jac[0][7] = 0;
            _jac[0][8] = _err[2];
        }
        return true;
    }

private:

};


NSP_SLAM_LYJ_END

#endif //SLAM_LYJ_FACTOR_H