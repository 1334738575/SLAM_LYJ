#ifndef SLAM_LYJ_TENSOR_H
#define SLAM_LYJ_TENSOR_H

#include "base/Base.h"

NSP_SLAM_LYJ_MATH_BEGIN


//tmp, input colmajor, return colmajor
static double getElement(const double * const data, const int ind){
    return data[ind];
}
static double getElement(const double * const data, const int rows, const int cols, const int ri, const int ci){
    return getElement(data, ci*rows+ri);
}

template<int ROWS, int COLS>
struct QRLYJ {
    static void QR(double* data, double* Q, double* R) {
        std::cout<<"Not suggest using!"<<std::endl;
        Eigen::Matrix<double, ROWS, COLS> matOri = Eigen::Map<Eigen::Matrix<double, ROWS, COLS>>(data);
        Eigen::Matrix<double, ROWS, ROWS> mQ = Eigen::Map<Eigen::Matrix<double, ROWS, ROWS>>(Q);
        Eigen::Matrix<double, ROWS, COLS> mR = Eigen::Map<Eigen::Matrix<double, ROWS, COLS>>(R);
        std::cout << "matOri:" << std::endl;
        std::cout << matOri << std::endl;
        Eigen::Matrix<double, ROWS, 1> n = matOri.col(0);
        Eigen::Matrix<double, ROWS, 1> a = Eigen::Matrix<double, ROWS, 1>::Zero();
        a(0) = n.norm();
        Eigen::Matrix<double, ROWS, 1> u = n - a;
        //u.normalize();
        double uNorm = u.squaredNorm();
        if (uNorm < 1e-10) {
            Eigen::Matrix<double, ROWS, COLS - 1> matOri2 = matOri.block(0, 1, ROWS, COLS - 1);
            Eigen::Matrix<double, ROWS, ROWS> matQ2;
            Eigen::Matrix<double, ROWS, COLS - 1> matR2;
            QRLYJ<ROWS, COLS - 1> qr;
            return qr.QR(matOri2.data(), matQ2.data(), matR2.data());
        }
        Eigen::Matrix<double, ROWS, ROWS> uut;
        for (int i = 0; i < ROWS; ++i) {
            for (int j = 0; j < ROWS; ++j) {
                uut(i, j) = u(i) * u(j);
            }
        }
        mQ = Eigen::Matrix<double, ROWS, ROWS>::Identity() - 2 * uut / uNorm;
        mR = mQ * matOri;
        std::cout << "mQ:" << std::endl;
        std::cout << mQ << std::endl;
        std::cout << "mR:" << std::endl;
        std::cout << mR << std::endl;
        Eigen::Matrix<double, ROWS - 1, COLS - 1> matOri2 = mR.block(1, 1, ROWS - 1, COLS - 1);
        Eigen::Matrix<double, ROWS - 1, ROWS - 1> matQ2;
        Eigen::Matrix<double, ROWS - 1, COLS - 1> matR2;
        QRLYJ<ROWS - 1, COLS - 1> qr;
        qr.QR(matOri2.data(), matQ2.data(), matR2.data());
    }
};
template<int COLS>
struct QRLYJ <1, COLS> {
    static void QR(double* data, double* Q, double* R) {
        Eigen::Matrix<double, 1, COLS> matOri = Eigen::Map<Eigen::Matrix<double, 1, COLS>>(data);
        Eigen::Matrix<double, 1, COLS> mR = Eigen::Map<Eigen::Matrix<double, 1, COLS>>(R);
        std::cout << "matOri:" << std::endl;
        std::cout << matOri << std::endl;
        *Q = 1;
        mR = matOri;
        std::cout << "mQ:" << std::endl;
        std::cout << *Q << std::endl;
        std::cout << "mR:" << std::endl;
        std::cout << mR << std::endl;
    }
};
template<int ROWS>
struct QRLYJ<ROWS, 1> {
    static void QR(double* data, double* Q, double* R) {
        Eigen::Matrix<double, ROWS, 1> matOri = Eigen::Map<Eigen::Matrix<double, ROWS, 1>>(data);
        Eigen::Matrix<double, ROWS, ROWS> mQ = Eigen::Map<Eigen::Matrix<double, ROWS, ROWS>>(Q);
        Eigen::Matrix<double, ROWS, 1> mR = Eigen::Map<Eigen::Matrix<double, ROWS, 1>>(R);
        std::cout << "matOri:" << std::endl;
        std::cout << matOri << std::endl;
        Eigen::Matrix<double, ROWS, 1>& n = matOri;
        Eigen::Matrix<double, ROWS, 1> a = Eigen::Matrix<double, ROWS, 1>::Zero();
        a(0) = n.norm();
        Eigen::Matrix<double, ROWS, 1> u = n - a;
        //u.normalize();
        double uNorm = u.squaredNorm();
        if (uNorm < 1e-10) {
            mQ.setIdentity();
            mR = matOri;
            return;
        }
        Eigen::Matrix<double, ROWS, ROWS> uut;
        for (int i = 0; i < ROWS; ++i) {
            for (int j = 0; j < ROWS; ++j) {
                uut(i, j) = u(i) * u(j);
            }
        }
        mQ = Eigen::Matrix<double, ROWS, ROWS>::Identity() - 2 * uut / uNorm;
        mR = a; // mQ* matOri;
        std::cout << "mQ:" << std::endl;
        std::cout << mQ << std::endl;
        std::cout << "mR:" << std::endl;
        std::cout << mR << std::endl;
    }
};
template<>
struct QRLYJ<1, 1> {
    static void QR(double* data, double* Q, double* R) {
        std::cout << "matOri:" << std::endl;
        std::cout << *data << std::endl;
        *Q = 1;
        *R = *data;
        std::cout << "mQ:" << std::endl;
        std::cout << *Q << std::endl;
        std::cout << "mR:" << std::endl;
        std::cout << *R << std::endl;
    }
};

struct TensorLYJ
{

};

NSP_SLAM_LYJ_MATH_END

#endif //SLAM_LYJ_TENSOR_H