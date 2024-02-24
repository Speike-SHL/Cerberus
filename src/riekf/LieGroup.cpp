/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   LieGroup.cpp
 *  @author Ross Hartley
 *  @brief  Source file for various Lie Group functions 
 *  @date   September 25, 2018
 **/

#include "LieGroup.h"

namespace inekf {

using namespace std;

const double TOLERANCE = 1e-10;

/// 返回向量v的反对称阵
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    // Convert vector to skew-symmetric matrix
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    M << 0, -v[2], v[1],
         v[2], 0, -v[0], 
        -v[1], v[0], 0;
        return M;
}

/// 指数映射,三维向量到旋转矩阵
Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w) {
    // Computes the vectorized exponential map for SO(3)
    Eigen::Matrix3d A = skew(w);
    double theta = w.norm();
    if (theta < TOLERANCE) {
        return Eigen::Matrix3d::Identity();
    }
    Eigen::Matrix3d R =  Eigen::Matrix3d::Identity() + (sin(theta)/theta)*A + ((1-cos(theta))/(theta*theta))*A*A;
    return R;
}

/// SEK3的指数映射
/// \f[
/// \mathrm{Exp}\left( \xi \right) =\left[ \begin{matrix}
/// 	R&		J_l\xi _v&		J_l\xi _p\\
/// 	0&		1&		0\\
/// 	0&		0&		1\\
/// \end{matrix} \right] 
/// \f]
Eigen::MatrixXd Exp_SEK3(const Eigen::VectorXd &v)
{
    // Computes the vectorized exponential map for SE_K(3)
    int K = (v.size()-3)/3;
    Eigen::MatrixXd X = Eigen::MatrixXd::Identity(3+K,3+K);
    Eigen::Matrix3d R;
    Eigen::Matrix3d Jl;
    Eigen::Vector3d w = v.head(3);
    double theta = w.norm();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    if (theta < TOLERANCE) {
        R = I;
        Jl = I;
    } else {
        Eigen::Matrix3d A = skew(w);
        double theta2 = theta*theta;
        double stheta = sin(theta);
        double ctheta = cos(theta);
        double oneMinusCosTheta2 = (1-ctheta)/(theta2);
        Eigen::Matrix3d A2 = A*A;
        R =  I + (stheta/theta)*A + oneMinusCosTheta2*A2;
        Jl = I + oneMinusCosTheta2*A + ((theta-stheta)/(theta2*theta))*A2;
    }
    X.block<3,3>(0,0) = R;
    for (int i=0; i<K; ++i) {
        X.block<3,1>(0,3+i) = Jl * v.segment<3>(3+3*i);
    }
    return X;
}

/// 返回SEK3的伴随
/// \f[
/// \left[ \begin{matrix}
///     \mathrm{R}&		\mathrm{v}&		\mathrm{p}\\
///     0&		1&		0\\
///     0&		0&		1\\
/// \end{matrix} \right] \rightarrow \left[ \begin{matrix}
///     \mathrm{R}&		0&		0\\
///     \mathrm{v}^{\land}\mathrm{R}&		\mathrm{R}&		0\\
///     \mathrm{p}^{\land}\mathrm{R}&		0&		\mathrm{R}\\
/// \end{matrix} \right] 
/// \f]
Eigen::MatrixXd Adjoint_SEK3(const Eigen::MatrixXd& X) {
    // Compute Adjoint(X) for X in SE_K(3)
    int K = X.cols()-3;
    Eigen::MatrixXd Adj = Eigen::MatrixXd::Zero(3+3*K, 3+3*K);
    Eigen::Matrix3d R = X.block<3,3>(0,0);
    Adj.block<3,3>(0,0) = R;
    for (int i=0; i<K; ++i) {
        Adj.block<3,3>(3+3*i,3+3*i) = R;
        Adj.block<3,3>(3+3*i,0) = skew(X.block<3,1>(0,3+i))*R;
    }
    return Adj;
}

} // end inekf namespace
