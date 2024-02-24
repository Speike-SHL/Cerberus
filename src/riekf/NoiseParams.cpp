/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   NoiseParams.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Invariant EKF noise parameter class
 *  @date   September 25, 2018
 **/

#include "NoiseParams.h"

namespace inekf
{

   using namespace std;

   // ------------ NoiseParams -------------
   /// 噪声默认构造 Gyro-0.01 Accl-0.1 Gyrobias-1e-5 Acclbias-1e-4 landmark-0.1 Contact-0.1
   NoiseParams::NoiseParams()
   {
      setGyroscopeNoise(0.01);
      setAccelerometerNoise(0.1);
      setGyroscopeBiasNoise(0.00001);
      setAccelerometerBiasNoise(0.0001);
      setLandmarkNoise(0.1);
      setContactNoise(0.1);
   }

   /// 设置 Qg 3x3, 每个对角都为std^2
   void NoiseParams::setGyroscopeNoise(double std) { Qg_ = std * std * Eigen::Matrix3d::Identity(); }
   /// 设置 Qg 3x3, 对角分别对应std(0)~std(3)
   void NoiseParams::setGyroscopeNoise(const Eigen::Vector3d &std) { Qg_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2); }
   /// 设置 Qg 3x3, Qg = 传入的cov
   void NoiseParams::setGyroscopeNoise(const Eigen::Matrix3d &cov) { Qg_ = cov; }

   /// 设置 Qa 3x3, 每个对角都为std^2
   void NoiseParams::setAccelerometerNoise(double std) { Qa_ = std * std * Eigen::Matrix3d::Identity(); }
   /// 设置 Qa 3x3, 对角分别对应std(0)~std(3)
   void NoiseParams::setAccelerometerNoise(const Eigen::Vector3d &std) { Qa_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2); }
   /// 设置 Qa 3x3, Qg = 传入的cov
   void NoiseParams::setAccelerometerNoise(const Eigen::Matrix3d &cov) { Qa_ = cov; }

   /// 设置 Qbg 3x3, 每个对角都为std^2
   void NoiseParams::setGyroscopeBiasNoise(double std) { Qbg_ = std * std * Eigen::Matrix3d::Identity(); }
   /// 设置 Qbg 3x3, 对角分别对应std(0)~std(3)
   void NoiseParams::setGyroscopeBiasNoise(const Eigen::Vector3d &std) { Qbg_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2); }
   // 设置 Qbg 3x3, Qg = 传入的cov
   void NoiseParams::setGyroscopeBiasNoise(const Eigen::Matrix3d &cov) { Qbg_ = cov; }

   /// 设置 Qba 3x3, 每个对角都为std^2
   void NoiseParams::setAccelerometerBiasNoise(double std) { Qba_ = std * std * Eigen::Matrix3d::Identity(); }
   /// 设置 Qba 3x3, 对角分别对应std(0)~std(3)
   void NoiseParams::setAccelerometerBiasNoise(const Eigen::Vector3d &std) { Qba_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2); }
   /// 设置 Qba 3x3, Qg = 传入的cov
   void NoiseParams::setAccelerometerBiasNoise(const Eigen::Matrix3d &cov) { Qba_ = cov; }

   /// 设置 Ql 3x3, 每个对角都为std^2
   void NoiseParams::setLandmarkNoise(double std) { Ql_ = std * std * Eigen::Matrix3d::Identity(); }
   // 设置 Ql 3x3, 对角分别对应std(0)~std(3)
   void NoiseParams::setLandmarkNoise(const Eigen::Vector3d &std) { Ql_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2); }
   // 设置 Ql 3x3, Qg = 传入的cov
   void NoiseParams::setLandmarkNoise(const Eigen::Matrix3d &cov) { Ql_ = cov; }

   /// 设置 Qc 3x3, 每个对角都为std^2
   void NoiseParams::setContactNoise(double std) { Qc_ = std * std * Eigen::Matrix3d::Identity(); }
   /// 设置 Qc 3x3, 对角分别对应std(0)~std(3)
   void NoiseParams::setContactNoise(const Eigen::Vector3d &std) { Qc_ << std(0) * std(0), 0, 0, 0, std(1) * std(1), 0, 0, 0, std(2) * std(2); }
   /// 设置 Qc 3x3, Qg = 传入的cov
   void NoiseParams::setContactNoise(const Eigen::Matrix3d &cov) { Qc_ = cov; }

   Eigen::Matrix3d NoiseParams::getGyroscopeCov() { return Qg_; }
   Eigen::Matrix3d NoiseParams::getAccelerometerCov() { return Qa_; }
   Eigen::Matrix3d NoiseParams::getGyroscopeBiasCov() { return Qbg_; }
   Eigen::Matrix3d NoiseParams::getAccelerometerBiasCov() { return Qba_; }
   Eigen::Matrix3d NoiseParams::getLandmarkCov() { return Ql_; }
   Eigen::Matrix3d NoiseParams::getContactCov() { return Qc_; }

   /// NoiseParams重载<<运算符, 打印输出当前对象的Qg,Qa,Qbg,Qba,Ql,Qc
   std::ostream &operator<<(std::ostream &os, const NoiseParams &p)
   {
      os << "--------- Noise Params -------------" << endl;
      os << "Gyroscope Covariance:\n"
         << p.Qg_ << endl;
      os << "Accelerometer Covariance:\n"
         << p.Qa_ << endl;
      os << "Gyroscope Bias Covariance:\n"
         << p.Qbg_ << endl;
      os << "Accelerometer Bias Covariance:\n"
         << p.Qba_ << endl;
      os << "Landmark Covariance:\n"
         << p.Ql_ << endl;
      os << "Contact Covariance:\n"
         << p.Qc_ << endl;
      os << "-----------------------------------" << endl;
      return os;
   }

} // end inekf namespace
