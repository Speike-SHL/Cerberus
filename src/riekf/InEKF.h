/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF.h
 *  @author Ross Hartley
 *  @brief  Header file for Invariant EKF 
 *  @date   September 25, 2018
 **/

#ifndef INEKF_H
#define INEKF_H 
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <map>
#if INEKF_USE_MUTEX
#include <mutex>
#endif
#include <algorithm>
#include "RobotState.h"
#include "NoiseParams.h"
#include "LieGroup.h"

namespace inekf {

/// @brief 数据类型,包含腿id,4x4位姿,6x6协方差
class Kinematics {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Kinematics(int id_in, Eigen::Matrix4d pose_in, Eigen::Matrix<double, 6, 6> covariance_in) : id(id_in), pose(pose_in), covariance(covariance_in) {}

        int id; /// 腿编号
        Eigen::Matrix4d pose; /// 位姿4x4
        Eigen::Matrix<double,6,6> covariance; /// 协方差
};

/// @brief 数据类型,包含地标id,3x1位移
class Landmark {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Landmark(int id_in, Eigen::Vector3d position_in) : id(id_in), position(position_in) { }

        int id;
        Eigen::Vector3d position;
};

// 定义了一个map容器, 键为int, 值为Vector3d, std::less表示键按照升序排列, 最后是用于对组的Eigen内存对齐
typedef std::map<int, Eigen::Vector3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Eigen::Vector3d>>> mapIntVector3d;
typedef std::map<int, Eigen::Vector3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Eigen::Vector3d>>>::iterator mapIntVector3dIterator;
typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark>> vectorLandmarks;
typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark>>::const_iterator vectorLandmarksIterator;
typedef std::vector<Kinematics, Eigen::aligned_allocator<Kinematics>> vectorKinematics;
typedef std::vector<Kinematics, Eigen::aligned_allocator<Kinematics>>::const_iterator vectorKinematicsIterator;

/// @brief 观测类,包含Y(观测) b H(观测矩阵) N(观测噪声协方差) PI(选择矩阵)
class Observation {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Observation(Eigen::VectorXd& Y, Eigen::VectorXd& b, Eigen::MatrixXd& H, Eigen::MatrixXd& N, Eigen::MatrixXd& PI);

        bool empty();

        Eigen::VectorXd Y;
        Eigen::VectorXd b;
        Eigen::MatrixXd H;
        Eigen::MatrixXd N;
        Eigen::MatrixXd PI;

        
        friend std::ostream& operator<<(std::ostream& os, const Observation& o);  
};


class InEKF {
    
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        InEKF();
        InEKF(NoiseParams params);
        InEKF(RobotState state);
        InEKF(RobotState state, NoiseParams params);

        RobotState getState();
        NoiseParams getNoiseParams();
        mapIntVector3d getPriorLandmarks();
        std::map<int,int> getEstimatedLandmarks();
        std::map<int,bool> getContacts();
        std::map<int,int> getEstimatedContactPositions();
        void setState(RobotState state);
        void setNoiseParams(NoiseParams params);
        void setPriorLandmarks(const mapIntVector3d& prior_landmarks);
        void setContacts(std::vector<std::pair<int,bool> > contacts);

        void Propagate(const Eigen::Matrix<double,6,1>& m, double dt);
        void Correct(const Observation& obs);
        void CorrectLandmarks(const vectorLandmarks& measured_landmarks);
        void CorrectKinematics(const vectorKinematics& measured_kinematics);

    private:
        RobotState state_;
        NoiseParams noise_params_;
        const Eigen::Vector3d g_; /// Gravity
        mapIntVector3d prior_landmarks_;    /// 静态地标<id p_wl>
        std::map<int,int> estimated_landmarks_;   /// 动态地标<id 在状态X中的索引>
        std::map<int,bool> contacts_;   /// 储存腿id和接触状态
        std::map<int,int> estimated_contact_positions_;   /// 记录已经加入状态估计的腿的(id, 在X阵中的索引)
#if INEKF_USE_MUTEX
        std::mutex estimated_contacts_mutex_;
        std::mutex estimated_landmarks_mutex_;
#endif
};

} // end inekf namespace
#endif 
