/* ----------------------------------------------------------------------------
 * Copyright 2024, Speike <shao-haoluo@foxmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file       EstiRIEKF.h
 * @author     Speike
 * @date       2024/01/20 14:34:52
 * @brief      右不变扩展卡尔曼滤波器在四足状态估计中的应用
 **/
#ifndef ESTIRIEKF_H
#define ESTIRIEKF_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "modern_robotics.h"
#include "Eigen/Dense"
#include "../riekf/InEKF.h"
#include <deque>
#include <numeric>
#include <glog/logging.h>
#include "../legKinematics/A1Kinematics.h"
#include "estimator.h"

using namespace std;

class Estimator;

class EstiRIEKF
{
public:
    EstiRIEKF(Estimator *esti);
    ~EstiRIEKF() = default;
    void run();
    void runriekf();
    void creatROSTopic(ros::NodeHandle *n);

private:
    void _initriekf();

private:
    // 构造函数使用，与原程序对接的一些信息
    double _dt;
    ros::NodeHandle *_n;
    double _riekf_pubFreq = 100;
    long long int _count = 0;
    ros::Publisher _riekf_esti_pub2;
    ros::Publisher _riekf_esti_path_pub2;
    nav_msgs::Path _riekf_esti_path_msg2;

    /// riekf的官方代码迁移
    inekf::RobotState _initial_state;
    inekf::NoiseParams _noise_params;
    inekf::InEKF _filter;
    double _riekf_pubFreq2 = 100;
    int _riekf_count2 = 0;

private:
    /// IMU初始化相关
    bool init_success_ = false;                          // 初始化是否成功
    int init_imu_queue_max_size_ = 3000;                 // 初始化IMU队列最大长度
    double max_static_gyro_var = 0.5;                    // 静态下陀螺测量方差
    double max_static_acce_var = 0.05;                   // 静态下加计测量方差
    Eigen::Vector3d cov_gyro_ = Eigen::Vector3d::Zero(); // 陀螺测量噪声协方差（初始化时评估）
    Eigen::Vector3d cov_acce_ = Eigen::Vector3d::Zero(); // 加计测量噪声协方差（初始化时评估）
    Eigen::Vector3d init_bg_ = Eigen::Vector3d::Zero();  // 陀螺初始零偏
    Eigen::Vector3d init_ba_ = Eigen::Vector3d::Zero();  // 加计初始零偏

private:
    Estimator *_esti;
};

#endif
