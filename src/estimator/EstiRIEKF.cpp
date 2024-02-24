/* ----------------------------------------------------------------------------
 * Copyright 2024, Speike <shao-haoluo@foxmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file       EstiRIEKF.cpp
 * @author     Speike
 * @date       2024/01/20 14:47:27
 * @brief      由于是根据matlab写的, 上面已经有很详细的注释了, 这里就不再重复了
 **/

#include "EstiRIEKF.h"

const int dimX = 9;
const int dimX_frak = 6;
const int dimP = 27;

EstiRIEKF::EstiRIEKF(Estimator *esti) : _esti(esti)
{
    _initriekf();
    cout << "EstiRIEKF 类中  this 地址 : " << this << endl;
    cout << "EstiRIEKF 类中 _esti 地址 : " << _esti << endl;
}

void EstiRIEKF::_initriekf()
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;
    Eigen::Matrix<double, 15, 15> P0 = Eigen::Matrix<double, 15, 15>::Identity();
    R0 << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    v0 << 0, 0, 0;
    p0 << 0.0387838, -0.000836197, 0.353079; // 因为位置不可观,所以从gazebo中读取初始位置
    bg0 << 0, 0, 0;
    ba0 << 0, 0, 0;
    P0.topLeftCorner<9, 9>() = 10 * Eigen::Matrix<double, 9, 9>::Identity();
    _initial_state.setRotation(R0);
    _initial_state.setVelocity(v0);
    _initial_state.setPosition(p0);
    _initial_state.setGyroscopeBias(bg0);
    _initial_state.setAccelerometerBias(ba0);
    _initial_state.setP(P0);

    _noise_params.setGyroscopeNoise(0.1);
    _noise_params.setAccelerometerNoise(0.1);
    _noise_params.setGyroscopeBiasNoise(0.00001);
    _noise_params.setAccelerometerBiasNoise(0.00001);
    _noise_params.setContactNoise(0.0001);

    _filter.setState(_initial_state);
    _filter.setNoiseParams(_noise_params);
    cout << "######################## Noise parameters are initialized to: #######################\n";
    cout << _filter.getNoiseParams() << endl;
    cout << "######################### Robot's state is initialized to: ########################\n";
    cout << _filter.getState() << endl;
}

void EstiRIEKF::run()
{
    runriekf();
    ++_count;
}

void EstiRIEKF::runriekf()
{
    
    // Eigen::Matrix<double, 6, 1> imu_measurement = Eigen::Matrix<double, 6, 1>::Zero();
    // cout << "Received IMU Data, propagating state\n";
    // imu_measurement.head(3) = _lowState->getGyro();
    // imu_measurement.tail(3) = _lowState->getAcc();
    // _filter.Propagate(imu_measurement, _dt);
    // LOG_EVERY_N(INFO, 100) << "_Count = " << _count << " :\n" << _filter.getState() << endl;

    // cout << "Received CONTACT Data, setting filter's contact state\n";
    // vector<pair<int, bool>> contacts;
    // for (int i = 0; i < 4; ++i)
    //     contacts.push_back(pair<int, bool>(i, (*_contact)(i)));
    // _filter.setContacts(contacts);

    // cout << "Received KINEMATIC observation, correcting state\n";
    // inekf::vectorKinematics measured_kinematics;
    // Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    // Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
    // Eigen::Matrix<double, 12, 12> Qe = 0.01 * 0.01 * Eigen::Matrix<double, 12, 12>::Identity();
    // Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(3, Qe.rows());
    // for (int i = 0; i < 4; ++i)
    // {
    //     pose.block<3, 3>(0, 0) = _robModel->getFootRotation(*_lowState, i);
    //     pose.block<3, 1>(0, 3) = _robModel->getFootPosition(*_lowState, i, FrameType::BODY);
    //     Jacobian.block<3, 3>(0, 3 * i) = _robModel->getJaco(*_lowState, i);
    //     covariance.block<3, 3>(3, 3) = Jacobian * Qe * Jacobian.transpose();
    //     inekf::Kinematics frame(i, pose, covariance);
    //     measured_kinematics.push_back(frame);
    // }
    // _filter.CorrectKinematics(measured_kinematics);
    // // cout << _filter.getState() << endl;

    // if (_riekf_count2 % ((int)(1.0 / (_dt * _riekf_pubFreq))) == 0)
    // {
    //     geometry_msgs::PoseStamped poses;
    //     nav_msgs::Odometry riekf_esti_msg;
    //     // 封装估计值
    //     riekf_esti_msg.header.stamp = ros::Time::now();
    //     riekf_esti_msg.header.frame_id = "odom";
    //     Eigen::Quaterniond q(_filter.getState().getRotation());
    //     riekf_esti_msg.pose.pose.orientation.w = q.w();
    //     riekf_esti_msg.pose.pose.orientation.x = q.x();
    //     riekf_esti_msg.pose.pose.orientation.y = q.y();
    //     riekf_esti_msg.pose.pose.orientation.z = q.z();
    //     riekf_esti_msg.pose.pose.position.x = _filter.getState().getPosition()(0);
    //     riekf_esti_msg.pose.pose.position.y = _filter.getState().getPosition()(1);
    //     riekf_esti_msg.pose.pose.position.z = _filter.getState().getPosition()(2);
    //     riekf_esti_msg.twist.twist.linear.x = _filter.getState().getVelocity()(0);
    //     riekf_esti_msg.twist.twist.linear.y = _filter.getState().getVelocity()(1);
    //     riekf_esti_msg.twist.twist.linear.z = _filter.getState().getVelocity()(2);
    //     _riekf_esti_pub2.publish(riekf_esti_msg);
    //     // 封装估计路径
    //     _riekf_esti_path_msg2.header.stamp = ros::Time::now();
    //     _riekf_esti_path_msg2.header.frame_id = "odom";
    //     poses.header.frame_id = "odom";
    //     poses.pose.orientation.w = q.w();
    //     poses.pose.orientation.x = q.x();
    //     poses.pose.orientation.y = q.y();
    //     poses.pose.orientation.z = q.z();
    //     poses.pose.position.x = _filter.getState().getPosition()(0);
    //     poses.pose.position.y = _filter.getState().getPosition()(1);
    //     poses.pose.position.z = _filter.getState().getPosition()(2);
    //     _riekf_esti_path_msg2.poses.push_back(poses);
    //     _riekf_esti_path_pub2.publish(_riekf_esti_path_msg2);
    //     _riekf_count2 = 1;
    // }
    // ++_riekf_count2;
}

void EstiRIEKF::creatROSTopic(ros::NodeHandle *n)
{
    this->_n = n;
    _riekf_esti_pub2 = _n->advertise<nav_msgs::Odometry>("/riekf/riekf_esti_pub2", 1);
    _riekf_esti_path_pub2 = _n->advertise<nav_msgs::Path>("/riekf/riekf_esti_path_pub2", 1);
}
