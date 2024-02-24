//
// Created by shuoy on 7/19/21.
//

#include "estimator.h"
#include "../utils/visualization.h"

Estimator::Estimator() : f_manager{Rs}
{
    ROS_INFO("init begins");
    initThreadFlag = false;
    clearState();
    _riekf = new EstiRIEKF(this);
    cout << "Estimator 类中 _riekf 地址 : " << _riekf << endl;
    cout << "Estimator 类中   this 地址 : " << this << endl;
}

Estimator::~Estimator()
{
    if (MULTIPLE_THREAD)
    {
        processThread.join();
        printf("join thread \n");
    }
}

void Estimator::clearState()
{
    mProcess.lock();
    // clear IMU buffer
    while (!accBuf.empty())
        accBuf.pop();
    while (!gyrBuf.empty())
        gyrBuf.pop();
    while (!featureBuf.empty())
        featureBuf.pop();
    while (!legAngBufList.empty())
        legAngBufList.pop();
    while (!legAngVelBufList.empty())
        legAngVelBufList.pop();
    while (!contactFlagBufList.empty())
        contactFlagBufList.pop();

    prevTime = -1;
    curTime = 0;
    openExEstimation = 0;
    initP = Eigen::Vector3d(0, 0, 0);
    initR = Eigen::Matrix3d::Identity();
    inputImageCnt = 0;
    initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();

        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();
        joint_angle_buf[i].clear();
        joint_velocity_buf[i].clear();
        foot_contact_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;

        if (il_pre_integrations[i] != nullptr)
        {
            delete il_pre_integrations[i];
        }
        il_pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;

    if (tmp_il_pre_integration != nullptr)
        delete tmp_il_pre_integration;

    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    tmp_il_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;

    mProcess.unlock();
}

void Estimator::setParameter()
{
    mProcess.lock();
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl
             << ric[i] << endl
             << tic[i].transpose() << endl;
    }
    f_manager.setRic(ric);
    ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
    g = G;
    cout << "set g " << g.transpose() << endl;
    featureTracker.readIntrinsicParameter(CAM_NAMES);

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !initThreadFlag)
    {
        initThreadFlag = true;
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
    // set leg kinematics related parameters
    // body_to_a1_body, in case the IMU and the robot body frame has nontrival transformation
    p_br = Eigen::Vector3d(0, 0.0, 0);
    R_br = Eigen::Matrix3d::Identity();
    // leg order: 0-FL  1-FR  2-RL  3-RR
    leg_offset_x[0] = 0.1805;
    leg_offset_x[1] = 0.1805;
    leg_offset_x[2] = -0.1805;
    leg_offset_x[3] = -0.1805;
    leg_offset_y[0] = 0.047;
    leg_offset_y[1] = -0.047;
    leg_offset_y[2] = 0.047;
    leg_offset_y[3] = -0.047;
    motor_offset[0] = 0.0838;
    motor_offset[1] = -0.0838;
    motor_offset[2] = 0.0838;
    motor_offset[3] = -0.0838;
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.21;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = VILO_LOWER_LEG_LENGTH;

    for (int i = 0; i < NUM_OF_LEG; i++)
    {
        Eigen::VectorXd rho_fix(RHO_FIX_SIZE);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i];
        rho_fix_list.push_back(rho_fix);
    }

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rho1[i] = VILO_LOWER_LEG_LENGTH * Eigen::Matrix<double, RHO_OPT_SIZE, 1>::Ones();
        Rho2[i] = VILO_LOWER_LEG_LENGTH * Eigen::Matrix<double, RHO_OPT_SIZE, 1>::Ones();
        Rho3[i] = VILO_LOWER_LEG_LENGTH * Eigen::Matrix<double, RHO_OPT_SIZE, 1>::Ones();
        Rho4[i] = VILO_LOWER_LEG_LENGTH * Eigen::Matrix<double, RHO_OPT_SIZE, 1>::Ones();
    }

    mProcess.unlock();
}
void Estimator::changeSensorType(int use_imu, int use_stereo)
{
    bool restart = false;
    mProcess.lock();
    if (!use_imu && !use_stereo)
        printf("at least use two sensors! \n");
    else
    {
        if (USE_IMU != use_imu)
        {
            USE_IMU = use_imu;
            if (USE_IMU)
            {
                // reuse imu; restart system
                restart = true;
            }
            else
            {
                if (last_marginalization_info != nullptr)
                    delete last_marginalization_info;

                tmp_pre_integration = nullptr;
                tmp_il_pre_integration = nullptr;
                last_marginalization_info = nullptr;
                last_marginalization_parameter_blocks.clear();
            }
        }

        STEREO = use_stereo;
        printf("use imu %d use stereo %d\n", USE_IMU, STEREO);
    }
    mProcess.unlock();
    if (restart)
    {
        clearState();
        setParameter();
    }
}

// convert input image into featureFrame, and then save featureFrame in featureBuf
void Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
{
    inputImageCnt++;
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    TicToc featureTrackerTime;

    if (_img1.empty())
        featureFrame = featureTracker.trackImage(t, _img);
    else
        featureFrame = featureTracker.trackImage(t, _img, _img1);
    // printf("featureTracker time: %f\n", featureTrackerTime.toc());

    if (SHOW_TRACK)
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();
        pubTrackImage(imgTrack, t);
    }

    if (MULTIPLE_THREAD)
    {
        //        if(inputImageCnt % 2 == 0) // why this, this should not be correct
        //        {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));
        mBuf.unlock();
        //        }
    }
    else
    {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));
        mBuf.unlock();
        TicToc processTime;
        processMeasurements();
        printf("process time: %f\n", processTime.toc());
    }
}

// save imu data into buffer
// use fastPredict to output an odometry that is the same frequency as IMU
void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();

    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    // printf("input imu with time %f \n", t);
    mBuf.unlock();

    // this logic boosts output to the same rate as IMU
    // need to deal this with leg odometry
    if (solver_flag == NON_LINEAR)
    {
        mPropagate.lock();
        fastPredictIMU(t, linearAcceleration, angularVelocity);
        pubLatestOdometry(latest_P, latest_Q, latest_V, t);
        mPropagate.unlock();
    }
}

void Estimator::inputLeg(double t, const Eigen::Ref<const Vector_dof> &jointAngles,
                         const Eigen::Ref<const Vector_dof> &jointVels, const Eigen::Ref<const Vector_leg> &contact_flags)
{
    mBuf.lock();
    leg_msg_counter++;

    //    if (leg_msg_counter % 2 == 0) {
    legAngBufList.push(make_pair(t, jointAngles));
    legAngVelBufList.push(make_pair(t, jointVels));
    contactFlagBufList.push(make_pair(t, contact_flags));
    //    }
    //    std::cout << "input foot force" << footForces.transpose() << std::endl;
    //    printf("input leg joint state and foot force with time %f \n", t);
    mBuf.unlock();
}

void Estimator::inputFeature(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame)
{
    mBuf.lock();
    featureBuf.push(make_pair(t, featureFrame));
    mBuf.unlock();

    if (!MULTIPLE_THREAD)
        processMeasurements();
}

// t0: the time of the previous image frame
// t1: the time of the current image frame
bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector,
                               vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if (accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    //    printf("get imu from %f %f\n", t0, t1);
    //    printf("imu front time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    // must have more IMU than image
    if (t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
}

// internal
bool Estimator::IMUAvailable(double t)
{
    if (!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}

// assume leg measurement aligns with IMU measurement very well
bool Estimator::getIMUAndLegInterval(double t0, double t1,
                                     vector<pair<double, Eigen::Vector3d>> &accVector,
                                     vector<pair<double, Eigen::Vector3d>> &gyrVector,
                                     vector<pair<double, Vector_dof>> &jointAngVector,
                                     vector<pair<double, Vector_dof>> &jointVelVector,
                                     vector<pair<double, Vector_leg>> &contactFlagVector)
{
    if (accBuf.empty())
    {
        printf("not receive imu nor leg\n");
        return false;
    }
    // must have more IMU than image
    if (t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0)
        {
            accBuf.pop();
            gyrBuf.pop();
            legAngBufList.pop();
            legAngVelBufList.pop();
            contactFlagBufList.pop();
        }
        while (accBuf.front().first < t1)
        {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
            jointAngVector.push_back(legAngBufList.front());
            legAngBufList.pop();
            jointVelVector.push_back(legAngVelBufList.front());
            legAngVelBufList.pop();
            contactFlagVector.push_back(contactFlagBufList.front());
            contactFlagBufList.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
        jointAngVector.push_back(legAngBufList.front());
        jointVelVector.push_back(legAngVelBufList.front());
        contactFlagVector.push_back(contactFlagBufList.front());
    }
    else
    {
        printf("wait for imu and leg\n");
        return false;
    }
    return true;
}

// in MULTIPLE_THREAD mode, this function will be called periodically
// 在void Estimator::setParameter()函数中被调用
void Estimator::processMeasurements()
{
    while (true)
    {
        pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>> feature;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;

        // vector of time index, Vector12d for ang, vel Vector4d for footForce
        vector<pair<double, Vector_dof>> jointAngVector, jointVelVector;
        vector<pair<double, Vector_leg>> contactFlagVector;

        if (!featureBuf.empty())
        {
            feature = featureBuf.front();
            curTime = feature.first + td;
            // wait until IMU is available
            while (1)
            {
                if ((!USE_IMU || IMUAvailable(feature.first + td)))
                    break;
                else
                {
                    // printf("wait for imu and leg ... \n");   // NOTE 注释输出
                    if (!MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }
            }
            mBuf.lock();
            // extract imu data from accbuf, gyrbuf
            // save imu data into accVector and gyrVector

            // TODO: get leg info and IMU together
            if (USE_LEG && USE_IMU)
            {
                // 从缓冲区内获取指定时间间隔的IMU和腿部数据
                getIMUAndLegInterval(prevTime, curTime, accVector, gyrVector, jointAngVector, jointVelVector, contactFlagVector);
            }
            else if (USE_IMU)
            {
                getIMUInterval(prevTime, curTime, accVector, gyrVector);
            }

            // remove feature buf
            featureBuf.pop();
            mBuf.unlock();

            // 打印从缓冲区读出的所有数据
            // cout << "=================================================================" << endl;
            // cout << "prevTime: " << prevTime << endl;
            // cout << "curTime: " << curTime << endl;
            // cout << " accVector size " << accVector.size()
            //      << " gyrVector size " << gyrVector.size()
            //      << " jointAngVector size " << jointAngVector.size()
            //      << " jointVelVector size " << jointVelVector.size()
            //      << " contactFlagVector size " << contactFlagVector.size();
            // for (const auto &item : accVector)
            //     cout << "accVector " << item.first << " " << item.second.transpose() << endl;
            // for (const auto &item : gyrVector)
            //     cout << "gyrVector " << item.first << " " << item.second.transpose() << endl;
            // for (const auto &item : jointAngVector)
            //     cout << "jointAngVector " << item.first << " " << item.second.transpose() << endl;
            // for (const auto &item : jointVelVector)
            //     cout << "jointVelVector " << item.first << " " << item.second.transpose() << endl;
            // for (const auto &item : contactFlagVector)
            //     cout << "contactFlagVector " << item.first << " " << item.second.transpose() << endl;
            // cout << "=================================================================" << endl;

            Vector3d tmpPs;
            Vector3d tmpPs2;
            Vector3d tmpVs;
            Matrix3d tmpRs;

            if (USE_LEG && USE_IMU)
            {
                cout << "USE_LEG & USE_IMU" << endl;
                // average acc to get initial Rs[0]
                if (!initFirstPoseFlag)
                    initFirstIMUPose(accVector);
                for (size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if (i == 0)
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;

                    processIMULeg(accVector[i].first, dt, accVector[i].second, gyrVector[i].second,
                                  jointAngVector[i].second, jointVelVector[i].second, contactFlagVector[i].second);
                }

                /* many debug print goes here */
            }
            else if (USE_IMU)   //没有进这里
            {
                // average acc to get initial Rs[0]
                if (!initFirstPoseFlag)
                    initFirstIMUPose(accVector);

                // Rs Ps now contains value before IMU propagation
                // save Rs Vs Ps before processing IMU
                tmpRs = Rs[frame_count];
                tmpPs = Ps[frame_count];
                tmpVs = Vs[frame_count];
                // pre-integration?
                for (size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if (i == 0)
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                }
            }

            mProcess.lock();
            processImage(feature.second, feature.first);
            prevTime = curTime;

            printStatistics(*this, 0);

            std_msgs::Header header;
            header.frame_id = "world";
            header.stamp = ros::Time::now();
            pubOdometry(*this, header);
            pubKeyPoses(*this, header);
            pubCameraPose(*this, header);
            pubPointCloud(*this, header);
            pubKeyframe(*this);
            pubTF(*this, header);
            mProcess.unlock();
        }

        if (!MULTIPLE_THREAD)
            break;

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    // return;
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for (size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());
    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    Rs[0] = R0;
    cout << "init R0 " << endl
         << Rs[0] << endl;
    // Vs[0] = Vector3d(5, 0, 0);
}

void Estimator::initFirstPose(Eigen::Vector3d p, Eigen::Matrix3d r)
{
    Ps[0] = p;
    Rs[0] = r;
    initP = p;
    initR = r;
}

void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        // if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processIMULeg(double t, double dt,
                              const Vector3d &linear_acceleration, const Vector3d &angular_velocity,
                              const Ref<const Vector_dof> &joint_angle, const Ref<const Vector_dof> &joint_velocity,
                              const Ref<const Vector_leg> &foot_contact)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
        phi_0 = joint_angle;
        dphi_0 = joint_velocity;
        c_0 = foot_contact;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }

    if (!il_pre_integrations[frame_count])
    {
        Vector_rho tmp;
        tmp.segment<RHO_OPT_SIZE>(0 * RHO_OPT_SIZE) = Rho1[frame_count];
        tmp.segment<RHO_OPT_SIZE>(1 * RHO_OPT_SIZE) = Rho2[frame_count];
        tmp.segment<RHO_OPT_SIZE>(2 * RHO_OPT_SIZE) = Rho3[frame_count];
        tmp.segment<RHO_OPT_SIZE>(3 * RHO_OPT_SIZE) = Rho4[frame_count];
        il_pre_integrations[frame_count] = new IMULegIntegrationBase{acc_0, gyr_0, phi_0, dphi_0, c_0,
                                                                     Bas[frame_count], Bgs[frame_count], tmp, rho_fix_list, p_br, R_br};
    }

    if (frame_count != 0)
    {
        //        std::cout << "input leg data: \n" << joint_angle.transpose() << std::endl;
        //        std::cout << joint_velocity.transpose() << std::endl;
        //        std::cout << foot_contact.transpose() << std::endl;
        //        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        il_pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity, joint_angle, joint_velocity, foot_contact);
        // if(solver_flag != NON_LINEAR)
        //        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);
        tmp_il_pre_integration->push_back(dt, linear_acceleration, angular_velocity, joint_angle, joint_velocity, foot_contact); // TAG 处理IMUlLEG

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);
        joint_angle_buf[frame_count].emplace_back(joint_angle);
        joint_velocity_buf[frame_count].emplace_back(joint_velocity);
        foot_contact_buf[frame_count].emplace_back(foot_contact);

        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
    phi_0 = joint_angle;
    dphi_0 = joint_velocity;
    c_0 = foot_contact;
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const double header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
    {
        marginalization_flag = MARGIN_OLD;
        // printf("keyframe\n");
    }
    else
    {
        marginalization_flag = MARGIN_SECOND_NEW;
        // printf("non-keyframe\n");
    }

    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header);
    imageframe.il_pre_integration = tmp_il_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));
    //    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    Vector_rho tmp;
    tmp.segment<RHO_OPT_SIZE>(0 * RHO_OPT_SIZE) = Rho1[frame_count];
    tmp.segment<RHO_OPT_SIZE>(1 * RHO_OPT_SIZE) = Rho2[frame_count];
    tmp.segment<RHO_OPT_SIZE>(2 * RHO_OPT_SIZE) = Rho3[frame_count];
    tmp.segment<RHO_OPT_SIZE>(3 * RHO_OPT_SIZE) = Rho4[frame_count];
    tmp_il_pre_integration = new IMULegIntegrationBase{acc_0, gyr_0, phi_0, dphi_0, c_0,
                                                       Bas[frame_count], Bgs[frame_count], tmp, rho_fix_list, p_br, R_br};

    // we do not really use this
    //    if(ESTIMATE_EXTRINSIC == 2)
    //    {
    //        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
    //        if (frame_count != 0)
    //        {
    //            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
    //            Matrix3d calib_ric;
    //            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
    //            {
    //                ROS_WARN("initial extrinsic rotation calib success");
    //                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
    //                ric[0] = calib_ric;
    //                RIC[0] = calib_ric;
    //                ESTIMATE_EXTRINSIC = 1;
    //            }
    //        }
    //    }

    if (solver_flag == INITIAL)
    {
        // monocular + IMU initilization
        //        if (!STEREO && USE_IMU)
        //        {
        //            if (frame_count == WINDOW_SIZE)
        //            {
        //                bool result = false;
        //                if(ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
        //                {
        //                    result = initialStructure();
        //                    initial_timestamp = header;
        //                }
        //                if(result)
        //                {
        //                    optimization();
        //                    updateLatestStates();
        //                    solver_flag = NON_LINEAR;
        //                    slideWindow();
        //                    ROS_INFO("Initialization finish!");
        //                }
        //                else
        //                    slideWindow();
        //            }
        //        }

        // stereo + IMU initilization
        if (STEREO && USE_IMU)
        {
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
            if (frame_count == WINDOW_SIZE)
            {
                map<double, ImageFrame>::iterator frame_it;
                int i = 0;
                for (frame_it = all_image_frame.begin(); frame_it != all_image_frame.end(); frame_it++)
                {
                    frame_it->second.R = Rs[i];
                    frame_it->second.T = Ps[i];
                    i++;
                }
                //  initialize the leg bias too
                // debug: check il_pre_integrations at this point
                solveGyroscopeBias(all_image_frame, Bgs);
                //                solveGyroLegBias(all_image_frame, Bgs, Rho1, Rho2, Rho3, Rho4);
                for (int i = 0; i <= WINDOW_SIZE; i++)
                {
                    Vector_rho tmp;
                    tmp.segment<RHO_OPT_SIZE>(0 * RHO_OPT_SIZE) = Rho1[i];
                    tmp.segment<RHO_OPT_SIZE>(1 * RHO_OPT_SIZE) = Rho2[i];
                    tmp.segment<RHO_OPT_SIZE>(2 * RHO_OPT_SIZE) = Rho3[i];
                    tmp.segment<RHO_OPT_SIZE>(3 * RHO_OPT_SIZE) = Rho4[i];
                    il_pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i], tmp);
                }
                ROS_WARN_STREAM("Initialization finish!0");
                optimization();
                ROS_WARN_STREAM("Initialization finish!1");
                updateLatestStates();
                ROS_WARN_STREAM("Initialization finish!2");
                solver_flag = NON_LINEAR;
                slideWindow();
                ROS_INFO("Initialization finish!");
                ROS_WARN_STREAM("Initialization finish!");
            }
        }

        // stereo only initilization
        //        if(STEREO && !USE_IMU)
        //        {
        //            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        //            f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        //            optimization();
        //
        //            if(frame_count == WINDOW_SIZE)
        //            {
        //                optimization();
        //                updateLatestStates();
        //                solver_flag = NON_LINEAR;
        //                slideWindow();
        //                ROS_INFO("Initialization finish!");
        //            }
        //        }

        if (frame_count < WINDOW_SIZE)
        {
            frame_count++;
            int prev_frame = frame_count - 1;
            Ps[frame_count] = Ps[prev_frame];
            Vs[frame_count] = Vs[prev_frame];
            Rs[frame_count] = Rs[prev_frame];
            Bas[frame_count] = Bas[prev_frame];
            Bgs[frame_count] = Bgs[prev_frame];
            Rho1[frame_count] = Rho1[prev_frame];
            Rho2[frame_count] = Rho2[prev_frame];
            Rho3[frame_count] = Rho3[prev_frame];
            Rho4[frame_count] = Rho4[prev_frame];
        }
    }
    else
    {
        TicToc t_solve;
        if (!USE_IMU)
            f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
        f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
        optimization();
        set<int> removeIndex;
        outliersRejection(removeIndex);
        f_manager.removeOutlier(removeIndex);
        if (!MULTIPLE_THREAD)
        {
            featureTracker.removeOutliers(removeIndex);
            predictPtsInNextFrame();
        }

        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        slideWindow();
        f_manager.removeFailures();
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
        updateLatestStates();
    }
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        if (USE_IMU)
        {
            para_SpeedBias[i][0] = Vs[i].x();
            para_SpeedBias[i][1] = Vs[i].y();
            para_SpeedBias[i][2] = Vs[i].z();

            para_SpeedBias[i][3] = Bas[i].x();
            para_SpeedBias[i][4] = Bas[i].y();
            para_SpeedBias[i][5] = Bas[i].z();

            para_SpeedBias[i][6] = Bgs[i].x();
            para_SpeedBias[i][7] = Bgs[i].y();
            para_SpeedBias[i][8] = Bgs[i].z();
        }
        if (USE_LEG)
        {
            para_LegBias[i][0] = Rho1[i][0];
            para_LegBias[i][1] = Rho2[i][0];
            para_LegBias[i][2] = Rho3[i][0];
            para_LegBias[i][3] = Rho4[i][0];
        }
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    if (USE_IMU)
    {
        Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                         para_Pose[0][3],
                                                         para_Pose[0][4],
                                                         para_Pose[0][5])
                                                 .toRotationMatrix());
        double y_diff = origin_R0.x() - origin_R00.x();
        // TODO
        Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
        if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
        {
            ROS_DEBUG("euler singular point!");
            rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                           para_Pose[0][3],
                                           para_Pose[0][4],
                                           para_Pose[0][5])
                                   .toRotationMatrix()
                                   .transpose();
        }

        for (int i = 0; i <= WINDOW_SIZE; i++)
        {

            Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                        para_Pose[i][1] - para_Pose[0][1],
                                        para_Pose[i][2] - para_Pose[0][2]) +
                    origin_P0;

            Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                        para_SpeedBias[i][1],
                                        para_SpeedBias[i][2]);

            Bas[i] = Vector3d(para_SpeedBias[i][3],
                              para_SpeedBias[i][4],
                              para_SpeedBias[i][5]);

            Bgs[i] = Vector3d(para_SpeedBias[i][6],
                              para_SpeedBias[i][7],
                              para_SpeedBias[i][8]);
        }
    }
    else
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

            Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);
        }
    }

    if (USE_IMU)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            tic[i] = Vector3d(para_Ex_Pose[i][0],
                              para_Ex_Pose[i][1],
                              para_Ex_Pose[i][2]);
            ric[i] = Quaterniond(para_Ex_Pose[i][6],
                                 para_Ex_Pose[i][3],
                                 para_Ex_Pose[i][4],
                                 para_Ex_Pose[i][5])
                         .normalized()
                         .toRotationMatrix();
        }
    }

    if (USE_LEG)
    {
        for (int i = 0; i <= WINDOW_SIZE; i++)
        {
            Rho1[i] << para_LegBias[i][0];
            Rho2[i] << para_LegBias[i][1];
            Rho3[i] << para_LegBias[i][2];
            Rho4[i] << para_LegBias[i][3];
        }
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);

    if (USE_IMU)
        td = para_Td[0][0];
}

bool Estimator::failureDetection()
{
    return false;
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        // return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        // ROS_INFO(" big translation");
        // return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        // ROS_INFO(" big z translation");
        // return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        // return true;
    }
    return false;
}

void Estimator::optimization()
{
    TicToc t_whole, t_prepare;
    vector2double();

    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    // loss_function = NULL;
    loss_function = new ceres::HuberLoss(1.0);
    // loss_function = new ceres::CauchyLoss(1.0 / FOCAL_LENGTH);
    // ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
    for (int i = 0; i < frame_count + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        if (USE_IMU)
            problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
        if (USE_LEG)
            problem.AddParameterBlock(para_LegBias[i], SIZE_LEG_BIAS);

        if (USE_LEG && !OPTIMIZE_LEG_BIAS)
        {
            problem.SetParameterBlockConstant(para_LegBias[i]);
        }

        if (frame_count < WINDOW_SIZE)
        {
            problem.SetParameterBlockConstant(para_LegBias[i]);
        }
    }
    if (!USE_IMU)
        problem.SetParameterBlockConstant(para_Pose[0]);

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if ((ESTIMATE_EXTRINSIC && frame_count == WINDOW_SIZE && Vs[0].norm() > 0.2) || openExEstimation)
        {
            // ROS_INFO("estimate extinsic param");
            openExEstimation = 1;
        }
        else
        {
            // ROS_INFO("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }
    problem.AddParameterBlock(para_Td[0], 1);

    if (!ESTIMATE_TD || Vs[0].norm() < 0.2)
        problem.SetParameterBlockConstant(para_Td[0]);

    if (last_marginalization_info && last_marginalization_info->valid)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    if (USE_LEG && USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (il_pre_integrations[j]->sum_dt > 10.0)
                continue;

            // for testing:
            // evaluate residual
            //            Vector12d rhoi; rhoi.setZero();
            //            rhoi.segment<3>(0) = Rho1[i];
            //            rhoi.segment<3>(3) = Rho2[i];
            //            rhoi.segment<3>(6) = Rho3[i];
            //            rhoi.segment<3>(9) = Rho4[i];
            //            Vector12d rhoj; rhoi.setZero();
            //            rhoj.segment<3>(0) = Rho1[j];
            //            rhoj.segment<3>(3) = Rho2[j];
            //            rhoj.segment<3>(6) = Rho3[j];
            //            rhoj.segment<3>(9) = Rho4[j];
            //            Eigen::Matrix<double, 39, 1> residual = il_pre_integrations[j]->evaluate(Ps[i], Quaterniond(Rs[i]), Vs[i], Bas[i], Bgs[i], rhoi,
            //                                                        Ps[j], Quaterniond(Rs[j]), Vs[j], Bas[j], Bgs[j], rhoj);

            IMULegFactor *imu_leg_factor = new IMULegFactor(il_pre_integrations[j]);
            problem.AddResidualBlock(imu_leg_factor, NULL, para_Pose[i], para_SpeedBias[i], para_LegBias[i],
                                     para_Pose[j], para_SpeedBias[j], para_LegBias[j]);

            // std::vector<double *> parameter_blocks = vector<double *>{para_Pose[i], para_SpeedBias[i], para_LegBias[i],
            //                                                           para_Pose[j], para_SpeedBias[j], para_LegBias[j]};
            // std::vector<int> block_sizes = imu_leg_factor->parameter_block_sizes();
            // Eigen::VectorXd residuals; residuals.resize(imu_leg_factor->num_residuals());
            // double **raw_jacobians = new double *[block_sizes.size()];
            // std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
            // jacobians.resize(block_sizes.size());
            // for (int xx = 0; xx < static_cast<int>(block_sizes.size()); xx++)
            // {
            //     jacobians[xx].resize(imu_leg_factor->num_residuals(), block_sizes[xx]);
            //     raw_jacobians[xx] = jacobians[xx].data();
            //     //dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
            // }
            // imu_leg_factor -> Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);
            //            std::cout << "residual between frame " << i << " and " << j << std::endl;
            //            std::cout << residuals.transpose() << std::endl;
            //            imu_leg_factor -> checkJacobian(parameter_blocks.data());
        }
    }
    else if (USE_IMU)
    {
        for (int i = 0; i < frame_count; i++)
        {
            int j = i + 1;
            if (pre_integrations[j]->sum_dt > 10.0)
                continue;

            IMUFactor *imu_factor = new IMUFactor(pre_integrations[j]);
            problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
        }
    }

    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;

        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;

        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }

            if (STEREO && it_per_frame.is_stereo)
            {
                Vector3d pts_j_right = it_per_frame.pointRight;
                if (imu_i != imu_j)
                {
                    ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
                else
                {
                    ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                           it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f, loss_function, para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]);
                }
            }
            f_m_cnt++;
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    // printf("prepare for ceres: %f \n", t_prepare.toc());

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    // options.num_threads = 8;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    // options.use_explicit_schur_complement = true;
    // options.minimizer_progress_to_stdout = true;
    // options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // cout << summary.BriefReport() << endl;
    // ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    // printf("solver costs: %f \n", t_solver.toc());

    double2vector();
    // printf("frame_count: %d \n", frame_count);

    if (frame_count < WINDOW_SIZE)
        return;

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();

        if (last_marginalization_info && last_marginalization_info->valid)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0] ||
                    last_marginalization_parameter_blocks[i] == para_LegBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        if (USE_LEG && USE_IMU)
        {
            if (il_pre_integrations[1]->sum_dt < 10.0)
            {
                IMULegFactor *imu_leg_factor = new IMULegFactor(il_pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_leg_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_SpeedBias[0], para_LegBias[0],
                                                                                                para_Pose[1], para_SpeedBias[1], para_LegBias[1]},
                                                                               vector<int>{0, 1, 2});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }
        else if (USE_IMU)
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor *imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                               vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                               vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (it_per_id.used_num < 4)
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i != imu_j)
                    {
                        Vector3d pts_j = it_per_frame.point;
                        ProjectionTwoFrameOneCamFactor *f_td = new ProjectionTwoFrameOneCamFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                                                  it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                       vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    if (STEREO && it_per_frame.is_stereo)
                    {
                        Vector3d pts_j_right = it_per_frame.pointRight;
                        if (imu_i != imu_j)
                        {
                            ProjectionTwoFrameTwoCamFactor *f = new ProjectionTwoFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{0, 4});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                        else
                        {
                            ProjectionOneFrameTwoCamFactor *f = new ProjectionOneFrameTwoCamFactor(pts_i, pts_j_right, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocityRight,
                                                                                                   it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                           vector<double *>{para_Ex_Pose[0], para_Ex_Pose[1], para_Feature[feature_index], para_Td[0]},
                                                                                           vector<int>{2});
                            marginalization_info->addResidualBlockInfo(residual_block_info);
                        }
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());

        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
            addr_shift[reinterpret_cast<long>(para_LegBias[i])] = para_LegBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

        addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info && last_marginalization_info->valid)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    if (USE_IMU)
                    {
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                    }
                    if (USE_LEG)
                    {
                        addr_shift[reinterpret_cast<long>(para_LegBias[i])] = para_LegBias[i - 1];
                    }
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    if (USE_IMU)
                    {
                        addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                    }

                    if (USE_LEG)
                    {
                        addr_shift[reinterpret_cast<long>(para_LegBias[i])] = para_LegBias[i];
                    }
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];

            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];

            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
        }
    }
    // printf("whole marginalization costs: %f \n", t_whole_marginalization.toc());
    // printf("whole time for ceres: %f \n", t_whole.toc());
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Headers[i] = Headers[i + 1];
                Rs[i].swap(Rs[i + 1]);
                Ps[i].swap(Ps[i + 1]);

                if (USE_LEG && USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);
                    std::swap(il_pre_integrations[i], il_pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);
                    joint_angle_buf[i].swap(joint_angle_buf[i + 1]);
                    joint_velocity_buf[i].swap(joint_velocity_buf[i + 1]);
                    foot_contact_buf[i].swap(foot_contact_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                    Rho1[i].swap(Rho1[i + 1]);
                    Rho2[i].swap(Rho2[i + 1]);
                    Rho3[i].swap(Rho3[i + 1]);
                    Rho4[i].swap(Rho4[i + 1]);
                }

                else if (USE_IMU)
                {
                    std::swap(pre_integrations[i], pre_integrations[i + 1]);

                    dt_buf[i].swap(dt_buf[i + 1]);
                    linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                    angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                    Vs[i].swap(Vs[i + 1]);
                    Bas[i].swap(Bas[i + 1]);
                    Bgs[i].swap(Bgs[i + 1]);
                }
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];

            if (USE_LEG && USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];
                Rho1[WINDOW_SIZE] = Rho1[WINDOW_SIZE - 1];
                Rho2[WINDOW_SIZE] = Rho2[WINDOW_SIZE - 1];
                Rho3[WINDOW_SIZE] = Rho3[WINDOW_SIZE - 1];
                Rho4[WINDOW_SIZE] = Rho4[WINDOW_SIZE - 1];

                //                delete pre_integrations[WINDOW_SIZE];
                //                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                delete il_pre_integrations[WINDOW_SIZE];
                Vector_rho tmp;
                tmp.segment<RHO_OPT_SIZE>(0 * RHO_OPT_SIZE) = Rho1[WINDOW_SIZE];
                tmp.segment<RHO_OPT_SIZE>(1 * RHO_OPT_SIZE) = Rho2[WINDOW_SIZE];
                tmp.segment<RHO_OPT_SIZE>(2 * RHO_OPT_SIZE) = Rho3[WINDOW_SIZE];
                tmp.segment<RHO_OPT_SIZE>(3 * RHO_OPT_SIZE) = Rho4[WINDOW_SIZE];
                il_pre_integrations[WINDOW_SIZE] = new IMULegIntegrationBase{acc_0, gyr_0, phi_0, dphi_0, c_0,
                                                                             Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE], tmp, rho_fix_list, p_br, R_br};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
                joint_angle_buf[WINDOW_SIZE].clear();
                joint_velocity_buf[WINDOW_SIZE].clear();
                foot_contact_buf[WINDOW_SIZE].clear();
            }
            else if (USE_IMU)
            {
                Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
                Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
                Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.il_pre_integration;
                all_image_frame.erase(all_image_frame.begin(), it_0);
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];

            if (USE_LEG && USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];
                    Vector_dof tmp_joint_angle = joint_angle_buf[frame_count][i];
                    Vector_dof tmp_joint_velocity = joint_velocity_buf[frame_count][i];
                    Vector_leg tmp_foot_contact = foot_contact_buf[frame_count][i];

                    il_pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration,
                                                                    tmp_angular_velocity, tmp_joint_angle, tmp_joint_velocity, tmp_foot_contact);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                    joint_angle_buf[frame_count - 1].push_back(tmp_joint_angle);
                    joint_velocity_buf[frame_count - 1].push_back(tmp_joint_velocity);
                    foot_contact_buf[frame_count - 1].push_back(tmp_foot_contact);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];
                Rho1[frame_count - 1] = Rho1[frame_count];
                Rho2[frame_count - 1] = Rho2[frame_count];
                Rho3[frame_count - 1] = Rho3[frame_count];
                Rho4[frame_count - 1] = Rho4[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                delete il_pre_integrations[WINDOW_SIZE];
                Vector_rho tmp;
                tmp.segment<RHO_OPT_SIZE>(0 * RHO_OPT_SIZE) = Rho1[WINDOW_SIZE];
                tmp.segment<RHO_OPT_SIZE>(1 * RHO_OPT_SIZE) = Rho2[WINDOW_SIZE];
                tmp.segment<RHO_OPT_SIZE>(2 * RHO_OPT_SIZE) = Rho3[WINDOW_SIZE];
                tmp.segment<RHO_OPT_SIZE>(3 * RHO_OPT_SIZE) = Rho4[WINDOW_SIZE];
                il_pre_integrations[WINDOW_SIZE] = new IMULegIntegrationBase{acc_0, gyr_0, phi_0, dphi_0, c_0,
                                                                             Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE], tmp, rho_fix_list, p_br, R_br};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
                joint_angle_buf[WINDOW_SIZE].clear();
                joint_velocity_buf[WINDOW_SIZE].clear();
                foot_contact_buf[WINDOW_SIZE].clear();
            }
            else if (USE_IMU)
            {
                for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
                {
                    double tmp_dt = dt_buf[frame_count][i];
                    Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                    Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                    pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                    dt_buf[frame_count - 1].push_back(tmp_dt);
                    linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                    angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
                }

                Vs[frame_count - 1] = Vs[frame_count];
                Bas[frame_count - 1] = Bas[frame_count];
                Bgs[frame_count - 1] = Bgs[frame_count];

                delete pre_integrations[WINDOW_SIZE];
                pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

                dt_buf[WINDOW_SIZE].clear();
                linear_acceleration_buf[WINDOW_SIZE].clear();
                angular_velocity_buf[WINDOW_SIZE].clear();
            }
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}

void Estimator::getPoseInWorldFrame(Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[frame_count];
    T.block<3, 1>(0, 3) = Ps[frame_count];
}

void Estimator::getPoseInWorldFrame(int index, Eigen::Matrix4d &T)
{
    T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = Rs[index];
    T.block<3, 1>(0, 3) = Ps[index];
}

void Estimator::predictPtsInNextFrame()
{
    // printf("predict pts in next frame\n");
    if (frame_count < 2)
        return;
    // predict next pose. Assume constant velocity motion
    Eigen::Matrix4d curT, prevT, nextT;
    getPoseInWorldFrame(curT);
    getPoseInWorldFrame(frame_count - 1, prevT);
    nextT = curT * (prevT.inverse() * curT);
    map<int, Eigen::Vector3d> predictPts;

    for (auto &it_per_id : f_manager.feature)
    {
        if (it_per_id.estimated_depth > 0)
        {
            int firstIndex = it_per_id.start_frame;
            int lastIndex = it_per_id.start_frame + it_per_id.feature_per_frame.size() - 1;
            // printf("cur frame index  %d last frame index %d\n", frame_count, lastIndex);
            if ((int)it_per_id.feature_per_frame.size() >= 2 && lastIndex == frame_count)
            {
                double depth = it_per_id.estimated_depth;
                Vector3d pts_j = ric[0] * (depth * it_per_id.feature_per_frame[0].point) + tic[0];
                Vector3d pts_w = Rs[firstIndex] * pts_j + Ps[firstIndex];
                Vector3d pts_local = nextT.block<3, 3>(0, 0).transpose() * (pts_w - nextT.block<3, 1>(0, 3));
                Vector3d pts_cam = ric[0].transpose() * (pts_local - tic[0]);
                int ptsIndex = it_per_id.feature_id;
                predictPts[ptsIndex] = pts_cam;
            }
        }
    }
    featureTracker.setPrediction(predictPts);
    // printf("estimator output %d predict pts\n",(int)predictPts.size());
}

double Estimator::reprojectionError(Matrix3d &Ri, Vector3d &Pi, Matrix3d &rici, Vector3d &tici,
                                    Matrix3d &Rj, Vector3d &Pj, Matrix3d &ricj, Vector3d &ticj,
                                    double depth, Vector3d &uvi, Vector3d &uvj)
{
    Vector3d pts_w = Ri * (rici * (depth * uvi) + tici) + Pi;
    Vector3d pts_cj = ricj.transpose() * (Rj.transpose() * (pts_w - Pj) - ticj);
    Vector2d residual = (pts_cj / pts_cj.z()).head<2>() - uvj.head<2>();
    double rx = residual.x();
    double ry = residual.y();
    return sqrt(rx * rx + ry * ry);
}

void Estimator::outliersRejection(set<int> &removeIndex)
{
    // return;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        double err = 0;
        int errCnt = 0;
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (it_per_id.used_num < 4)
            continue;
        feature_index++;
        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;
        double depth = it_per_id.estimated_depth;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i != imu_j)
            {
                Vector3d pts_j = it_per_frame.point;
                double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                     Rs[imu_j], Ps[imu_j], ric[0], tic[0],
                                                     depth, pts_i, pts_j);
                err += tmp_error;
                errCnt++;
                // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
            }
            // need to rewrite projecton factor.........
            if (STEREO && it_per_frame.is_stereo)
            {

                Vector3d pts_j_right = it_per_frame.pointRight;
                if (imu_i != imu_j)
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                         Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                         depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
                else
                {
                    double tmp_error = reprojectionError(Rs[imu_i], Ps[imu_i], ric[0], tic[0],
                                                         Rs[imu_j], Ps[imu_j], ric[1], tic[1],
                                                         depth, pts_i, pts_j_right);
                    err += tmp_error;
                    errCnt++;
                    // printf("tmp_error %f\n", FOCAL_LENGTH / 1.5 * tmp_error);
                }
            }
        }
        double ave_err = err / errCnt;
        if (ave_err * FOCAL_LENGTH > 3)
            removeIndex.insert(it_per_id.feature_id);
    }
}

void Estimator::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
{
    double dt = t - latest_time;
    latest_time = t;
    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
    latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
    latest_V = latest_V + dt * un_acc;
    latest_acc_0 = linear_acceleration;
    latest_gyr_0 = angular_velocity;
}

void Estimator::updateLatestStates()
{
    mPropagate.lock();
    latest_time = Headers[frame_count] + td;
    latest_P = Ps[frame_count];
    latest_Q = Rs[frame_count];
    latest_V = Vs[frame_count];
    latest_Ba = Bas[frame_count];
    latest_Bg = Bgs[frame_count];
    latest_acc_0 = acc_0;
    latest_gyr_0 = gyr_0;
    mBuf.lock();
    queue<pair<double, Eigen::Vector3d>> tmp_accBuf = accBuf;
    queue<pair<double, Eigen::Vector3d>> tmp_gyrBuf = gyrBuf;
    mBuf.unlock();
    while (!tmp_accBuf.empty())
    {
        double t = tmp_accBuf.front().first;
        Eigen::Vector3d acc = tmp_accBuf.front().second;
        Eigen::Vector3d gyr = tmp_gyrBuf.front().second;
        fastPredictIMU(t, acc, gyr);
        tmp_accBuf.pop();
        tmp_gyrBuf.pop();
    }
    mPropagate.unlock();
}

void Estimator::receiveGroundTruthData(Vector3d &P, Quaterniond &Q, Vector3d &V)
{

    gt_position = P;
    gt_orientation = Q;
    gt_velocity = V;
}
