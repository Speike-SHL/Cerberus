/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF.cpp
 *  @author Ross Hartley
 *  @brief  Source file for Invariant EKF 
 *  @date   September 25, 2018
 **/

#include "InEKF.h"

namespace inekf {

using namespace std;

void removeRowAndColumn(Eigen::MatrixXd& M, int index);

// ------------ Observation -------------
// 默认构造,单纯列表初始化Y b H N PI
Observation::Observation(Eigen::VectorXd& Y, Eigen::VectorXd& b, Eigen::MatrixXd& H, Eigen::MatrixXd& N, Eigen::MatrixXd& PI) :
    Y(Y), b(b), H(H), N(N), PI(PI) {}

/// @brief 检查观测Y中是否为空,空返回1,非空返回0 
bool Observation::empty() { return Y.rows() == 0; }

/// @brief 重载<<运算符,按格式输出 Y b H N PI 
ostream& operator<<(ostream& os, const Observation& o) {
    os << "---------- Observation ------------" << endl;
    os << "Y:\n" << o.Y << endl << endl;
    os << "b:\n" << o.b << endl << endl;
    os << "H:\n" << o.H << endl << endl;
    os << "N:\n" << o.N << endl << endl;
    os << "PI:\n" << o.PI << endl;
    os << "-----------------------------------";
    return os;  
}

// ------------ InEKF -------------
// finished主要为了保证向量大小和内存的完整定义,提升代码的健壮性,因为用的临时对象流式创建。
/// 默认构造,设置g_ = [0;0;-9.81]
InEKF::InEKF() : g_((Eigen::VectorXd(3) << 0,0,-9.81).finished()){}

/// 初始化g_ = [0;0;-9.81] 和 NoiseParams
InEKF::InEKF(NoiseParams params) : g_((Eigen::VectorXd(3) << 0,0,-9.81).finished()), noise_params_(params) {}

/// 初始化g_ = [0;0;-9.81] 和 RobotState
InEKF::InEKF(RobotState state) : g_((Eigen::VectorXd(3) << 0,0,-9.81).finished()), state_(state) {}

/// 初始化g_ = [0;0;-9.81]、 RobotState 和 NoiseParams
InEKF::InEKF(RobotState state, NoiseParams params) : g_((Eigen::VectorXd(3) << 0,0,-9.81).finished()), state_(state), noise_params_(params) {}

// Return robot's current state
RobotState InEKF::getState() { 
    return state_; 
}

// Sets the robot's current state
void InEKF::setState(RobotState state) { 
    state_ = state;
}

// Return noise params
NoiseParams InEKF::getNoiseParams() { 
    return noise_params_; 
}

// Sets the filter's noise parameters
void InEKF::setNoiseParams(NoiseParams params) { 
    noise_params_ = params; 
}

// Return filter's prior (static) landmarks
mapIntVector3d InEKF::getPriorLandmarks() { 
    return prior_landmarks_; 
}

// Set the filter's prior (static) landmarks
void InEKF::setPriorLandmarks(const mapIntVector3d& prior_landmarks) { 
    prior_landmarks_ = prior_landmarks; 
}

// Return filter's estimated landmarks
map<int,int> InEKF::getEstimatedLandmarks() { 
#if INEKF_USE_MUTEX
    lock_guard<mutex> mlock(estimated_landmarks_mutex_);
#endif
    return estimated_landmarks_; 
}

// Return filter's estimated landmarks
map<int,int> InEKF::getEstimatedContactPositions() { 
#if INEKF_USE_MUTEX
    lock_guard<mutex> mlock(estimated_contacts_mutex_);
#endif
    return estimated_contact_positions_; 
}

/// Set or Update the filter's contact state, 通过这里把contact信息添加进InEKF的contact_
void InEKF::setContacts(vector<pair<int,bool> > contacts) {
#if INEKF_USE_MUTEX
    lock_guard<mutex> mlock(estimated_contacts_mutex_);
#endif
    // Insert new measured contact states
    for (vector<pair<int,bool> >::iterator it=contacts.begin(); it!=contacts.end(); ++it) {
        // 将contacts中的(int bool)插入到contacts_中,然后插入结果赋值给ret
        // 若插入成功, ret.first的迭代器指向contacts_中新插入的(int bool), ret.second==true
        // 若插入失败, 代表contacts_中已经有相同的key了, ret.first的迭代器指向contacts_中原有的(int bool), ret.second==false
        pair<map<int,bool>::iterator,bool> ret = contacts_.insert(*it);
        // 如果插入失败,说明contacts_中已经有这个contacts的key了,则用contacts中的新value替换原有contacts_中的value
        if (ret.second==false) {
            ret.first->second = it->second;
        }
    }
    return;
}

// Return the filter's contact state
std::map<int,bool> InEKF::getContacts() {
#if INEKF_USE_MUTEX
    lock_guard<mutex> mlock(estimated_contacts_mutex_);
#endif
    return contacts_; 
}

/// InEKF Propagation - 惯性数据
void InEKF::Propagate(const Eigen::Matrix<double,6,1>& m, double dt) {

    /// 1.bias修正后的角速度和加速度 
    /// @f[
    /// \begin{array}{c}
	/// \omega _k=\omega -b_g\\
	/// a_k=a-b_a\\
    /// \end{array}
    /// @f]
    Eigen::Vector3d w = m.head(3) - state_.getGyroscopeBias();    // Angular Velocity
    Eigen::Vector3d a = m.tail(3) - state_.getAccelerometerBias(); // Linear Acceleration

    Eigen::MatrixXd X = state_.getX();
    Eigen::MatrixXd P = state_.getP();

    // Extract State
    Eigen::Matrix3d R = state_.getRotation();   //3x3
    Eigen::Vector3d v = state_.getVelocity();   //3x1
    Eigen::Vector3d p = state_.getPosition();   //3x1

    /// 2.使用捷联IMU离散运动模型预估状态
    /// @f[\begin{array}{c}
    /// 	\mathrm{R}_{\mathrm{k}+1}=\mathrm{R}_{\mathrm{k}}\mathrm{Exp}\left( \mathrm{\omega}_{\mathrm{k}}\Delta \mathrm{t} \right)\\
    /// 	\mathrm{v}_{\mathrm{k}+1}=\mathrm{v}_{\mathrm{k}}+\left( \mathrm{R}_{\mathrm{k}}\mathrm{a}_{\mathrm{k}}+\mathrm{g} \right) \Delta \mathrm{t}\\
    /// 	\mathrm{p}_{\mathrm{k}+1}=\mathrm{p}_{\mathrm{k}}+\mathrm{v}_{\mathrm{k}}\Delta \mathrm{t}+\frac{1}{2}\left( \mathrm{R}_{\mathrm{k}}\mathrm{a}_{\mathrm{k}}+\mathrm{g} \right) \Delta \mathrm{t}^2\\
    /// \end{array}@f]
    Eigen::Vector3d phi = w*dt; 
    Eigen::Matrix3d R_pred = R * Exp_SO3(phi);
    Eigen::Vector3d v_pred = v + (R*a + g_)*dt;
    Eigen::Vector3d p_pred = p + v*dt + 0.5*(R*a + g_)*dt*dt;

    // 设置新状态
    state_.setRotation(R_pred);
    state_.setVelocity(v_pred);
    state_.setPosition(p_pred);

    // ---- Linearized invariant error dynamics -----
    /// 3.计算系统矩阵A, 中间可能会有接触项,不过只用IMU更新的话没有,是15x15的.
    /// @f[\mathrm{A}_{\mathrm{k}}=\left[ \begin{matrix}
	/// 0&		0&		0&		\cdots&		-\mathrm{R}&		0\\
	/// \left( \mathrm{g} \right) _{\times}&		0&		0&		\cdots&		-\left( \mathrm{v} \right) _{\times}\mathrm{R}&		-\mathrm{R}\\
	/// 0&		\mathrm{I}&		0&		\cdots&		-\left( \mathrm{p} \right) _{\times}\mathrm{R}&		0\\
	/// \vdots&		\vdots&		\vdots&		\ddots&		\vdots&		\vdots\\
	/// 0&		0&		0&		\cdots&		0&		0\\
	/// 0&		0&		0&		\cdots&		0&		0\\
    /// \end{matrix} \right] _{\mathrm{k},  \left( 15+3\mathrm{n} \right) \times \left( 15+3\mathrm{n} \right)}@f]
    int dimX = state_.dimX();
    int dimP = state_.dimP();
    int dimTheta = state_.dimTheta();
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dimP,dimP);
    // Inertial terms
    A.block<3,3>(3,0) = skew(g_); // TODO: Efficiency could be improved by not computing the constant terms every time
    A.block<3,3>(6,3) = Eigen::Matrix3d::Identity();
    // Bias terms
    A.block<3,3>(0,dimP-dimTheta) = -R;
    A.block<3,3>(3,dimP-dimTheta+3) = -R;
    for (int i=3; i<dimX; ++i) {
        A.block<3,3>(3*i-6,dimP-dimTheta) = -skew(X.block<3,1>(0,i))*R;
    }
    // cout << A << endl;   // XXX:

    /// 4.计算过程噪声协方差矩阵Qk,中间可能会有Contact项
    /// @f[\mathrm{Q}_{\mathrm{k}}=\left[ \begin{matrix}
	/// \mathrm{Q}_{\mathrm{g}}&		0&		0&		\cdots&		0&		0\\
	/// 0&		\mathrm{Q}_{\mathrm{a}}&		0&		\cdots&		0&		0\\
	/// 0&		0&		0&		\cdots&		0&		0\\
	/// \vdots&		\vdots&		\vdots&		\ddots&		\vdots&		\vdots\\
	/// 0&		0&		0&		\cdots&		\mathrm{Q}_{\mathrm{bg}}&		0\\
	/// 0&		0&		0&		\cdots&		0&		\mathrm{Q}_{\mathrm{ba}}\\
    /// \end{matrix} \right] _{\left( 15+3\mathrm{n} \right) \times \left( 15+3\mathrm{n} \right)}@f]
    Eigen::MatrixXd Qk = Eigen::MatrixXd::Zero(dimP,dimP); // Landmark noise terms will remain zero
    Qk.block<3,3>(0,0) = noise_params_.getGyroscopeCov(); 
    Qk.block<3,3>(3,3) = noise_params_.getAccelerometerCov();
    //HACK 下面不是很懂，为什么不对estimated_landmarks_操作
    for(map<int,int>::iterator it=estimated_contact_positions_.begin(); it!=estimated_contact_positions_.end(); ++it) {
        Qk.block<3,3>(3+3*(it->second-3),3+3*(it->second-3)) = noise_params_.getContactCov(); // Contact noise terms
    }
    Qk.block<3,3>(dimP-dimTheta,dimP-dimTheta) = noise_params_.getGyroscopeBiasCov();
    Qk.block<3,3>(dimP-dimTheta+3,dimP-dimTheta+3) = noise_params_.getAccelerometerBiasCov();
    // cout << Qk << endl;  // XXX:

    /// 5.对系统误差协方差矩阵计算公式离散化并更新系统误差协方差阵
    /// @f[\begin{array}{l}
    ///	\frac{\mathrm{d}}{\mathrm{dt}}\mathrm{P}_{\mathrm{t}}=\mathrm{A}_{\mathrm{t}}\mathrm{P}_{\mathrm{t}}+\mathrm{P}_{\mathrm{t}}{\mathrm{A}_{\mathrm{t}}}^{\mathrm{T}}+\mathrm{Q}_{\mathrm{t}}\\
    /// 	\mathrm{Q}_{\mathrm{t}}=\left[ \begin{matrix}
    /// 	\mathrm{Ad}_{\mathrm{X}_{\mathrm{t}}}&		0\\
    /// 	0&		\mathrm{I}\\
    /// \end{matrix} \right] \mathrm{Cov}\left( \mathrm{w} \right) \left[ \begin{matrix}
    /// 	\mathrm{Ad}_{\mathrm{X}_{\mathrm{t}}}&		0\\
    /// 	0&		\mathrm{I}\\
    /// \end{matrix} \right] ^{\mathrm{T}}\\
    ///	 \Downarrow\\
    ///	 \mathrm{P}_{\mathrm{k}+1}=\Phi _{\mathrm{k}}\mathrm{P}_{\mathrm{k}}{\Phi _{\mathrm{k}}}^{\mathrm{T}}+\mathrm{Q}_{\mathrm{k}}\\
    ///	 \Phi _{\mathrm{k}}=\exp \left( \mathrm{A}\Delta \mathrm{t} \right) \approx \mathrm{I}+\mathrm{A}\Delta \mathrm{t}\\
    ///	 \mathrm{Q}_{\mathrm{k}}=\Phi _{\mathrm{k}}\mathrm{Q}_{\mathrm{t}}{\Phi _{\mathrm{k}}}^{\mathrm{T}}\Delta \mathrm{t}\\
    /// \end{array}@f]
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dimP,dimP);
    Eigen::MatrixXd Phi = I + A*dt; // TODO: Fast approximation of exp(A*dt). explore using the full exp() instead
    Eigen::MatrixXd Adj = I;
    Adj.block(0,0,dimP-dimTheta,dimP-dimTheta) = Adjoint_SEK3(X); // Approx 200 microseconds
    Eigen::MatrixXd PhiAdj = Phi * Adj;
    Eigen::MatrixXd Qk_hat = PhiAdj * Qk * PhiAdj.transpose() * dt; // Approximated discretized noise matrix (faster by 400 microseconds)

    // Propagate Covariance
    Eigen::MatrixXd P_pred = Phi * P * Phi.transpose() + Qk_hat;

    // Set new covariance
    state_.setP(P_pred);

    return;
}

/// Correct State: Right-Invariant Observation
/// 只是套公式完成更新过程,被 CorrectLandmarks() 和 CorrectKinematics() 函数调用
/// @f[\begin{array}{l}
/// 	\text{卡尔曼增益：S}_{\mathrm{t}}=\mathrm{H}_{\mathrm{t}}\mathrm{P}_{\mathrm{t}}{\mathrm{H}_{\mathrm{t}}}^{\mathrm{T}}+\bar{\mathrm{N}}_{\mathrm{t}}\,\,, \mathrm{K}_{\mathrm{t}}=\left[ \begin{array}{c}
/// 	\mathrm{K}_{\mathrm{t}}^{\mathrm{\xi}}\\
/// 	\mathrm{K}_{\mathrm{t}}^{\mathrm{\zeta}}\\
/// \end{array} \right] =\mathrm{P}_{\mathrm{t}}{\mathrm{H}_{\mathrm{t}}}^{\mathrm{T}}{\mathrm{S}_{\mathrm{t}}}^{-1}\\
/// 	\text{状态更新：}\left( {\bar{\mathrm{X}}_{\mathrm{t}}}^+,\bar{\mathrm{\theta}}_{\mathrm{t}}^{+} \right) =\left( \exp \left( \mathrm{K}_{\mathrm{t}}^{\mathrm{\xi}}\Pi \bar{\mathrm{X}}_{\mathrm{t}}\mathrm{Y}_{\mathrm{t}} \right) \bar{\mathrm{X}}_{\mathrm{t}}\,\,, \bar{\mathrm{\theta}}_{\mathrm{t}}+\mathrm{K}_{\mathrm{t}}^{\mathrm{\zeta}}\Pi \bar{\mathrm{X}}_{\mathrm{t}}\mathrm{Y}_{\mathrm{t}}\,\, \right)\\
/// 	\text{协方差更新：P}_{\mathrm{t}}^{+}=\left( \mathrm{I}-\mathrm{K}_{\mathrm{t}}\mathrm{H}_{\mathrm{t}} \right) \mathrm{P}_{\mathrm{t}}\left( \mathrm{I}-\mathrm{K}_{\mathrm{t}}\mathrm{H}_{\mathrm{t}} \right) ^{\mathrm{T}}+\mathrm{K}_{\mathrm{t}}\bar{\mathrm{N}}_{\mathrm{t}}\mathrm{K}_{\mathrm{t}}^{\mathrm{T}}\\
/// \end{array}@f]
void InEKF::Correct(const Observation &obs)
{
    // Compute Kalman Gain
    Eigen::MatrixXd P = state_.getP();
    Eigen::MatrixXd PHT = P * obs.H.transpose();
    Eigen::MatrixXd S = obs.H * PHT + obs.N;
    Eigen::MatrixXd K = PHT * S.inverse();

    // Copy X along the diagonals if more than one measurement
    Eigen::MatrixXd BigX;
    state_.copyDiagX(obs.Y.rows() / state_.dimX(), BigX);

    // Compute correction terms
    Eigen::MatrixXd Z = BigX * obs.Y - obs.b;
    Eigen::VectorXd delta = K * obs.PI * Z;
    Eigen::MatrixXd dX = Exp_SEK3(delta.segment(0, delta.rows() - state_.dimTheta()));
    Eigen::VectorXd dTheta = delta.segment(delta.rows() - state_.dimTheta(), state_.dimTheta());

    // Update state
    Eigen::MatrixXd X_new = dX * state_.getX(); // Right-Invariant Update
    Eigen::VectorXd Theta_new = state_.getTheta() + dTheta;
    state_.setX(X_new);
    state_.setTheta(Theta_new);

    // Update Covariance
    Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(state_.dimP(), state_.dimP()) - K * obs.H;
    Eigen::MatrixXd P_new = IKH * P * IKH.transpose() + K * obs.N * K.transpose(); // Joseph update form

    state_.setP(P_new);
}

/// Create Observation from vector of landmark measurements
/// @param[in] measured_landmarks 储存了观测到的 Landmark 列表, <id 地标相对于body的p.3x1>
void InEKF::CorrectLandmarks(const vectorLandmarks &measured_landmarks)
{
#if INEKF_USE_MUTEX
    lock_guard<mutex> mlock(estimated_landmarks_mutex_);
#endif
    Eigen::VectorXd Y;
    Eigen::VectorXd b;
    Eigen::MatrixXd H;
    Eigen::MatrixXd N;
    Eigen::MatrixXd PI;

    Eigen::Matrix3d R = state_.getRotation();
    vectorLandmarks new_landmarks;  // 新增的landmarks列表
    vector<int> used_landmark_ids;  // 储存处理过的landmark id,避免一帧数据有相同id的lm

    /// 1.循环对measured_landmarks中每一个地标观测id进行操作
    for (vectorLandmarksIterator it = measured_landmarks.begin(); it != measured_landmarks.end(); ++it)
    {
        /// 2.如果这一帧消息中包含了多个同样id的数据,则删除并跳过该帧(因为这会导致 InEKF::Correct 中出现奇点问题)
        if (find(used_landmark_ids.begin(), used_landmark_ids.end(), it->id) != used_landmark_ids.end())
        {
            cout << "Duplicate landmark ID detected! Skipping measurement.\n";
            continue;
        }
        else
        {
            used_landmark_ids.push_back(it->id);  // 将id添加到处理过的列表中,下面对此id的观测数据进行处理
        }

        /// 3.从 prior_landmarks_ 和 estimated_landmarks_ 中查找该观测id
        mapIntVector3dIterator it_prior = prior_landmarks_.find(it->id);
        map<int, int>::iterator it_estimated = estimated_landmarks_.find(it->id);
        /// 3.1 如果在 prior_landmarks_ 中找到该id, 则作为静态地标构造观测进行更新
        if (it_prior != prior_landmarks_.end())
        {
            // Found in prior landmark set
            int dimX = state_.dimX();
            int dimP = state_.dimP();
            int startIndex;

            /// - 构造Y矩阵
            /// @f[\mathrm{Y}_{\mathrm{t}}=\left[ \begin{array}{c}
            /// 	\mathrm{p}_{\mathrm{bl}}\\
            /// 	0\\
            /// 	1\\
            /// 	\vdots\\
            /// \end{array} \right] _{\mathrm{dimX}\times 1}\,\,\left( \text{若有多个观测，就是多个}Y_t\text{的垂直累加} \right)@f] 
            startIndex = Y.rows();
            Y.conservativeResize(startIndex + dimX, Eigen::NoChange);   // Y矩阵新增dimX行,不改变列数
            Y.segment(startIndex, dimX) = Eigen::VectorXd::Zero(dimX);  //初始化为0
            Y.segment(startIndex, 3) = it->position; // p_bl
            Y(startIndex + 4) = 1;

            /// - 构造b矩阵
            /// @f[\mathrm{b}=\left[ \begin{array}{c}
            /// 	\mathrm{p}_{\mathrm{wl}}\\
            /// 	0\\
            /// 	1\\
            /// 	\vdots\\
            /// \end{array} \right] _{\mathrm{dimX}\times 1}\,\,\,\,\left( \text{若有多个观测，就是多个b的垂直累加} \right)@f]
            startIndex = b.rows();
            b.conservativeResize(startIndex + dimX, Eigen::NoChange);
            b.segment(startIndex, dimX) = Eigen::VectorXd::Zero(dimX);
            b.segment(startIndex, 3) = it_prior->second; // p_wl
            b(startIndex + 4) = 1;

            /// - 构造H矩阵
            /// @f[\mathrm{H}_{\mathrm{t}}=\left[ \begin{array}{llll:ll}
            /// 	\left( \mathrm{p}_{\mathrm{wl}} \right) _{\times}&		0_{3\times 3}&		-\mathrm{I}&		\cdots&		0_{3\times 3}&		0_{3\times 3}\\
            /// \end{array} \right] _{3\times \mathrm{dimP}}\,\,\left( \begin{array}{l}
            /// 	\text{最后两个}0\text{是bias项},\text{对观测无影响，直接为}0\\
            /// 	\text{若有多个观测},\text{就是多个H}_{\mathrm{t}}\text{的垂直累加}\\
            /// \end{array} \right)@f] 
            startIndex = H.rows();
            H.conservativeResize(startIndex + 3, dimP);
            H.block(startIndex, 0, 3, dimP) = Eigen::MatrixXd::Zero(3, dimP);
            H.block(startIndex, 0, 3, 3) = skew(it_prior->second);       // skew(p_wl)
            H.block(startIndex, 6, 3, 3) = -Eigen::Matrix3d::Identity(); // -I

            /// - 构造N矩阵
            /// @f[
            /// \bar{\mathrm{N}}_{\mathrm{t}}=\bar{\mathrm{R}}_{\mathrm{t}}\mathrm{Cov}\left( \mathrm{w}^{\mathrm{l}} \right) \bar{\mathrm{R}}_{\mathrm{t}}^{\top}\,\,_{3\times 3}\,\,\left( \text{若有多个观测},\text{就是多个N阵的对角叠加} \right) 
            /// @f]
            startIndex = N.rows();
            N.conservativeResize(startIndex + 3, startIndex + 3);
            N.block(startIndex, 0, 3, startIndex) = Eigen::MatrixXd::Zero(3, startIndex);   // 下三角置0
            N.block(0, startIndex, startIndex, 3) = Eigen::MatrixXd::Zero(startIndex, 3);   // 上三角置0
            N.block(startIndex, startIndex, 3, 3) = R * noise_params_.getLandmarkCov() * R.transpose();

            /// - 构造@f$\Pi@f$矩阵
            /// @f[\Pi =\left[ \begin{matrix}
            /// 	\mathrm{I}_{3\times 3}&		0&		0&		\cdots\\
            /// \end{matrix} \right] _{3\times \mathrm{dimX}}\,\,\left( \text{若有多个观测，则对角累计}\Pi \right)@f] 
            startIndex = PI.rows();
            int startIndex2 = PI.cols();
            PI.conservativeResize(startIndex + 3, startIndex2 + dimX);
            PI.block(startIndex, 0, 3, startIndex2) = Eigen::MatrixXd::Zero(3, startIndex2);        // 从第二个观测起生效,下三角置0
            PI.block(0, startIndex2, startIndex, dimX) = Eigen::MatrixXd::Zero(startIndex, dimX);   // 从第二个观测起生效,上三角置0
            PI.block(startIndex, startIndex2, 3, dimX) = Eigen::MatrixXd::Zero(3, dimX);    // 对角位置的新PI全部置0
            PI.block(startIndex, startIndex2, 3, 3) = Eigen::Matrix3d::Identity();  // 新的PI前3x3为单位阵
        }
        /// 3.2 如果在 estimated_landmarks_ 中找到该id, 则作为动态地标构造观测进行更新
        else if (it_estimated != estimated_landmarks_.end())
        {
            // Found in estimated landmark set
            int dimX = state_.dimX();
            int dimP = state_.dimP();
            int startIndex;

            /// - 构造Y矩阵
            /// @f[\mathrm{Y}_{\mathrm{t}}=\left[ \begin{array}{c}
            /// 	\mathrm{p}_{\mathrm{bl}}\\
            /// 	0\\
            /// 	1\\
            /// 	\vdots\\
            /// 	-1\\
            /// 	\vdots\\
            /// \end{array} \right] _{\mathrm{dimX}\times 1}\,\,\left( \begin{array}{l}
            /// 	-1\text{处为动态地标id在当前状态X中的列位置},\\
            /// 	\text{若有多个观测，就是多个Y}_{\mathrm{t}}\text{的垂直累加}\\
            /// \end{array}\,\,\, \right)@f]
            startIndex = Y.rows();
            Y.conservativeResize(startIndex + dimX, Eigen::NoChange);
            Y.segment(startIndex, dimX) = Eigen::VectorXd::Zero(dimX);
            Y.segment(startIndex, 3) = it->position; // p_bl
            Y(startIndex + 4) = 1;
            Y(startIndex + it_estimated->second) = -1;

            /// - 构造b矩阵
            /// @f[\mathrm{b}=\left[ \begin{array}{c}
            /// 	0_{3\times 1}\\
            /// 	0\\
            /// 	1\\
            /// 	\vdots\\
            /// 	-1\\
            /// 	\vdots\\
            /// \end{array} \right] _{\mathrm{dimX}\times 1}\,\,\left( \begin{array}{l}
            /// 	-1\text{处为动态地标id在当前状态X中的列位置},\\
            /// 	\text{若有多个观测，就是多个b的垂直累加}\\
            /// \end{array}\,\,\, \right)@f] 
            startIndex = b.rows();
            b.conservativeResize(startIndex + dimX, Eigen::NoChange);
            b.segment(startIndex, dimX) = Eigen::VectorXd::Zero(dimX);
            b(startIndex + 4) = 1;
            b(startIndex + it_estimated->second) = -1;

            /// - 构造H矩阵
            /// @f[\mathrm{H}_{\mathrm{t}}=\left[ \begin{array}{llllll:ll}
            /// 	0_{3\times 3}&		0_{3\times 3}&		-\mathrm{I}&		\cdots&		\mathrm{I}&		\cdots&		0_{3\times 3}&		0\\
            /// \end{array}_{3\times 3} \right] _{3\times \mathrm{dimP}}\,\,\left( \begin{array}{l}
            /// 	\text{I处为动态地标id在当前状态X中的列位置},\\
            /// 	\text{最后两个}0\text{是bias项},\text{对观测无影响，直接为}0\\
            /// 	\text{若有多个观测},\text{就是多个H}_{\mathrm{t}}\text{的垂直累加}\\
            /// \end{array} \right)@f]
            startIndex = H.rows();
            H.conservativeResize(startIndex + 3, dimP);
            H.block(startIndex, 0, 3, dimP) = Eigen::MatrixXd::Zero(3, dimP);
            H.block(startIndex, 6, 3, 3) = -Eigen::Matrix3d::Identity();                           // -I
            H.block(startIndex, 3 * it_estimated->second - 6, 3, 3) = Eigen::Matrix3d::Identity(); // I

            /// - 构造N矩阵
            /// @f[\bar{\mathrm{N}}_{\mathrm{t}}=\bar{\mathrm{R}}_{\mathrm{t}}\mathrm{Cov}\left( \mathrm{w}_{\mathrm{t}}^{\mathrm{l}} \right) \bar{\mathrm{R}}_{\mathrm{t}}^{\top}\,\,_{3\times 3}\,\,\left( \text{若有多个观测},\text{就是多个N阵的对角叠加} \right)@f]
            startIndex = N.rows();
            N.conservativeResize(startIndex + 3, startIndex + 3);
            N.block(startIndex, 0, 3, startIndex) = Eigen::MatrixXd::Zero(3, startIndex);
            N.block(0, startIndex, startIndex, 3) = Eigen::MatrixXd::Zero(startIndex, 3);
            N.block(startIndex, startIndex, 3, 3) = R * noise_params_.getLandmarkCov() * R.transpose();

            /// - 构造@f$\Pi@f$矩阵
            /// @f[\Pi =\left[ \begin{matrix}
            /// 	\mathrm{I}_{3\times 3}&		0&		0&		\cdots&		0&		\cdots\\
            /// \end{matrix} \right] _{3\times \mathrm{dimX}}\,\,\left( \begin{array}{l}
            /// 	\text{后一个}0\text{是当前动态地标id在状态X中的位置}\\
            /// 	\text{若有多个观测，则对角累加}\Pi\\
            /// \end{array} \right)@f]
            startIndex = PI.rows();
            int startIndex2 = PI.cols();
            PI.conservativeResize(startIndex + 3, startIndex2 + dimX);
            PI.block(startIndex, 0, 3, startIndex2) = Eigen::MatrixXd::Zero(3, startIndex2);
            PI.block(0, startIndex2, startIndex, dimX) = Eigen::MatrixXd::Zero(startIndex, dimX);
            PI.block(startIndex, startIndex2, 3, dimX) = Eigen::MatrixXd::Zero(3, dimX);
            PI.block(startIndex, startIndex2, 3, 3) = Eigen::Matrix3d::Identity();
        }
        /// 3.3 如果是检测到的新地标(不存在于 prior_landmarks_ 和 estimated_landmarks_ 中),则添加到 new_landmarks的列表中,后续修改状态矩阵
        else
        {
            new_landmarks.push_back(*it);
        }
    }

    /// 4.当前帧观测数据处理结束,根据构造的Y b H N PI矩阵, 调用 Correct() 函数进行更新, 静态和动态地标同时构造Y, 进行更新
    /// 即每帧观测只进行一次更新过程
    Observation obs(Y, b, H, N, PI);
    if (!obs.empty())
    {
        this->Correct(obs);
    }

    /// 5.向滤波器中增加新检测到的地标, 即对 new_landmarks 列表进行操作
    /// @note 在对地标的检测中,不同于正向运动学更新,不需要从状态中删除地标
    if (new_landmarks.size() > 0)
    {
        Eigen::MatrixXd X_aug = state_.getX();
        Eigen::MatrixXd P_aug = state_.getP();
        Eigen::Vector3d p = state_.getPosition();
        // 循环对每个新增动态地标操作
        for (vectorLandmarksIterator it = new_landmarks.begin(); it != new_landmarks.end(); ++it)
        {
            /// - 扩充X, 论文公式(31)
            /// @f[\mathrm{X} = \left[
            /// \begin{array}{cccc}
            /// \mathrm{R} & \mathrm{v} & \mathrm{p} & \cdots \\
            /// 0          & 1          & 0          & 0      \\
            /// 0          & 0          & 1          & 0      \\
            /// 0          & 0          & 0          & 1      \\
            /// \end{array}
            /// \right]_{dim \mathrm{X}}
            /// \Longrightarrow
            /// \left[
            /// \begin{array}{cccc:c}
            /// \mathrm{R} & \mathrm{v} & \mathrm{p} & \cdots & \mathrm{p}_{\mathrm{t}}+\mathrm{R}_{\mathrm{t}}\mathrm{p}_{\mathrm{bl}} \\
            /// 0          & 1          & 0          & 0      & 0 \\
            /// 0          & 0          & 1          & 0      & 0 \\
            /// 0          & 0          & 0          & 1      & 0 \\
            /// \hdashline
            /// 0          & 0          & 0          & 0      & 1 \\
            /// \end{array}
            /// \right]_{dim \mathrm{X}+1}@f]
            int startIndex = X_aug.rows();
            X_aug.conservativeResize(startIndex + 1, startIndex + 1);   // 大小加1
            X_aug.block(startIndex, 0, 1, startIndex) = Eigen::MatrixXd::Zero(1, startIndex);   //下三角置0
            X_aug.block(0, startIndex, startIndex, 1) = Eigen::MatrixXd::Zero(startIndex, 1);   //上三角置0
            X_aug(startIndex, startIndex) = 1;  //新增对角置1
            X_aug.block(0, startIndex, 3, 1) = p + R * it->position;

            /// - 更新误差协方差矩阵P, 初始化新地标的协方差, 论文公式(32)
            /// @todo speed up
            /// @f[\left[ \begin{array}{c}
            /// 	\xi _{t}^{R}\\
            /// 	\xi _{t}^{v}\\
            /// 	\xi _{t}^{p}\\
            /// 	\vdots\\
            /// 	\hdashline
            /// 	\xi _{t}^{l}\\
            /// 	\hdashline
            /// 	\zeta _{t}^{g}\\
            /// 	\zeta _{t}^{a}\\
            /// \end{array} \right] _{\mathrm{dimP}+3}=\left[ \begin{array}{cccc:cc}
            /// 	\mathrm{I}&		0&		0&		\cdots&		0&		0\\
            /// 	0&		\mathrm{I}&		0&		\cdots&		0&		0\\
            /// 	0&		0&		\mathrm{I}&		\cdots&		0&		0\\
            /// 	\vdots&		\vdots&		\vdots&		\mathrm{I}&		0&		0\\
            /// 	\hdashline
            /// 	0&		0&		\mathrm{I}&		0&		0&		0\\
            /// 	\hdashline
            /// 	0&		0&		0&		0&		\mathrm{I}&		0\\
            /// 	0&		0&		0&		0&		0&		\mathrm{I}\\
            /// \end{array} \right] _{\left( \mathrm{dimP}+3 \right) \times \mathrm{dimP}}\left[ \begin{array}{c}
            /// 	\xi _{t}^{R}\\
            /// 	\xi _{t}^{v}\\
            /// 	\xi _{t}^{p}\\
            /// 	\vdots\\
            /// 	\hdashline
            /// 	\zeta _{t}^{g}\\
            /// 	\zeta _{t}^{a}\\
            /// \end{array} \right] _{\mathrm{dimP}}+\left[ \begin{array}{c}
            /// 	0\\
            /// 	0\\
            /// 	0\\
            /// 	0\\
            /// 	\hdashline
            /// 	\bar{\mathrm{R}}_{\mathrm{t}} \\
            /// 	\hdashline
            /// 	0\\
            /// 	0\\
            /// \end{array} \right] \mathrm{w}_{\mathrm{t}}^{l}@f]
            /// @f[\xi _{\mathrm{t}}^{\mathrm{new}}\triangleq \mathrm{F}_{\mathrm{t}}\xi _{\mathrm{t}}+\mathrm{G}_{\mathrm{t}}\mathrm{w}_{\mathrm{t}}^{l}\Longrightarrow \mathrm{P}_{\mathrm{t}}^{\mathrm{new}}=\mathrm{F}_{\mathrm{t}}\mathrm{P}_{\mathrm{t}}\mathrm{F}_{\mathrm{t}}^{\top}+\mathrm{G}_{\mathrm{t}}\mathrm{Cov}\left( \mathrm{w}_{\mathrm{t}}^{l} \right) \mathrm{G}_{\mathrm{t}}^{\top}@f]
            Eigen::MatrixXd F = Eigen::MatrixXd::Zero(state_.dimP() + 3, state_.dimP());    // F阵
            F.block(0, 0, state_.dimP() - state_.dimTheta(), state_.dimP() - state_.dimTheta()) = Eigen::MatrixXd::Identity(state_.dimP() - state_.dimTheta(), state_.dimP() - state_.dimTheta());     // for old X
            F.block(state_.dimP() - state_.dimTheta(), 6, 3, 3) = Eigen::Matrix3d::Identity();   // for new landmark
            F.block(state_.dimP() - state_.dimTheta() + 3, state_.dimP() - state_.dimTheta(), state_.dimTheta(), state_.dimTheta()) = Eigen::MatrixXd::Identity(state_.dimTheta(), state_.dimTheta()); // for theta
            Eigen::MatrixXd G = Eigen::MatrixXd::Zero(F.rows(), 3);     // G阵
            G.block(G.rows() - state_.dimTheta() - 3, 0, 3, 3) = R;
            P_aug = (F * P_aug * F.transpose() + G * noise_params_.getLandmarkCov() * G.transpose()).eval(); //.eval的作用是让Eigen显示计算链式矩阵运算，并立刻返回结果，避免大型链式计算中可能存在的错误发生

            // 更新状态和协方差
            state_.setX(X_aug);
            state_.setP(P_aug);

            /// - 把新增的动态地标状态的<id, 在X中的索引>放入 estimated_landmarks_
            estimated_landmarks_.insert(pair<int, int>(it->id, startIndex));
        }
    }
    return;
}

/// Correct state using kinematics measured between imu and contact point
/// @brief 使用一个或多个正向运动学观测信息更新Y b H N PI矩阵, 并调用 Correct() 方法更新状态. 然后判断腿是否触地，向X中增加和删除腿的状态量.
/// @param[in] measured_kinematics 传入的内容为观测的一个或多个 Kinematics(id pose4x4 covariance6x6) 列表
void InEKF::CorrectKinematics(const vectorKinematics &measured_kinematics)
{
#if INEKF_USE_MUTEX
    lock_guard<mutex> mlock(estimated_contacts_mutex_);
#endif
    Eigen::VectorXd Y;
    Eigen::VectorXd b;
    Eigen::MatrixXd H;
    Eigen::MatrixXd N;
    Eigen::MatrixXd PI;

    Eigen::Matrix3d R = state_.getRotation();
    vector<pair<int, int>> remove_contacts;  // 要删除的腿列表,<id,id在X中的索引>
    vectorKinematics new_contacts;  // 要新增的腿列表
    vector<int> used_contact_ids;   // 已处理过的contact id列表

    /// 1.循环对measured_kinematics中每一个正向运动学观测进行操作
    for (vectorKinematicsIterator it = measured_kinematics.begin(); it != measured_kinematics.end(); ++it)
    {
        /// 2.如果这一帧消息中包含了多个同样id的数据,则删除并跳过该帧(因为这会导致 InEKF::Correct 中出现奇点问题)
        if (find(used_contact_ids.begin(), used_contact_ids.end(), it->id) != used_contact_ids.end())
        {
            cout << "Duplicate contact ID detected! Skipping measurement.\n";
            continue;
        }
        else
        {
            used_contact_ids.push_back(it->id); // 记录已使用的id(下面将要处理该id)
        }

        /// 3.从 InEKF::contact_ 中查找当前要处理的运动学数据的id. 如果不存在,则说明状态未知,跳过;若存在,则用contact_indicated记录其状态,
        map<int, bool>::iterator it_contact = contacts_.find(it->id);
        if (it_contact == contacts_.end())
        {
            continue;
        } 
        bool contact_indicated = it_contact->second;

        /// 4.查看是否能够从 estimated_contact_positions_ (已经加入状态估计的腿id)中找到该id
        map<int, int>::iterator it_estimated = estimated_contact_positions_.find(it->id); // 该id在estimated_contact_positions_中的迭代器
        bool found = it_estimated != estimated_contact_positions_.end();

        /// 4.1 如果找到该id且该id的状态为未触地,则添加该id在 estimated_contact_positions_ 中的迭代器到 remove_contacts (删除腿列表)
        if (!contact_indicated && found)
        {
            remove_contacts.push_back(*it_estimated); 
        }
        /// 4.2 如果没找到该id但该id的状态为触地,则添加该id指向的 Kinematics(id pose4x4 covariance6x6) 对象到 new_contacts (新增腿列表)
        else if (contact_indicated && !found)
        {
            new_contacts.push_back(*it);
        }
        /// 4.3 如果找到该id且该id状态仍为触地,则用正向运动学进行更新
        else if (contact_indicated && found)
        {
            int dimX = state_.dimX();
            int dimP = state_.dimP();
            int startIndex;

            /// - 构造Y矩阵
            /// @f[\mathrm{Y}_{\mathrm{t}}=\left[ \begin{array}{c}
            /// 	\mathrm{h}_{\mathrm{p}}\left( \tilde{\mathrm{\alpha}}_{\mathrm{t}} \right)\\
            /// 	0\\
            /// 	1\\
            /// 	\vdots\\
            /// 	-1\\
            /// 	\vdots\\
            /// \end{array} \right] _{\mathrm{dimX}\times 1}\,\,\left( \begin{array}{l}
            /// 	\text{-1处为腿id在当前状态X中的列位置},\\
            /// 	\text{若有多个观测，就是多个}Y_t\text{的垂直累加}\\
            /// \end{array} \right)@f]
            startIndex = Y.rows();  // 开始索引,多个观测时,Y中已经有数据了
            Y.conservativeResize(startIndex + dimX, Eigen::NoChange);  // Y矩阵新增dimX行,不改变列数
            Y.segment(startIndex, dimX) = Eigen::VectorXd::Zero(dimX);
            Y.segment(startIndex, 3) = it->pose.block<3, 1>(0, 3); // p_bc
            Y(startIndex + 4) = 1;
            Y(startIndex + it_estimated->second) = -1;  // it_estimated代表当前估计的腿在状态X中的列位置

            /// - 构造b矩阵
            /// @f[\mathrm{b}=\left[ \begin{array}{c}
            /// 	0_{3\times 1}\\
            /// 	0\\
            /// 	1\\
            /// 	\vdots\\
            /// 	-1\\
            /// 	\vdots\\
            /// \end{array} \right] _{\mathrm{dimX}\times 1}\,\,\left( \begin{array}{l}
            /// 	\text{-1处为腿id在当前状态X中的列位置},\\
            /// 	\text{若有多个观测，就是多个b的垂直累加}\\
            /// \end{array} \right)@f] 
            startIndex = b.rows();
            b.conservativeResize(startIndex + dimX, Eigen::NoChange);
            b.segment(startIndex, dimX) = Eigen::VectorXd::Zero(dimX);
            b(startIndex + 4) = 1;
            b(startIndex + it_estimated->second) = -1;

            /// - 构造H矩阵
            /// @f[\mathrm{H}_{\mathrm{t}}=\left[ \begin{array}{llllll:ll}
            /// 	0_{3\times 3}&		0_{3\times 3}&		-\mathrm{I}&		\cdots&		\mathrm{I}&		\cdots&		0_{3\times 3}&		0\\
            /// \end{array}_{3\times 3} \right] _{3\times \mathrm{dimP}}\,\,\left( \begin{array}{l}
            /// 	\text{I处为腿id在当前状态X中的列位置},\\
            /// 	\text{最后两个}0\text{是bias项},\text{对观测无影响，直接为}0\\
            /// 	\text{若有多个观测},\text{就是多个H}_{\mathrm{t}}\text{的累加}\\
            /// \end{array} \right)@f]
            startIndex = H.rows();
            H.conservativeResize(startIndex + 3, dimP);
            H.block(startIndex, 0, 3, dimP) = Eigen::MatrixXd::Zero(3, dimP);
            H.block(startIndex, 6, 3, 3) = -Eigen::Matrix3d::Identity();                           // -I
            H.block(startIndex, 3 * it_estimated->second - 6, 3, 3) = Eigen::Matrix3d::Identity(); // I

            /// - 构造N矩阵
            /// @f[\bar{\mathrm{N}}_{\mathrm{t}}=\bar{\mathrm{R}}_{\mathrm{t}}\mathrm{J}_{\mathrm{p}}\left( \mathrm{\alpha}_{\mathrm{t}} \right) \mathrm{Cov}\left( \mathrm{w}_{\mathrm{t}}^{\mathrm{\alpha}} \right) \mathrm{J}_{\mathrm{p}}^{\top}\left( \mathrm{\alpha}_{\mathrm{t}} \right) \bar{\mathrm{R}}_{\mathrm{t}}^{\top}\,\,_{3\times 3}\,\, \left( \text{若有多个观测}, \text{就是多个N阵的对角叠加} \right)@f]             
            startIndex = N.rows();
            N.conservativeResize(startIndex + 3, startIndex + 3);
            N.block(startIndex, 0, 3, startIndex) = Eigen::MatrixXd::Zero(3, startIndex);  //从第二个观测起生效,下三角置0
            N.block(0, startIndex, startIndex, 3) = Eigen::MatrixXd::Zero(startIndex, 3);  //从第二个观测起生效,上三角置0
            N.block(startIndex, startIndex, 3, 3) = R * it->covariance.block<3, 3>(3, 3) * R.transpose();   //对角放新的3X3的N矩阵

            /// - 构造@f$\Pi@f$矩阵
            /// @f[\Pi =\left[ \begin{matrix}
            /// 	\mathrm{I}_{3\times 3}&		0&		0&		\cdots&		0&		\cdots\\
            /// \end{matrix} \right] _{3\times \mathrm{dimX}}\,\, \left( \begin{array}{l}
            /// 	\text{后一个}0\text{是当前腿id在状态X中的位置}\\
            /// 	\text{若有多个观测，则对角累计}\Pi\\
            /// \end{array} \right)@f]
            startIndex = PI.rows();
            int startIndex2 = PI.cols();
            PI.conservativeResize(startIndex + 3, startIndex2 + dimX);
            PI.block(startIndex, 0, 3, startIndex2) = Eigen::MatrixXd::Zero(3, startIndex2);    //从第二个观测起生效,下三角置0
            PI.block(0, startIndex2, startIndex, dimX) = Eigen::MatrixXd::Zero(startIndex, dimX);   //从第二个观测起生效,上三角置0
            PI.block(startIndex, startIndex2, 3, dimX) = Eigen::MatrixXd::Zero(3, dimX);    //对角放新的PI全部置0
            PI.block(startIndex, startIndex2, 3, 3) = Eigen::Matrix3d::Identity();  //新的PI前3x3为单位阵
        }
        /// 4.4 如果没找到该id且该id状态为未触地,则跳过
        else
        {
            continue;
        }
    }

    /// 5.循环结束,观测数据处理完成. 用前面构造的 Y b H N PI 构造一个 Observation 对象, 然后调用 Correct() 函数进行更新
    Observation obs(Y, b, H, N, PI);
    if (!obs.empty())
    {
        this->Correct(obs);
    }

    /// 6.从状态中移除未触地的腿
    if (remove_contacts.size() > 0)
    {
        Eigen::MatrixXd X_rem = state_.getX(); 
        Eigen::MatrixXd P_rem = state_.getP();
        for (vector<pair<int, int>>::iterator it = remove_contacts.begin(); it != remove_contacts.end(); ++it)
        {
            /// - 在 estimated_contact_positions_ 中删除腿id
            estimated_contact_positions_.erase(it->first);

            /// - 在X中直接删除腿id对应的行和列
            removeRowAndColumn(X_rem, it->second);

            /// - 从P中直接删除腿id对应的行和列
            int startIndex = 3 + 3 * (it->second - 3);
            removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
            removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient
            removeRowAndColumn(P_rem, startIndex); // TODO: Make more efficient

            /// - 因为X中删除了一个状态量,所以需要更新 estimated_landmarks 和 estimated_contact_positions 中id对应的X中的位置
            for (map<int, int>::iterator it2 = estimated_landmarks_.begin(); it2 != estimated_landmarks_.end(); ++it2)
            {
                if (it2->second > it->second)
                    it2->second -= 1;
            }
            for (map<int, int>::iterator it2 = estimated_contact_positions_.begin(); it2 != estimated_contact_positions_.end(); ++it2)
            {
                if (it2->second > it->second)
                    it2->second -= 1;
            }
            /// - 同时因为X中删除了一个状态量,所以需要删除 remove_contacts 列表中记录的id对应的X中的位置
            for (vector<pair<int, int>>::iterator it2 = it; it2 != remove_contacts.end(); ++it2)
            {
                if (it2->second > it->second)
                    it2->second -= 1;
            }

            state_.setX(X_rem);
            state_.setP(P_rem);
        }
    }

    /// 7.向状态中增加新触地的腿
    if (new_contacts.size() > 0)
    {
        Eigen::MatrixXd X_aug = state_.getX();
        Eigen::MatrixXd P_aug = state_.getP();
        Eigen::Vector3d p = state_.getPosition();
        // 循环对每个新增腿进行操作
        for (vectorKinematicsIterator it = new_contacts.begin(); it != new_contacts.end(); ++it)
        {
            /// - 修改X矩阵, 论文公式(31)
            /// @f[\mathrm{X} = \left[
            /// \begin{array}{cccc}
            /// \mathrm{R} & \mathrm{v} & \mathrm{p} & \cdots \\
            /// 0          & 1          & 0          & 0      \\
            /// 0          & 0          & 1          & 0      \\
            /// 0          & 0          & 0          & 1      \\
            /// \end{array}
            /// \right]_{dim \mathrm{X}}
            /// \Longrightarrow
            /// \left[
            /// \begin{array}{cccc:c}
            /// \mathrm{R} & \mathrm{v} & \mathrm{p} & \cdots & \mathrm{p}_{\mathrm{t}}+\mathrm{R}_{\mathrm{t}} \mathrm{h}_{\mathrm{p}}\left(\alpha_{\mathrm{t}}\right) \\
            /// 0          & 1          & 0          & 0      & 0 \\
            /// 0          & 0          & 1          & 0      & 0 \\
            /// 0          & 0          & 0          & 1      & 0 \\
            /// \hdashline
            /// 0          & 0          & 0          & 0      & 1 \\
            /// \end{array}
            /// \right]_{dim \mathrm{X}+1}@f]
            int startIndex = X_aug.rows();
            X_aug.conservativeResize(startIndex + 1, startIndex + 1);   //将X大小扩充1
            X_aug.block(startIndex, 0, 1, startIndex) = Eigen::MatrixXd::Zero(1, startIndex);   //新增下三角部分置0
            X_aug.block(0, startIndex, startIndex, 1) = Eigen::MatrixXd::Zero(startIndex, 1);   //新增上三角部分置0
            X_aug(startIndex, startIndex) = 1;  //新增对角线部分置1
            X_aug.block(0, startIndex, 3, 1) = p + R * it->pose.block<3, 1>(0, 3);  // 更新腿的状态

            /// - 更新误差协方差矩阵P，论文公式(32)
            /// @todo speed up
            /// @f[\left[ \begin{array}{c}
            /// 	\xi _{t}^{R}\\
            /// 	\xi _{t}^{v}\\
            /// 	\xi _{t}^{p}\\
            /// 	\vdots\\
            /// 	\hdashline
            /// 	\xi _{t}^{d}\\
            /// 	\hdashline
            /// 	\zeta _{t}^{g}\\
            /// 	\zeta _{t}^{a}\\
            /// \end{array} \right] _{\mathrm{dimP}+3}=\left[ \begin{array}{cccc:cc}
            /// 	\mathrm{I}&		0&		0&		\cdots&		0&		0\\
            /// 	0&		\mathrm{I}&		0&		\cdots&		0&		0\\
            /// 	0&		0&		\mathrm{I}&		\cdots&		0&		0\\
            /// 	\vdots&		\vdots&		\vdots&		\mathrm{I}&		0&		0\\
            /// 	\hdashline
            /// 	0&		0&		\mathrm{I}&		0&		0&		0\\
            /// 	\hdashline
            /// 	0&		0&		0&		0&		\mathrm{I}&		0\\
            /// 	0&		0&		0&		0&		0&		\mathrm{I}\\
            /// \end{array} \right] _{\left( \mathrm{dimP}+3 \right) \times \mathrm{dimP}}\left[ \begin{array}{c}
            /// 	\xi _{t}^{R}\\
            /// 	\xi _{t}^{v}\\
            /// 	\xi _{t}^{p}\\
            /// 	\vdots\\
            /// 	\hdashline
            /// 	\zeta _{t}^{g}\\
            /// 	\zeta _{t}^{a}\\
            /// \end{array} \right] _{\mathrm{dimP}}+\left[ \begin{array}{c}
            /// 	0\\
            /// 	0\\
            /// 	0\\
            /// 	0\\
            /// 	\hdashline
            /// 	\bar{\mathrm{R}}_{\mathrm{t}}\mathrm{J}_{\mathrm{p}}\left( \bar{\alpha}_{\mathrm{t}} \right)\\
            /// 	\hdashline
            /// 	0\\
            /// 	0\\
            /// \end{array} \right] \mathrm{w}_{\mathrm{t}}^{\alpha}@f]
            /// @f[\xi _{\mathrm{t}}^{\mathrm{new}}\triangleq \mathrm{F}_{\mathrm{t}}\xi _{\mathrm{t}}+\mathrm{G}_{\mathrm{t}}\mathrm{w}_{\mathrm{t}}^{\alpha}\Longrightarrow \mathrm{P}_{\mathrm{t}}^{\mathrm{new}}=\mathrm{F}_{\mathrm{t}}\mathrm{P}_{\mathrm{t}}\mathrm{F}_{\mathrm{t}}^{\top}+\mathrm{G}_{\mathrm{t}}\mathrm{Cov}\left( \mathrm{w}_{\mathrm{t}}^{\alpha} \right) \mathrm{G}_{\mathrm{t}}^{\top}@f]
            Eigen::MatrixXd F = Eigen::MatrixXd::Zero(state_.dimP() + 3, state_.dimP());    // F阵
            F.block(0, 0, state_.dimP() - state_.dimTheta(), state_.dimP() - state_.dimTheta()) = Eigen::MatrixXd::Identity(state_.dimP() - state_.dimTheta(), state_.dimP() - state_.dimTheta());     // for old X
            F.block(state_.dimP() - state_.dimTheta(), 6, 3, 3) = Eigen::Matrix3d::Identity();                                                                                                         // for new landmark
            F.block(state_.dimP() - state_.dimTheta() + 3, state_.dimP() - state_.dimTheta(), state_.dimTheta(), state_.dimTheta()) = Eigen::MatrixXd::Identity(state_.dimTheta(), state_.dimTheta()); // for theta
            Eigen::MatrixXd G = Eigen::MatrixXd::Zero(F.rows(), 3);     // G阵
            G.block(G.rows() - state_.dimTheta() - 3, 0, 3, 3) = R;
            P_aug = (F * P_aug * F.transpose() + G * it->covariance.block<3, 3>(3, 3) * G.transpose()).eval(); //.eval的作用是让Eigen显示计算链式矩阵运算，并立刻返回结果，避免大型链式计算中可能存在的错误发生

            // Update state and covariance
            state_.setX(X_aug);
            state_.setP(P_aug);

            /// - 把新增的腿状态的<id, 在X中的索引>放入 estimated_contact_positions_
            estimated_contact_positions_.insert(pair<int, int>(it->id, startIndex));
        }
    }

    return;
}

/// 删除矩阵M的第index行和index列,从零开始
void removeRowAndColumn(Eigen::MatrixXd& M, int index) {
    unsigned int dimX = M.cols();
    // cout << "Removing index: " << index<< endl;
    // 把矩阵index行下面的dim-index-1行用最后dim-index-1行代替
    M.block(index,0,dimX-index-1,dimX) = M.bottomRows(dimX-index-1).eval();
    // 把矩阵index列右边的dim-index-1列用最右边的dim-index-1列代替
    M.block(0,index,dimX,dimX-index-1) = M.rightCols(dimX-index-1).eval();
    // 然后把矩阵重新resize
    M.conservativeResize(dimX-1,dimX-1);
}

} // end inekf namespace
