# include "eskf.h"

bool eskfEstimator::Init(const Mat3d& RIO, const Vec3d& TIO, const Vec3d& gravity, 
                         const Mat3d& R0, const Vec3d& t0, const double& time0, 
                         const Vec3d& v0, const Vec3d& ba, const Vec3d& bw) {
    eskfEstimator::Options options;
    options.V = VEL_N_wheel * Mat3d::Identity();
    options.Q.block<3, 3>(0, 0) = ACC_N * Mat3d::Identity();
    options.Q.block<3, 3>(3, 3) = GYR_N * Mat3d::Identity();
    options.rio = RIO;
    options.tio = TIO;
    options.g = gravity;
    options.ba = ba;
    options.bw = bw;
    options.origin_R = R0;
    options.origin_t = t0;
    options_ = options;
    eskfEstimator::State state;
    state.timestamp = time0;
    state.p = t0;
    state.v = v0;
    state.r = R0;
    state_ = state;
    return true;
}

void eskfEstimator::InsertImu(const pair<double, Vec3d>& cur_acc, const pair<double, Vec3d>& cur_gyr) {
    if(!first_imu){
        state_.acc = cur_acc.second;
        state_.gyr = cur_gyr.second;
        first_imu = true;
    }
    Propagate(cur_acc.first, cur_acc.second, cur_gyr.second);
}

void eskfEstimator::InsertWheel(const pair<double, Vec3d>& cur_vel) {
    if(!first_imu)
        return;
    Vec3d acc1 = state_.acc;
    Vec3d gyr1 = state_.gyr;
    // 推算到wheel时刻
    Propagate(cur_vel.first, acc1, gyr1);
    Vec3d vel_wheel(cur_vel.second.x(), 0., 0.);
    Update(vel_wheel);
}

void eskfEstimator::Propagate(const double& cur_time, const Vec3d& acc1, const Vec3d& gyr1) {
    // state_ 的状态量
    double dt = cur_time - state_.timestamp;
    double dt2 = dt * dt;
    Vec3d last_acc = state_.r * (state_.acc - options_.ba) - options_.g;
    Mat3d last_r = state_.r;
    Vec3d delta_angle_axis = (0.5 * (state_.gyr + gyr1) - options_.bw) * dt;
    state_.r = state_.r * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized());
    Vec3d new_acc = state_.r * (acc1 - options_.ba) - options_.g;
    Vec3d ave_acc = 0.5 * (last_acc + new_acc);
    state_.p = state_.p + state_.v * dt + 0.5 * ave_acc * dt2;
    state_.v = state_.v + ave_acc * dt;

    // state_ 的cov 用eskf误差传递
    Mat9d Fx = Mat9d::Identity();
    Fx.block<3, 3>(0, 3) = dt * Mat3d::Identity();
    Fx.block<3, 3>(3, 6) = last_r * Utility::skewSymmetric(state_.acc - options_.ba) * dt;
    Fx.block<3, 3>(6, 6) = Utility::expmap(-delta_angle_axis).toRotationMatrix();

    Eigen::Matrix<double, 9, 6> Fi = Eigen::Matrix<double, 9, 6>::Zero();
    Fi.block<6, 6>(3, 0) = dt * Mat6d::Identity();

    state_.cov = Fx * state_.cov.eval() * Fx.transpose() + Fi * options_.Q * Fi.transpose();
    state_.cov = 0.5 * state_.cov + 0.5 * state_.cov.transpose().eval();
    state_.cov.diagonal() = state_.cov.diagonal().cwiseAbs();

    // 更新读数
    state_.acc = acc1;
    state_.gyr = gyr1;
    state_.timestamp = cur_time;
}

void eskfEstimator::Update(const Vec3d& vel_wheel) {
    Mat9d eskf_cov = state_.cov;
    Vec3d vel_world = state_.r * options_.rio * vel_wheel;

    // residual
    Vec3d res = vel_world - state_.v;
    // jacobian H/x * x/dx
    Eigen::Matrix<double, 3, 9> H;
    H.setZero();
    H.block<3, 3>(0, 3) = Mat3d::Identity();
    
    Mat3d S = H * eskf_cov * H.transpose() + options_.V;
    Eigen::Matrix<double, 9, 3> K = eskf_cov * H.transpose() * S.inverse();
    Mat9d G = Mat9d::Identity() - K * H;

    Vec9d delta_x = K * res;

    eskf_cov = G * eskf_cov * G.transpose() + K * options_.V * K.transpose();
    eskf_cov = 0.5 * eskf_cov.eval() + 0.5 * eskf_cov.transpose().eval();
    eskf_cov.diagonal() = eskf_cov.diagonal().cwiseAbs();

    // update
    state_.p += delta_x.segment<3>(0);
    state_.v += delta_x.segment<3>(3);
    state_.r = state_.r * Utility::expmap(delta_x.segment<3>(6));
    state_.cov = eskf_cov;
}

void eskfEstimator::GetResult(double& time, Mat3d& next_R, Vec3d& next_t) {
    time = state_.timestamp;
    next_R = state_.r;
    next_t = state_.p;
}