#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "parameters.h"
#include "../utility/utility.h"

using Vec3d = Eigen::Vector3d;
using Vec9d = Eigen::Matrix<double, 9, 1>;
using Mat3d = Eigen::Matrix3d;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Mat9d = Eigen::Matrix<double, 9, 9>;
using Quat = Eigen::Quaterniond;

class eskfEstimator
{
  public:
    struct Options
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Mat6d Q = Mat6d::Zero(); // 运动方程噪声
        Mat3d V = Mat3d::Zero(); // 观测方程噪声
        Mat3d rio = Mat3d::Identity(); // 外参
        Vec3d tio = Vec3d::Identity(); // 外参
        Vec3d g = Vec3d::Identity(); // 重力
        Vec3d ba = Vec3d::Identity();
        Vec3d bw = Vec3d::Identity();
        Mat3d origin_R = Mat3d::Identity(); //初始位姿
        Vec3d origin_t = Vec3d::Identity();
    };
    
    struct State
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        double timestamp = 0;

        Vec3d p = Vec3d::Zero();
        Vec3d v = Vec3d::Zero();
        Mat3d r = Mat3d::Identity();

        Vec3d acc = Vec3d::Zero();
        Vec3d gyr = Vec3d::Zero();

        Mat9d cov = Mat9d::Zero();
    };

  public:
    bool Init(const Mat3d& RIO, const Vec3d& TIO, const Vec3d& gravity,
              const Mat3d& R0, const Vec3d& t0, const double& time0,
              const Vec3d& v0, const Vec3d& ba, const Vec3d& bw);

    void InsertImu(const pair<double, Vec3d>& cur_acc, const pair<double, Vec3d>& cur_gyr);
    void InsertWheel(const pair<double, Vec3d>& cur_vel);

    void Propagate(const double& cur_time, const Vec3d& acc1, const Vec3d& gyr1);
    void Update(const Vec3d& vel_wheel);

    void GetResult(double& time, Mat3d& next_R, Vec3d& next_t);

    bool first_imu = false;
  private:
    Options options_;
    State state_;
    
};