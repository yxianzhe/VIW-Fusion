/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <cmath>
#include <cassert>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

class Utility
{
  public:
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        dq.normalize();
        return dq;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        ans << typename Derived::Scalar(0), -q(2), q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }

    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
    {
        //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
        //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
        //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
        //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
        return q;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
    {
        Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
        ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
        return ans;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
    {
        Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
        ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
        return ans;
    }

    static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
    {
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        double y = atan2(n(1), n(0));
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }

    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) / 180.0 * M_PI;
        Scalar_t p = ypr(1) / 180.0 * M_PI;
        Scalar_t r = ypr(2) / 180.0 * M_PI;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        return Rz * Ry * Rx;
    }

    static Eigen::Matrix3d g2R(const Eigen::Vector3d &g);

    template <size_t N>
    struct uint_
    {
    };

    template <size_t N, typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<N>)
    {
        unroller(f, iter, uint_<N - 1>());
        f(iter + N);
    }

    template <typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<0>)
    {
        f(iter);
    }

    template <typename T>
    static T normalizeAngle(const T& angle_degrees) {
      T two_pi(2.0 * 180);
      if (angle_degrees > 0)
      return angle_degrees -
          two_pi * std::floor((angle_degrees + T(180)) / two_pi);
      else
        return angle_degrees +
            two_pi * std::floor((-angle_degrees + T(180)) / two_pi);
    };
    template <typename T>
    static Eigen::Quaternion<T> quaternionAverage(const std::vector<Eigen::Quaternion<T>>& quaternions)
    {
        if (quaternions.empty())
        {
            std::cerr << "Error trying to calculate the average quaternion of an empty set!\n";
            return Eigen::Quaternion<T>::Identity();
        }

        // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
        Eigen::Matrix<T, 4, 4> A = Eigen::Matrix<T, 4, 4>::Zero();

        for (size_t q=0; q<quaternions.size(); ++q){
            Eigen::Matrix<T, 4, 1> quat_vec = quaternions[q].coeffs();
            A = A.eval() + quat_vec * quat_vec.transpose();
        }


        // normalise with the number of quaternions
        A /= quaternions.size();

        // Compute the SVD of this 4x4 matrix
        Eigen::JacobiSVD<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

        Eigen::Matrix<T, Eigen::Dynamic, 1> singularValues = svd.singularValues();
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> U = svd.matrixU();

        // find the eigen vector corresponding to the largest eigen value
        int largestEigenValueIndex = 0;
        float largestEigenValue;
        bool first = true;

        for (size_t i=0; i<(size_t)singularValues.rows(); ++i)
        {
            if (first)
            {
                largestEigenValue = singularValues(i);
                largestEigenValueIndex = i;
                first = false;
            }
            else if (singularValues(i) > largestEigenValue)
            {
                largestEigenValue = singularValues(i);
                largestEigenValueIndex = i;
            }
        }

        Eigen::Quaternion<T> average(U(3, largestEigenValueIndex), U(0, largestEigenValueIndex), U(1, largestEigenValueIndex), U(2, largestEigenValueIndex));

        return average;
    }

    static Eigen::Vector3d logmap(const Eigen::Quaterniond &q) {
        Eigen::AngleAxisd aa(q);
        return aa.angle() * aa.axis();
    }

    static Eigen::Quaterniond expmap(const Eigen::Vector3d &w) {
        Eigen::AngleAxisd aa(w.norm(), w.stableNormalized());
        return Eigen::Quaterniond(aa);
    }

    static Eigen::Matrix3d jr(Eigen::Vector3d theta) {
        double norm = theta.norm();
        Eigen::Matrix3d jr;
        if (norm < 1.745329252e-7) {
        jr = Eigen::Matrix3d::Identity() - 1.0 / 2.0 * skewSymmetric(theta) +
            1.0 / 6.0 * skewSymmetric(theta) * skewSymmetric(theta);
        } else {
        jr = Eigen::Matrix3d::Identity() - (1.0 - cos(norm)) / (norm * norm) * skewSymmetric(theta) +
            (norm - sin(norm)) / (norm * norm * norm) * skewSymmetric(theta) * skewSymmetric(theta);
        }
        return jr;
    }

    static Eigen::Matrix3d jri(Eigen::Vector3d theta) {
        double norm = theta.norm();
        Eigen::Matrix3d jri;
        if (norm < 1.745329252e-7) {
        jri = Eigen::Matrix3d::Identity() + 1.0 / 2.0 * skewSymmetric(theta) +
                1.0 / 4.0 * skewSymmetric(theta) * skewSymmetric(theta);
        } else {
        jri = Eigen::Matrix3d::Identity() + 1.0 / 2.0 * skewSymmetric(theta) +
                ((1.0) / (norm * norm) - (1.0 + cos(norm)) / (2 * norm * sin(norm))) *
                    skewSymmetric(theta) * skewSymmetric(theta);
        }
        return jri;
    }
};
