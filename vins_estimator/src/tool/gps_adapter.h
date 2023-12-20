#pragma once

#include <vector>
using namespace std;

#include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>

class CoordConverter
{
  public:
    CoordConverter(){is_init_ = false;}

    void SetReferencePoint(const Eigen::Vector3d& llh_point, const Eigen::Vector3d& enu_point);
    void ConvertToENU(const Eigen::Vector3d& llh_point, Eigen::Vector3d& enu_point);
    void ConvertToENU(const std::vector<Eigen::Vector3d>& llh_points,
                      std::vector<Eigen::Vector3d>& enu_points);
    void Forward(const Eigen::Vector3d& llh_point, Eigen::Vector3d& enu_point);
    void Wgs84ToEcef(const Eigen::Vector3d& llh_point, Eigen::Vector3d& ecef_point,
                     Eigen::Matrix3d* enu_rot_ecef);
    bool is_init_ = false;
    
  private:
    Eigen::Vector3d ref_llh_point_;
    Eigen::Vector3d ref_enu_point_;
    
    Eigen::Vector3d ref_ecef_point_;
    Eigen::Matrix3d ref_enu_rot_ecef_;

};





