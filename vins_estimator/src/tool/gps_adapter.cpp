#include "gps_adapter.h"


void CoordConverter::SetReferencePoint(const Eigen::Vector3d& llh_point,
                                       const Eigen::Vector3d& enu_point) {
  ref_llh_point_ = llh_point;
  ref_enu_point_ = enu_point;
  Wgs84ToEcef(ref_llh_point_, ref_ecef_point_, &ref_enu_rot_ecef_);
  is_init_ = true;
}

void CoordConverter::ConvertToENU(const Eigen::Vector3d& llh_point, Eigen::Vector3d& enu_point) {
  Forward(llh_point, enu_point);
}

void CoordConverter::ConvertToENU(const std::vector<Eigen::Vector3d>& llh_points,
                                  std::vector<Eigen::Vector3d>& enu_points) {
  enu_points.resize(llh_points.size());
  for (size_t i = 0; i < enu_points.size(); ++i) {
    Forward(llh_points[i], enu_points[i]);
  }
}

void CoordConverter::Forward(const Eigen::Vector3d& llh_point, Eigen::Vector3d& enu_point) {
  Eigen::Vector3d ecef_point;
  Wgs84ToEcef(llh_point, ecef_point, &ref_enu_rot_ecef_);

  Eigen::Vector3d enu_offset = ref_enu_rot_ecef_ * (ecef_point - ref_ecef_point_);
  enu_point = ref_enu_point_ + enu_offset;
}

void CoordConverter::Wgs84ToEcef(const Eigen::Vector3d& llh_point, Eigen::Vector3d& ecef_point,
                                 Eigen::Matrix3d* enu_rot_ecef) {
  // WGS 84 Earth Parameters.
  constexpr double kf = 1.0 / 298.257223563;
  constexpr double kRe = 6378137.0;
  constexpr double ke2 = kf * (2.0 - kf);
  const double ke = std::sqrt(ke2);
  constexpr double kDegToRad = M_PI / 180.0;

  // Limit the input.
  const double longitude = std::max(std::min(llh_point.x(), 180.0), -180.0);
  const double latitude = std::max(std::min(llh_point.y(), 90.0), -90.0);
  const double height = std::max(std::min(llh_point.z(), 10000.0), -1000.0);

  // Pre-computations.
  const double cos_lat = std::cos(latitude * kDegToRad);
  const double sin_lat = std::sin(latitude * kDegToRad);
  const double cos_lon = std::cos(longitude * kDegToRad);
  const double sin_lon = std::sin(longitude * kDegToRad);

  // Compute Rn
  const double sin_lat2 = sin_lat * sin_lat;
  const double sq = 1. - ke2 * sin_lat2;
  const double Rn = kRe / std::max(std::sqrt(std::max(sq, 0.0)), 1e-14);

  // WGS84 point to ECEF point.
  ecef_point.x() = (Rn + height) * cos_lat * cos_lon;
  ecef_point.y() = (Rn + height) * cos_lat * sin_lon;
  ecef_point.z() = (Rn * (1.0 - ke2) + height) * sin_lat;

  // Rotation matrix form ECEF to ENU
  if (enu_rot_ecef != nullptr) {
    *enu_rot_ecef << -sin_lon, cos_lon, 0.0, -cos_lon * sin_lat, -sin_lon * sin_lat, cos_lat,
        cos_lon * cos_lat, sin_lon * cos_lat, sin_lat;
  }
}



