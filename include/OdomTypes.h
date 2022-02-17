//
// Created by vuong on 2/10/22.
//

#ifndef ORB_SLAM3_ODOMTYPES_H
#define ORB_SLAM3_ODOMTYPES_H

#include <Eigen/Core>
#include "sophus/se2.hpp"
#include "sophus/se3.hpp"
namespace ORB_SLAM3 {
namespace ODOM {
class Meas {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  Meas(const Sophus::SE2f& meas_, const double& timestamp)
      : meas(meas_), t(timestamp) {}

 public:
  Sophus::SE2f meas;
  double t;
};
class Preintegrated {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Preintegrated(Eigen::Vector3d& vNoises, Sophus::SE3f& Tbo_);
  Preintegrated(Preintegrated* pOdomPre);
  void IntegratedNewMeasurement(Sophus::SE2f& PrevMeas, Sophus::SE2f& CurrMeas);
  void MergePrevious(ODOM::Preintegrated* pPrev);
 public:
  Sophus::SE2f Meas;
  Eigen::Matrix3f Cov, Cov1;
  double NoiseX, NoiseY, NoiseRotZ;
  Sophus::SE3f Tbo;
  Eigen::Vector3f Delta;
  std::vector<Sophus::SE2f> mvMeasurements;
};
}  // namespace ODOM
}  // namespace ORB_SLAM3
#endif  // ORB_SLAM3_ODOMTYPES_H
