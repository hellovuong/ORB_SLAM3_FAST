//
// Created by vuong on 2/10/22.
//

#include "OdomTypes.h"
#include "sophus/se3.hpp"
namespace ORB_SLAM3 {
namespace ODOM {
Preintegrated::Preintegrated(Eigen::Vector3d& vNoises, Sophus::SE3f& Tbo_)
    : NoiseX(vNoises.x()),
      NoiseY(vNoises.y()),
      NoiseRotZ(vNoises.z()),
      Tbo(Tbo_) {
  Meas = Sophus::SE2f(Eigen::Rotation2Df::Identity().toRotationMatrix(),
                      Eigen::Vector2f::Zero());
  Cov.setZero();
  Cov1.setZero();
  Delta.setZero();
  mvMeasurements.clear();
}
void Preintegrated::IntegratedNewMeasurement(Sophus::SE2f& PrevMeas,
                                             Sophus::SE2f& CurrMeas) {
  mvMeasurements.push_back(PrevMeas);
  Sophus::SE2f delta = PrevMeas.inverse() * CurrMeas;

  Eigen::Matrix2f Phi_ik = Meas.rotationMatrix();
  Meas = Meas * delta;

  Delta.head(2) += Eigen::Rotation2Df(Delta.z()) * delta.translation();
  Delta.z() += delta.so2().log();

  Eigen::Matrix3f Ak = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f Bk = Eigen::Matrix3f::Identity();

  Eigen::Matrix3f Ak1 = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f Bk1 = Eigen::Matrix3f::Identity();

  Ak.block<2, 1>(0, 2) = Phi_ik * Eigen::Vector2f(-delta.translation().y(),
                                                  delta.translation().x());
  Bk.block<2, 2>(0, 0) = Phi_ik;

  Ak1.block<2, 1>(1, 0) = Phi_ik * Eigen::Vector2f(-delta.translation().y(),
                                                   delta.translation().x());
  Bk1.block<2, 2>(1, 1) = Phi_ik;

  Eigen::Matrix3f Sigmak = Cov;
  Eigen::Matrix3f Sigma_vk = Eigen::Matrix3f::Identity();

  Eigen::Matrix3f Sigmak1 = Cov1;
  Eigen::Matrix3f Sigma_vk1 = Eigen::Matrix3f::Identity();

  Sigma_vk(0, 0) = float(NoiseX * NoiseX);
  Sigma_vk(1, 1) = float(NoiseY * NoiseY);
  Sigma_vk(2, 2) = float(NoiseRotZ * NoiseRotZ);

  Sigma_vk1(0, 0) = float(NoiseRotZ * NoiseRotZ);
  Sigma_vk1(1, 1) = float(NoiseX * NoiseX);
  Sigma_vk1(2, 2) = float(NoiseY * NoiseY);

  Eigen::Matrix3f Sigma_k_1 =
      Ak * Sigmak * Ak.transpose() + Bk * Sigma_vk * Bk.transpose();
  Cov = Sigma_k_1;

  Eigen::Matrix3f Sigma_k_1_1 =
      Ak1 * Sigmak1 * Ak1.transpose() + Bk1 * Sigma_vk1 * Bk1.transpose();
  Cov1 = Sigma_k_1_1;
}
Preintegrated::Preintegrated(Preintegrated* pOdomPre)
    : Meas(pOdomPre->Meas),
      Cov(pOdomPre->Cov),
      Cov1(pOdomPre->Cov1),
      NoiseX(pOdomPre->NoiseX),
      NoiseY(pOdomPre->NoiseY),
      NoiseRotZ(pOdomPre->NoiseRotZ),
      Tbo(pOdomPre->Tbo),
      Delta(pOdomPre->Delta),
      mvMeasurements(pOdomPre->mvMeasurements) {}
}  // namespace ODOM
}  // namespace ORB_SLAM3