//
// Created by vuong on 2/16/22.
//
#include "OdomFactor.h"
namespace ORB_SLAM3 {
Eigen::Vector3d vNoise(0.1, 0.1, 0.1);
EdgeWOdometry::EdgeWOdometry(ODOM::Preintegrated* pInt) : mpInt(pInt) {
  // Extrinsic matrix
  Tbo = mpInt->Tbo.cast<double>();
  // Set information matrix
  setInformation(pInt->Cov1.cast<double>().inverse());
}
void EdgeWOdometry::computeError() {
  const auto* VP1 = dynamic_cast<const VertexPose*>(_vertices[0]);
  const auto* VP2 = dynamic_cast<const VertexPose*>(_vertices[1]);

  const Eigen::Matrix3d Rbo = Tbo.rotationMatrix();
  const Eigen::Vector3d tbo = Tbo.translation();

  const double dr = mpInt->Delta.cast<double>().z();
  const Eigen::Vector2d dp = mpInt->Delta.head(2).cast<double>();

  const double er =
      e3.transpose() *
          Sophus::SO3d(Eigen::Quaterniond(Rbo.transpose() *
                                          VP1->estimate().Rwb.transpose() *
                                          VP2->estimate().Rwb * Rbo)
                           .normalized())
              .log() -
      dr;

  const Eigen::Vector2d ep =
      lambd * (VP1->estimate().Rwb * Rbo).transpose() *
          (VP2->estimate().Rwb * tbo + VP2->estimate().twb -
           VP1->estimate().Rwb * tbo - VP1->estimate().twb) -
      dp;

  _error << er, ep;
}
void EdgeWOdometry::linearizeOplus() {
  const auto* VP1 = dynamic_cast<const VertexPose*>(_vertices[0]);
  const auto* VP2 = dynamic_cast<const VertexPose*>(_vertices[1]);

  const Eigen::Matrix3d Rbo = Tbo.rotationMatrix();
  const Eigen::Vector3d tbo = Tbo.translation();
  const Eigen::Matrix3d Rob = Rbo.transpose();

  const Eigen::Matrix3d Rwb1 = VP1->estimate().Rwb;
  const Eigen::Matrix3d Rbw1 = Rwb1.transpose();
  const Eigen::Vector3d twb1 = VP1->estimate().twb;

  const Eigen::Matrix3d Rwb2 = VP2->estimate().Rwb;
  const Eigen::Vector3d twb2 = VP2->estimate().twb;

  const Eigen::Matrix3d tmp = (Rbw1 * Rwb2 * Rbo).transpose();

  const Eigen::Vector3d deltaR =
      Sophus::SO3d(Eigen::Quaterniond(Rob * Rbw1 * VP2->estimate().Rwb * Rbo)
                       .normalized())
          .log();
  const Eigen::Matrix3d invJr;  // = InverseRightJacobianSO3(deltaR);
  Sophus::rightJacobianInvSO3(deltaR, invJr);
  _jacobianOplusXi.setZero();
  // Jacobian wrt Pose1
  // rotation
  _jacobianOplusXi.block<1, 3>(0, 0) = -e3.transpose() * invJr * tmp;  // OK
  //    _jacobianOplusXi.block<2, 3>(1, 0) =
  //        lambd * Rbo.transpose() *
  //        Skew(Rbw1 * (VP2->estimate().twb + Rwb2 * tbo -
  //                     VP1->estimate().twb));  // TODO: Check this
  _jacobianOplusXi.block<2, 3>(1, 0) =
      lambd * (Rwb1 * Rbo).transpose() * (Rwb1 * Sophus::SO3d::hat(tbo)) +
      lambd * Rbo.transpose() *
          Sophus::SO3d::hat(Rwb1.transpose() *
                            (Rwb2 * tbo + twb2 - Rwb1 * tbo - twb1));
  // translation
  _jacobianOplusXi.block<1, 3>(0, 3) = Eigen::Vector3d::Zero().transpose();
  _jacobianOplusXi.block<2, 3>(1, 3) = -lambd * (Rwb1 * Rbo).transpose();
  if (_jacobianOplusXi.maxCoeff() > 1e8 || _jacobianOplusXi.minCoeff() < -1e8) {
    std::cout << "numerical unstable in preintegration" << std::endl;
  }
  _jacobianOplusXj.setZero();
  // Jacobian wrt Pose2
  // rotation
  _jacobianOplusXj.block<1, 3>(0, 0) = e3.transpose() * invJr * Rob;
  _jacobianOplusXj.block<2, 3>(1, 0) =
      -lambd * Rob * Rbw1 * Rwb2 * Sophus::SO3d::hat(tbo);
  // translation
  _jacobianOplusXj.block<1, 3>(0, 3) = Eigen::Vector3d::Zero().transpose();
  _jacobianOplusXj.block<2, 3>(1, 3) = lambd * Rob * Rbw1 * Rwb2;
  if (_jacobianOplusXj.maxCoeff() > 1e8 || _jacobianOplusXj.minCoeff() < -1e8) {
    std::cerr << "numerical unstable in preintegration" << std::endl;
  }
}
EdgePlane::EdgePlane(Sophus::SE3f& Ref, Eigen::Vector3d& Noise)
    : Twb_orig(Ref.cast<double>()) {
  Eigen::Matrix3d Info =
      Eigen::Vector3d(1 / Noise.z(), 1 / Noise.x(), 1 / Noise.y()).asDiagonal();
  setInformation(Info);
}
EdgePlane::EdgePlane(Sophus::SE3f& Ref) : Twb_orig(Ref.cast<double>()) {
  Eigen::Matrix3d Info =
      Eigen::Vector3d(1 / vNoise.z(), 1 / vNoise.x(), 1 / vNoise.y())
          .asDiagonal();
  setInformation(Info);
}
void EdgePlane::computeError() {
  const auto* VP = dynamic_cast<const VertexPose*>(_vertices[0]);
  const Sophus::SE3d Twbi(Eigen::Quaterniond(VP->estimate().Rwb).normalized(),
                          VP->estimate().twb);

  const Sophus::SE3d E = Twb_orig.inverse() * Twbi;
  const Eigen::Vector2d er_xy = lambd * E.so3().log();
  const double ep_z = e3.transpose() * E.translation();

  _error << er_xy, ep_z;
}
void EdgePlane::linearizeOplus() {
  const auto* VP = dynamic_cast<const VertexPose*>(_vertices[0]);

  const Sophus::SE3d Twbi(Eigen::Quaterniond(VP->estimate().Rwb).normalized(),
                          VP->estimate().twb);
  const Sophus::SE3d E = Twb_orig.inverse() * Twbi;

  const Eigen::Matrix3d invJr;  // = InverseRightJacobianSO3(deltaR);
  Sophus::rightJacobianInvSO3(E.so3().log(), invJr);

  _jacobianOplusXi.setZero();
  _jacobianOplusXi.block<2, 3>(0, 0) = lambd * invJr;
  _jacobianOplusXi.block<1, 3>(2, 0) =
      e3.transpose() * Twb_orig.rotationMatrix();
}
}  // namespace ORB_SLAM3