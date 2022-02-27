//
// Created by vuong on 2/16/22.
//
#include "OdomFactor.h"
namespace ORB_SLAM3 {
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
void EdgePlane::computeError() {
  const auto* VP = static_cast<const VertexPose*>(_vertices[0]);
  const auto* VPlanar = static_cast<const VertexPlanar*>(_vertices[1]);

  const Sophus::SE3d Twbi = Sophus::SE3d(
      Eigen::Quaterniond(VP->estimate().Rwb).normalized(), VP->estimate().twb);
  const Sophus::SO3d Rpw = VPlanar->estimate().getQpw();
  const Eigen::Vector2d er = lambd * Tbo.rotationMatrix().transpose() *
                             Twbi.rotationMatrix().transpose() *
                             Rpw.matrix().transpose() * e3;
  const double ep = VPlanar->estimate().getZpw() +
                    (Rpw.matrix() * (Twbi.translation() +
                                     Twbi.rotationMatrix() * Tbo.translation()))
                        .z();
  _error << er, ep;
}
void EdgePlane::linearizeOplus() {
  const auto* VP = static_cast<const VertexPose*>(_vertices[0]);
  const auto* VPlanar = static_cast<const VertexPlanar*>(_vertices[1]);
  const Sophus::SE3d Twbi = Sophus::SE3d(
      Eigen::Quaterniond(VP->estimate().Rwb).normalized(), VP->estimate().twb);
  const Sophus::SO3d Rpw = VPlanar->estimate().getQpw();

  //  const Eigen::Matrix3d invJr;
  //
  //  Sophus::rightJacobianInvSO3(
  //      Sophus::SO3d(Eigen::Quaterniond(Tbo.rotationMatrix().transpose() *
  //                                      Twbi.rotationMatrix().transpose() *
  //                                      Rpw.matrix().transpose())
  //                       .normalized())
  //          .log(),
  //      invJr);
  // jacobian wrt body pose
  _jacobianOplusXi.setZero();
  // rotation
  // d_er_phi_i
  _jacobianOplusXi.block<2, 3>(0, 0) =
      lambd * Tbo.rotationMatrix().transpose() *
      Sophus::SO3d::hat(Twbi.rotationMatrix().transpose() *
                        Rpw.matrix().transpose() * e3);
  //  _jacobianOplusXi.block<2, 3>(0, 0) =
  //      -lambd * invJr * Rpw.matrix() * Twbi.rotationMatrix();
  // d_ep_phi_i
  _jacobianOplusXi.block<1, 3>(2, 0) = -e3.transpose() * Rpw.matrix() *
                                       Twbi.rotationMatrix() *
                                       Sophus::SO3d::hat(Tbo.translation());
  // translation
  // d_er_p_i = 0;
  // d_ep_p_i
  _jacobianOplusXi.block<1, 3>(2, 3) = e3.transpose() * Rpw.matrix();
  //  std::cout << " Jacobian: Jxi: \n" << _jacobianOplusXi << std::endl;
  // jacobian wrt planar constraints
  _jacobianOplusXj.setZero();
  // rotaion
  // d_er_phi_pw
  _jacobianOplusXj.block<2, 3>(0, 0) =
      lambd * Tbo.rotationMatrix().transpose() *
      Twbi.rotationMatrix().transpose() *
      Sophus::SO3d::hat(Rpw.matrix().transpose() * e3);
  //  _jacobianOplusXj.block<2, 3>(0, 0) = -lambd * invJr * Rpw.matrix();
  // d_ep_phi_pw
  _jacobianOplusXj.block<1, 3>(2, 0) =
      -e3.transpose() * Rpw.matrix() *
      Sophus::SO3d::hat(Twbi.translation() +
                        Twbi.rotationMatrix() * Tbo.translation());
  // translation
  // d_er_p_pw = 0
  // d_ep_p_pw = 1
  _jacobianOplusXj(2, 3) = 1.0;
}
EdgePlane::EdgePlane()
    : g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexPose, VertexPlanar>() {
  Eigen::Vector3d vInfo;
  // TODO: Check noise
  Eigen::Vector3d vNoise(0.01, 0.01, 0.05);
  // clang-format off
    vInfo <<  1 / vNoise.z(), 1 / vNoise.x(), 1 / vNoise.y();
  // clang-format on
  setInformation(vInfo.asDiagonal());
}

EdgeWOdometryExtr::EdgeWOdometryExtr(ODOM::Preintegrated* pInt) : mpInt(pInt) {
  resize(3);
  // Set information matrix
  setInformation(pInt->Cov1.cast<double>().inverse());
}
void EdgeWOdometryExtr::computeError() {
  const auto* VP1 = static_cast<const VertexPose*>(_vertices[0]);
  const auto* VP2 = static_cast<const VertexPose*>(_vertices[1]);
  const auto* VPExtr = static_cast<const g2o::VertexSE3Expmap*>(_vertices[2]);

  const Eigen::Matrix3d Rbo = VPExtr->estimate().rotation().matrix();
  const Eigen::Vector3d tbo = VPExtr->estimate().translation();

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
void EdgeWOdometryExtr::linearizeOplus() {
  const auto* VP1 = static_cast<const VertexPose*>(_vertices[0]);
  const auto* VP2 = static_cast<const VertexPose*>(_vertices[1]);
  const auto* VPExtr = static_cast<const g2o::VertexSE3Expmap*>(_vertices[2]);

  const Eigen::Matrix3d Rbo = VPExtr->estimate().rotation().matrix();
  const Eigen::Vector3d tbo = VPExtr->estimate().translation();
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

  const Eigen::Matrix3d invJr;
  Sophus::rightJacobianInvSO3(deltaR, invJr);

  // Jacobian wrt Pose1
  _jacobianOplus[0].setZero();
  // rotation
  _jacobianOplus[0].block<1, 3>(0, 0) = -e3.transpose() * invJr * tmp;  // OK

  _jacobianOplus[0].block<2, 3>(1, 0) =
      lambd * (Rwb1 * Rbo).transpose() * (Rwb1 * Sophus::SO3d::hat(tbo)) +
      lambd * Rbo.transpose() *
          Sophus::SO3d::hat(Rwb1.transpose() *
                            (Rwb2 * tbo + twb2 - Rwb1 * tbo - twb1));
  // translation
  _jacobianOplus[0].block<1, 3>(0, 3) = Eigen::Vector3d::Zero().transpose();
  _jacobianOplus[0].block<2, 3>(1, 3) = -lambd * (Rwb1 * Rbo).transpose();
  if (_jacobianOplus[0].maxCoeff() > 1e8 ||
      _jacobianOplus[0].minCoeff() < -1e8) {
    std::cout << "numerical unstable in preintegration" << std::endl;
  }

  // Jacobian wrt Pose2
  _jacobianOplus[1].setZero();
  // rotation
  _jacobianOplus[1].block<1, 3>(0, 0) = e3.transpose() * invJr * Rob;
  _jacobianOplus[1].block<2, 3>(1, 0) =
      -lambd * Rob * Rbw1 * Rwb2 * Sophus::SO3d::hat(tbo);
  // translation
  _jacobianOplus[1].block<1, 3>(0, 3) = Eigen::Vector3d::Zero().transpose();
  _jacobianOplus[1].block<2, 3>(1, 3) = lambd * Rob * Rbw1 * Rwb2;
  if (_jacobianOplus[1].maxCoeff() > 1e8 ||
      _jacobianOplus[1].minCoeff() < -1e8) {
    std::cerr << "numerical unstable in preintegration" << std::endl;
  }

  // Jacobian wrt Extr
  _jacobianOplus[2].setZero();
  // rotation
  _jacobianOplus[2].block<1, 3>(0, 0) =
      e3.transpose() * invJr *
      (Eigen::Matrix3d::Identity() -
       Sophus::SO3d(Eigen::Quaterniond(Rob * Rbw1 * VP2->estimate().Rwb * Rbo)
                        .normalized())
           .matrix());
  _jacobianOplus[2].block<2, 3>(1, 0) =
      lambd *
      Sophus::SO3d::hat((VP1->estimate().Rwb * Rbo).transpose() *
                        (VP2->estimate().Rwb * tbo + VP2->estimate().twb -
                         VP1->estimate().Rwb * tbo - VP1->estimate().twb));
  // translation
  _jacobianOplus[2].block<2, 3>(0, 3) =
      lambd * (Rwb1 * Rbo).transpose() * (Rwb2 - Rbw1);
}

PlanarConstraint::PlanarConstraint(const std::vector<Sophus::SE3f>& vTwo) {
  zpw = 0;
  if (vTwo.empty()) {
    cerr << "Empty vector Wheel Odometry " << std::endl;
    return;
  }
  double zpw_ = 0.0;
  std::vector<Eigen::Quaternionf> qRws;
  for (const auto& Pose : vTwo) {
    qRws.emplace_back(Pose.unit_quaternion().inverse());
    zpw_ += -(Pose.rotationMatrix() * Pose.translation()).z();
  }
  // Quaternion avg
  Eigen::Quaternionf QuaterninonAvg = Converter::QuaternionAvg(qRws);
  QuaterninonAvg.normalize();
  Sophus::SO3f spQuaAvg(QuaterninonAvg);
  spQuaAvg = Sophus::SO3f::exp(
      Eigen::Vector3f(spQuaAvg.log().x(), spQuaAvg.log().y(), 0));

  setQpw(spQuaAvg);
  // Z avg
  setZpw(zpw_ / static_cast<double>(vTwo.size()));

  std::cout << "Init plane: qpw: " << getQpw().log().transpose()
            << " zqw: " << zpw << std::endl;
}
void PlanarConstraint::Update(const double* pu) {
  const Sophus::SO3d dR =
      Sophus::SO3d::exp(Eigen::Vector3d(pu[0], pu[1], pu[2])).matrix();
  zpw += (qpw.matrix() * Eigen::Vector3f(0, 0, static_cast<float>(pu[3]))).z();
  qpw = qpw * dR.cast<float>();
}
PlanarConstraint::PlanarConstraint(PlanarConstraint* pPlanarConstraint) {
  setZpw(pPlanarConstraint->getZpw());
  setQpw(pPlanarConstraint->getQpw());
}
VertexPlanar::VertexPlanar(PlanarConstraint* pPlanarConstraint) {
  setEstimate(PlanarConstraint(pPlanarConstraint));
}
}  // namespace ORB_SLAM3