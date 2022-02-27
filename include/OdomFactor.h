//
// Created by vuong on 2/16/22.
//

#ifndef ORB_SLAM3_ODOMFACTOR_H
#define ORB_SLAM3_ODOMFACTOR_H

#include <Frame.h>
#include <KeyFrame.h>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include "Converter.h"
#include "G2oTypes.h"

#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/types/types_sba.h"

namespace ORB_SLAM3 {
typedef Eigen::Matrix<double, 2, 3> Matrix23d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

const static Matrix23d lambd = Eigen::Matrix3d::Identity().block<2, 3>(0, 0);

const static Eigen::Vector3d e3 = Eigen::Vector3d(0, 0, 1);

class PlanarConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  PlanarConstraint() = default;
  PlanarConstraint(const Sophus::SO3f& qpw_, double zpw_)
      : qpw(qpw_), zpw(zpw_){};
  PlanarConstraint(const std::vector<Sophus::SE3f>& vPoses);
  PlanarConstraint(PlanarConstraint* pPlanarConstraint);
 public:
  void setQpw(const Sophus::SO3f& qpw_) { PlanarConstraint::qpw = qpw_; }
  void setZpw(double zpw_) { PlanarConstraint::zpw = zpw_; }
  const Sophus::SO3f& getQpw() const { return qpw; }
  double getZpw() const { return zpw; }

  void Update(const double* pu);

 private:
  Sophus::SO3f qpw;
  double zpw;
};
class VertexPlanar : public g2o::BaseVertex<4, PlanarConstraint> {
  // rotation around z and translation in xy are optimizable variables
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexPlanar() = default;
  VertexPlanar(PlanarConstraint* pPlanarConstraint);
  virtual bool read(std::istream& is) { return true; }
  virtual bool write(std::ostream& os) const { return true; }

  virtual void setToOriginImpl() {}

  virtual void oplusImpl(const double* update_) {
    _estimate.Update(update_);
    updateCache();
  }
};
class EdgeWOdometry
    : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexPose, VertexPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeWOdometry() = default;
  EdgeWOdometry(ODOM::Preintegrated* pInt);
  void computeError();
  virtual void linearizeOplus();
  virtual bool read(std::istream& is) { return true; }
  virtual bool write(std::ostream& os) const { return true; }

  ODOM::Preintegrated* mpInt{};
  Sophus::SE3d Tbo;
};
class EdgeWOdometryExtr : public g2o::BaseMultiEdge<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeWOdometryExtr() = default;
  EdgeWOdometryExtr(ODOM::Preintegrated* pInt);
  void computeError();
  virtual void linearizeOplus();
  virtual bool read(std::istream& is) { return true; }
  virtual bool write(std::ostream& os) const { return true; }

  ODOM::Preintegrated* mpInt{};
};
class EdgePlane
    : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexPose, VertexPlanar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgePlane();
  void computeError();
  virtual void linearizeOplus();
  virtual bool read(std::istream& is) { return true; }
  virtual bool write(std::ostream& os) const { return true; }
  Sophus::SE3d Twb_orig;
  Sophus::SE3d Tbo;
};
}  // namespace ORB_SLAM3
#endif  // ORB_SLAM3_ODOMFACTOR_H
