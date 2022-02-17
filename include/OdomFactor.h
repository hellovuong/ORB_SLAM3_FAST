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
// clang-format off
const static Matrix23d lambd = (Matrix23d()
                                    << 1, 0, 0,
                                       0, 1, 0).finished();
// clang-format on
const static Eigen::Vector3d e3 = (Eigen::Vector3d() << 0, 0, 1).finished();
class EdgeWOdometry
    : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, VertexPose, VertexPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeWOdometry() = default;
  EdgeWOdometry(ODOM::Preintegrated* pInt);
  virtual void computeError();
  virtual void linearizeOplus();
  virtual bool read(std::istream& is) { return true; }
  virtual bool write(std::ostream& os) const { return true; }

  ODOM::Preintegrated* mpInt{};
  Sophus::SE3d Tbo;
};
class EdgePlane : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgePlane() = default;
  EdgePlane(Sophus::SE3f& Ref, Eigen::Vector3d& Noise);
  EdgePlane(Sophus::SE3f& Ref);
  virtual void computeError();
  virtual void linearizeOplus();
  virtual bool read(std::istream& is) { return true; }
  virtual bool write(std::ostream& os) const { return true; }
  Sophus::SE3d Twb_orig;
};
}  // namespace ORB_SLAM3
#endif  // ORB_SLAM3_ODOMFACTOR_H
