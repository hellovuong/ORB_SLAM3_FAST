//
// Created by vuong on 2/20/22.
//

#ifndef ORB_SLAM3_WODOMETRYOPTIMIZATION_H
#define ORB_SLAM3_WODOMETRYOPTIMIZATION_H

#include "Frame.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Map.h"
#include "MapPoint.h"
#include "OdomFactor.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

namespace ORB_SLAM3 {
class WOdometryOptimization {
 public:
  // Online calibration
  void static OnlineExtrinsic(const std::vector<KeyFrame*>& vpKFs,
                              Sophus::SE3f& Extr);
  // Full Inertial (Global BA) Optimization
  void static FullInertialBANew(Map* pMap, int its, bool bFixLocal = false,
                                unsigned long nLoopKF = 0,
                                bool* pbStopFlag = nullptr, bool bInit = false,
                                float priorG = 1e2, float priorA = 1e6,
                                Eigen::VectorXd* vSingVal = nullptr,
                                bool* bHess = nullptr,
                                PlanarConstraint* pPlanarConstraint = nullptr);
  // Local Inertial BA
  void static LocalInertialBANew(KeyFrame* pKF, bool* pbStopFlag, Map* pMap,
                                 int& num_fixedKF, int& num_OptKF, int& num_MPs,
                                 int& num_edges, bool bLarge = false,
                                 bool bRecInit = false,
                                 PlanarConstraint* pPlanarConstraint = nullptr);
  // Motion-only BA
  int static PoseInertialOptimizationLastKeyFrame(
      Frame* pFrame, PlanarConstraint* pPlanarConstraint = nullptr,
      bool bRecInit = false);
  int static PoseInertialOptimizationLastFrame(
      Frame* pFrame, PlanarConstraint* pPlanarConstraint = nullptr,
      bool bRecInit = false);

  // Marginalize block element (start:end,start:end). Perform Schur complement.
  // Marginalized elements are filled with zeros.
  static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd& H, const int& start,
                                     const int& end);

};
}  // namespace ORB_SLAM3
#endif  // ORB_SLAM3_WODOMETRYOPTIMIZATION_H
