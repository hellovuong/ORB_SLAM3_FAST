/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CONVERTER_H
#define CONVERTER_H

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "Thirdparty/Sophus/sophus/geometry.hpp"
#include "Thirdparty/Sophus/sophus/sim3.hpp"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

namespace ORB_SLAM3 {

class Converter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static std::vector<cv::Mat> toDescriptorVector(const cv::Mat& Descriptors);

  static Eigen::Matrix<float, 3, 3> toMatrix3f(const cv::Mat& cvMat3);

  /**
   * \brief Method to find the average of a set of rotation quaternions using
   *Singular Value Decomposition
   *
   * The algorithm used is described here:
   * https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
   * @param vQuaternions: set of consicutive Quaternoins
   **/
  static Eigen::Quaternion<float> QuaternionAvg(
      std::vector<Eigen::Quaternion<float>>& vQuaternions);

  // TODO: Sophus migration, to be deleted in the future
  static Sophus::SE3<float> toSophus(const cv::Mat& T);
  static Sophus::Sim3f toSophus(const g2o::Sim3& S);
};

}  // namespace ORB_SLAM3

#endif  // CONVERTER_H
