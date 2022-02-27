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

#include "Converter.h"

namespace ORB_SLAM3 {

std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat& Descriptors) {
  std::vector<cv::Mat> vDesc;
  vDesc.reserve(Descriptors.rows);
  for (int j = 0; j < Descriptors.rows; j++)
    vDesc.push_back(Descriptors.row(j));

  return vDesc;
}

Eigen::Matrix<float, 3, 3> Converter::toMatrix3f(const cv::Mat& cvMat3) {
  Eigen::Matrix<float, 3, 3> M;
  // clang-format off
//  M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
//      cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
//      cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);
  // clang-format on
  cv::cv2eigen(cvMat3, M);
  return M;
}

Sophus::SE3<float> Converter::toSophus(const cv::Mat& T) {
  Eigen::Matrix4d eigMat;
  cv::cv2eigen(T, eigMat);
  Eigen::Quaternionf q(eigMat.block<3, 3>(0, 0).cast<float>());
  return Sophus::SE3<float>(q.normalized(),
                            eigMat.block<3, 1>(0, 3).cast<float>());
}

Sophus::Sim3f Converter::toSophus(const g2o::Sim3& S) {
  return Sophus::Sim3f(
      Sophus::RxSO3d((float)S.scale(), S.rotation().matrix()).cast<float>(),
      S.translation().cast<float>());
}
//template <typename T>
Eigen::Quaternion<float> Converter::QuaternionAvg(
    std::vector<Eigen::Quaternion<float>>& vQuaternions) {
  if (vQuaternions.empty()) {
    std::cerr << "Try to avg an empty vector return Identity" << std::endl;
    return Eigen::Quaternion<float>::Identity();
  }
  // first build a 4x4 matrix which is the elementwise sum of the product of
  // each quaternion with itself
  Eigen::Matrix<float, 4, 4> A = Eigen::Matrix<float, 4, 4>::Zero();

  for (auto quaternion : vQuaternions) {
    A = A.eval() +
        quaternion.coeffs() * quaternion.coeffs().transpose();
  }

  // normalise with the number of quaternions
  A /= vQuaternions.size();

  // Compute the SVD of this 4x4 matrix
  Eigen::JacobiSVD<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> svd(
      A, Eigen::ComputeThinU | Eigen::ComputeThinV);

  Eigen::Matrix<float, Eigen::Dynamic, 1> singularValues = svd.singularValues();
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> U = svd.matrixU();

  // find the eigen vector corresponding to the largest eigen value
  int largestEigenValueIndex = 0;
  float largestEigenValue;
  bool first = true;

  for (auto i = 0; i < singularValues.rows(); ++i) {
    if (first) {
      largestEigenValue = singularValues(i);
      largestEigenValueIndex = i;
      first = false;
    } else if (singularValues(i) > largestEigenValue) {
      largestEigenValue = singularValues(i);
      largestEigenValueIndex = i;
    }
  }
  Eigen::Quaternion<float> QuaternionAvg(
      U(3, largestEigenValueIndex), U(0, largestEigenValueIndex),
      U(1, largestEigenValueIndex), U(2, largestEigenValueIndex));
  return QuaternionAvg;
}

}  // namespace ORB_SLAM3
