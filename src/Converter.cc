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

cv::Mat Converter::toCvMat(const Eigen::Matrix<float, 3, 4>& m) {
  cv::Mat cvMat(3, 4, CV_32F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 4; j++) cvMat.at<float>(i, j) = m(i, j);

  return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3f& m) {
  cv::Mat cvMat(3, 3, CV_32F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) cvMat.at<float>(i, j) = m(i, j);

  return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<float, 3, 1>& m) {
  cv::Mat cvMat(3, 1, CV_32F);
  for (int i = 0; i < 3; i++) cvMat.at<float>(i) = m(i);

  return cvMat.clone();
}

Eigen::Matrix<float, 3, 1> Converter::toVector3f(const cv::Mat& cvVector) {
  Eigen::Matrix<float, 3, 1> v;
  v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

  return v;
}

Eigen::Matrix<float, 3, 3> Converter::toMatrix3f(const cv::Mat& cvMat3) {
  Eigen::Matrix<float, 3, 3> M;

  M << cvMat3.at<float>(0, 0), cvMat3.at<float>(0, 1), cvMat3.at<float>(0, 2),
      cvMat3.at<float>(1, 0), cvMat3.at<float>(1, 1), cvMat3.at<float>(1, 2),
      cvMat3.at<float>(2, 0), cvMat3.at<float>(2, 1), cvMat3.at<float>(2, 2);

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

}  // namespace ORB_SLAM3
