//
// Created by vuong on 2/10/22.
//

#ifndef ORB_SLAM3_ODOMTYPES_H
#define ORB_SLAM3_ODOMTYPES_H

#include <Eigen/Core>
namespace ORB_SLAM3{
namespace ODOM {
class Meas {
 public:
  Meas(const double& x, const double& y, const double& yaw,
       const double& timestamp)
      : meas(x, y, yaw), t(timestamp){};
 public:
  Eigen::Vector3f meas;
  double t;
};
}  // namespace ODOM
}   // namespace ORB_SLAM3
#endif  // ORB_SLAM3_ODOMTYPES_H
