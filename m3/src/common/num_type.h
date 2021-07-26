//
// Created by gaoxiang on 2020/8/10.
//

#ifndef MAPPING_NUM_TYPE_H
#define MAPPING_NUM_TYPE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Sophus/se2.hpp"
#include "Sophus/se3.hpp"

/// 数值类型缩写
using IdType = unsigned long;

// pose represented as sophus structs
using SE2 = Sophus::SE2d;
using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;

/// numerical types
/// alias for eigen struct
using V2f = Eigen::Vector2f;
using V3f = Eigen::Vector3f;
using V4f = Eigen::Matrix<float, 4, 1>;
using V6f = Eigen::Matrix<float, 6, 1>;
using M2f = Eigen::Matrix<float, 2, 2>;
using M3f = Eigen::Matrix<float, 3, 3>;
using M4f = Eigen::Matrix<float, 4, 4>;
using M6f = Eigen::Matrix<float, 6, 6>;
using Aff3f = Eigen::Affine3f;
using Quatf = Eigen::Quaternion<float>;
using Trans3f = Eigen::Translation3f;
using AngAxisf = Eigen::AngleAxis<float>;

using V2d = Eigen::Vector2d;
using V3d = Eigen::Vector3d;
using V4d = Eigen::Vector4d;
using V6d = Eigen::Matrix<double, 6, 1>;
using V7d = Eigen::Matrix<double, 7, 1>;
using VXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using M1d = Eigen::Matrix<double, 1, 1>;
using M2d = Eigen::Matrix<double, 2, 2>;
using M3d = Eigen::Matrix<double, 3, 3>;
using M4d = Eigen::Matrix<double, 4, 4>;
using M6d = Eigen::Matrix<double, 6, 6>;
using H6d = Eigen::Matrix<double, 1, 6>;
using H1d = Eigen::Matrix<double, 1, 1>;
using Aff3d = Eigen::Affine3d;
using Quatd = Eigen::Quaternion<double>;
using Trans3d = Eigen::Translation3d;
using AngAxisd = Eigen::AngleAxis<double>;

using Quat = Quatd;

using Vec3f = Eigen::Vector3f;
using Mat36f = Eigen::Matrix<float, 3, 6>;
using Mat66f = Eigen::Matrix<float, 6, 6>;
using Vec6f = Eigen::Matrix<float, 6, 1>;
using SE3f =  Sophus::SE3f;





#endif  // MAPPING_NUM_TYPE_H
