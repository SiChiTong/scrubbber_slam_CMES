//
// Created by gaoxiang on 2019/12/12.
//

#include "algorithm/constant.h"
#include <cfloat>

namespace HAMO {

template <>
const float Constant<float>::MAX_REAL = FLT_MAX;
template <>
const float Constant<float>::ZERO_TOLERANCE = 1e-06f;
template <>
const double Constant<double>::MAX_REAL = DBL_MAX;
template <>
const double Constant<double>::ZERO_TOLERANCE = 1e-08;

}  // namespace HAMO