//
// Created by gaoxiang on 2019/12/13.
//

#ifndef HAMO_LINUX_MC_MATH_H
#define HAMO_LINUX_MC_MATH_H

#include <stdlib.h>
#include <cmath>

namespace HAMO {

template <typename N>
class Math {
   public:
    static N Sqrt(N fValue);

    static N Sin(N fValue);

    static N Cos(N fValue);

    static N Tan(N fValue);

    static N Asin(N fValue);

    static N Acos(N fValue);

    static N Atan(N fValue);

    static N Atan2(N fY, N fX);

    static N Ceil(N fValue);

    static N Floor(N fValue);

    static N Round(N fValue);

    static N Pow(N fx, N fy);

    static N Exp(N fValue);

    static N Abs(N fValue);

    static N Max(N fVal1, N fVal2);

    static N Min(N fVal1, N fVal2);

    static N InvSqrt(N fValue);

    static N ToRadians(N fAngle);

    static N ToDegrees(N fAngle);

    static N Clamp(N fVal, N fmin, N fmax);

    static N Hypot(N fx, N fy);

    static N Fmod(N fx, N fy);
};

//------------------------------------------------------------------------------
typedef Math<float> Mathf;
typedef Math<double> Mathd;

//------------------------------------------------------------------------------
template <typename N>
N Math<N>::Sqrt(N fValue) {
    double dValue = sqrt((double)fValue);
    return N(dValue);
}

template <typename N>
N Math<N>::Sin(N fValue) {
    double dValue = sin((double)fValue);
    return N(dValue);
}

template <typename N>
N Math<N>::Cos(N fValue) {
    double dValue = cos((double)fValue);
    return N(dValue);
}

template <typename N>
N Math<N>::Tan(N fValue) {
    double dValue = tan((double)fValue);
    return N(dValue);
}

template <typename N>
N Math<N>::Asin(N fValue) {
    double dValue = asin((double)fValue);
    return N(dValue);
}

template <typename N>
N Math<N>::Acos(N fValue) {
    double dValue = acos((double)fValue);
    return N(dValue);
}

template <typename N>
N Math<N>::Atan(N fValue) {
    double dValue = atan((double)fValue);
    return N(dValue);
}

template <typename N>
N Math<N>::Atan2(N fY, N fX) {
    double dValue = atan2((double)fY, (double)fX);
    return N(dValue);
}

template <typename N>
N Math<N>::Ceil(N fValue) {
    double dValue = ceil((double)fValue);
    return N(dValue);
}

template <typename N>
N Math<N>::Floor(N fValue) {
    double dValue = floor((double)fValue);
    return N(dValue);
}

template <typename N>
N Math<N>::Round(N fValue) {
    double dValue = round((double)fValue);
    return N(dValue);
}

template <typename N>
N Math<N>::Pow(N fx, N fy) {
    double dValue = pow((double)fx, (double)fy);
    return N(dValue);
}

template <typename N>
N Math<N>::Exp(N fValue) {
    double dValue = exp((double)fValue);
    return N(dValue);
}

template <typename N>
N Math<N>::Abs(N fValue) {
    return fValue > N(0) ? fValue : -fValue;
}

template <typename N>
N Math<N>::Max(N fVal1, N fVal2) {
    return (fVal1 > fVal2 ? fVal1 : fVal2);
}

template <typename N>
N Math<N>::Min(N fVal1, N fVal2) {
    return (fVal1 < fVal2 ? fVal1 : fVal2);
}

template <typename N>
N Math<N>::InvSqrt(N fValue) {
    double dValue = sqrt((double)fValue);
    if (dValue != (N)0) {
        double inv = 1.0 / dValue;
        return (N)inv;
    } else {
        return (N)0;
    }
}

template <typename N>
N Math<N>::Clamp(N fVal, N fMin, N fMax) {
    if (fVal > fMax) return fMax;
    if (fVal < fMin) return fMin;
    return fVal;
}

template <typename N>
N Math<N>::Hypot(N fx, N fy) {
    double dValue = hypot((double)fx, (double)fy);
    return (N)dValue;
}

template <typename N>
N Math<N>::Fmod(N fx, N fy) {
    double dValue = fmod((double)fx, (double)fy);
    return (N)dValue;
}

template <typename N>
N Math<N>::ToRadians(N fAngle) {
    return fAngle * ((N)0.017453292519943295769);
}

template <typename N>
N Math<N>::ToDegrees(N fAngle) {
    return fAngle * ((N)57.295779513082320876798);
}

}  // namespace HAMO

#endif  // HAMO_LINUX_MC_MATH_H
