//
// Created by xiang on 2020/1/9.
//

#ifndef HAMO_LINUX_VECTOR2_H
#define HAMO_LINUX_VECTOR2_H

#include <Eigen/Core>
#include "algorithm/mc_math.h"

namespace HAMO {

template <typename N>
class Vector2 {
   public:
    typedef N ValueType;

   public:
    // 构造函数
    Vector2();

    Vector2(N xx, N yy);

    Vector2(const Vector2 &vec);

    inline Eigen::Matrix<N, 2, 1> ToEigen() const { return Eigen::Matrix<N, 2, 1>(x, y); }

   public:
    // 成员函数
    void Set(N xx, N yy);

    N Length() const;

    N LengthSquare() const;

    N Normalize();

    N &operator[](size_t index) {
        if (index == 0) return x;
        if (index == 1) return y;
    }

    const N &operator[](size_t index) const {
        if (index == 0) return x;
        if (index == 1) return y;
    }

   public:
    N x, y;
};

//------------------------------------------------------------------------------
typedef Vector2<float> Vector2f;
typedef Vector2<double> Vector2d;

//------------------------------------------------------------------------------

template <typename N>
Vector2<N>::Vector2() {
    x = y = ValueType(0);
}

template <typename N>
Vector2<N>::Vector2(N xx, N yy) {
    x = xx;
    y = yy;
}

template <typename N>
Vector2<N>::Vector2(const Vector2 &vec) {
    x = vec.x;
    y = vec.y;
}

template <typename N>
void Vector2<N>::Set(N xx, N yy) {
    x = xx;
    y = yy;
}

template <typename N>
N Vector2<N>::Length() const {
    return Math<N>::Sqrt(x * x + y * y);
}

template <typename N>
N Vector2<N>::Normalize() {
    N fLength = Math<N>::Sqrt(x * x + y * y);
    if (fLength > ValueType(0)) {
        N fInvLength = ValueType(1) / fLength;
        x *= fInvLength;
        y *= fInvLength;
    }

    return fLength;
}

template <typename N>
N Vector2<N>::LengthSquare() const {
    return x * x + y * y;
}

}  // namespace HAMO

#endif  // HAMO_LINUX_VECTOR2_H
