//
// Created by xiang on 2020/1/9.
//

#ifndef HAMO_LINUX_VECTOR3_H
#define HAMO_LINUX_VECTOR3_H

#include <Eigen/Core>
#include "algorithm/mc_math.h"

namespace HAMO {

template <typename N>
class Vector3 {
   public:
    typedef N ValueType;

   public:
    // 构造函数
    Vector3();

    Vector3(N xx, N yy, N zz);

    Vector3(const Vector3 &vec);

    Vector3(const Eigen::Matrix<N, 3, 1> &evec);

    inline Eigen::Matrix<N, 3, 1> ToEigen() const { return Eigen::Matrix<N, 3, 1>(x, y, z); }

   public:
    // 成员函数
    void Set(N xx, N yy, N zz);

    void Multiply(N scalar);

    N Length() const;

    N norm() const { return Length(); }

    N LengthSquare() const;

    N squaredNorm() const { return LengthSquare(); }

    N Normalize();

    N normalize() { return Normalize(); }

    Vector3 CrossProduct(const Vector3 &vec) const;

    Vector3 cross(const Vector3 &vec) const { return CrossProduct(vec); }

    Vector3 MiddlePoint(const Vector3 &vec) const;

    N DotProduct(const Vector3<N> &vec) const;

    N dot(const Vector3<N> &vec) const { return DotProduct(vec); }

    void clear();

    Vector3<N> operator-() const;

    Vector3<N> &operator-=(const Vector3 &v);

    Vector3<N> &operator+=(const Vector3 &v);

    Vector3<N> &operator*=(N scalar);

    N &operator[](size_t index) {
        if (index == 0) return x;
        if (index == 1) return y;
        if (index == 2) return z;
    }

    const N &operator[](size_t index) const {
        if (index == 0) return x;
        if (index == 1) return y;
        if (index == 2) return z;
    }

   public:
    N x, y, z;
};

//------------------------------------------------------------------------------
typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;

//------------------------------------------------------------------------------

template <typename N>
Vector3<N>::Vector3() {
    x = y = z = ValueType(0);
}

template <typename N>
Vector3<N>::Vector3(N xx, N yy, N zz) {
    x = xx;
    y = yy;
    z = zz;
}

template <typename N>
Vector3<N>::Vector3(const Vector3 &vec) {
    x = vec.x;
    y = vec.y;
    z = vec.z;
}

template <typename N>
Vector3<N>::Vector3(const Eigen::Matrix<N, 3, 1> &evec) {
    x = evec[0];
    y = evec[1];
    z = evec[2];
}

template <typename N>
void Vector3<N>::Set(N xx, N yy, N zz) {
    x = xx;
    y = yy;
    z = zz;
}

template <typename N>

void Vector3<N>::Multiply(N scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
}

template <typename N>
N Vector3<N>::Length() const {
    return Math<N>::Sqrt(x * x + y * y + z * z);
}

template <typename N>
N Vector3<N>::Normalize() {
    N fLength = Math<N>::Sqrt(x * x + y * y + z * z);
    if (fLength > ValueType(0)) {
        N fInvLength = ValueType(1) / fLength;
        x *= fInvLength;
        y *= fInvLength;
        z *= fInvLength;
    }

    return fLength;
}

template <typename N>
N Vector3<N>::LengthSquare() const {
    return x * x + y * y + z * z;
}

template <typename N>
Vector3<N> Vector3<N>::CrossProduct(const Vector3 &vec) const {
    return Vector3(y * vec.z - z * vec.y, z * vec.x - x * vec.z, x * vec.y - y * vec.x);
}

template <typename N>
N Vector3<N>::DotProduct(const Vector3<N> &vec) const {
    return x * vec.x + y * vec.y + z * vec.z;
}

template <typename N>
Vector3<N> Vector3<N>::MiddlePoint(const Vector3 &point) const {
    N scaler = ValueType(0.5);

    return Vector3((x + x + point.x) * scaler, (y + y + point.y) * scaler, (z + z + point.z) * scaler);
}

template <typename N>
Vector3<N> Vector3<N>::operator-() const {
    return Vector3(-x, -y, -z);
}

template <typename N>
Vector3<N> &Vector3<N>::operator+=(const Vector3 &vec) {
    x += vec.x;
    y += vec.y;
    z += vec.z;

    return (*this);
}

template <typename N>
inline Vector3<N> &Vector3<N>::operator*=(N scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

template <typename N>
Vector3<N> &Vector3<N>::operator-=(const Vector3 &vec) {
    x -= vec.x;
    y -= vec.y;
    z -= vec.z;

    return (*this);
}

//---------------------------------

template <typename N>
Vector3<N> operator+(const Vector3<N> &vec1, const Vector3<N> &vec2) {
    return Vector3<N>(vec1.x + vec2.x, vec1.y + vec2.y, vec1.z + vec2.z);
}

template <typename N>
Vector3<N> operator-(const Vector3<N> &vec1, const Vector3<N> &vec2) {
    return Vector3<N>(vec1.x - vec2.x, vec1.y - vec2.y, vec1.z - vec2.z);
}

template <typename N>
Vector3<N> operator*(const Vector3<N> &vec, N length) {
    return Vector3<N>(vec.x * length, vec.y * length, vec.z * length);
}

template <typename N>
Vector3<N> CrossProduct(const Vector3<N> &vec1, const Vector3<N> &vec2) {
    return Vector3<N>(vec1.y * vec2.z - vec1.z * vec2.y, vec1.z * vec2.x - vec1.x * vec2.z,
                      vec1.x * vec2.y - vec1.y * vec2.x);
}

template <typename N>
N DotProduct(const Vector3<N> &vec1, const Vector3<N> &vec2) {
    return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
}

template <typename N>
Vector3<N> Normalize(const Vector3<N> &vec) {
    N fLength = Math<N>::Sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    if (fLength > N(0)) {
        N fInvLength = N(1) / fLength;

        return Vector3<N>(vec.x * fInvLength, vec.y * fInvLength, vec.z * fInvLength);
    } else {
        return vec;
    }
}

template <typename N>
void Vector3<N>::clear() {
    x = 0;
    y = 0;
    z = 0;
}

}  // namespace HAMO

#endif  // HAMO_LINUX_VECTOR3_H
