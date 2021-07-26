//
// Created by xiang on 2020/1/9.
//

#ifndef HAMO_LINUX_VECTOR4_H
#define HAMO_LINUX_VECTOR4_H
#include <Eigen/Core>

namespace HAMO {

template <typename N>
class Vector4 {
   public:
    typedef N ValueType;

   public:
    // 构造函数
    Vector4();

    Vector4(N xx, N yy, N zz, N ww);

    Vector4(const Vector4 &vec);

    void SetConstant(N val);

    Vector4 Min(const Vector4 &vec) const;

    Vector4 Max(const Vector4 &vec) const;

    N &operator[](size_t index) {
        if (index == 0) return x;
        if (index == 1) return y;
        if (index == 2) return z;
        if (index == 3) return w;
    }

    const N &operator[](size_t index) const {
        if (index == 0) return x;
        if (index == 1) return y;
        if (index == 2) return z;
        if (index == 3) return w;
    }

   public:
    // 成员函数
    inline Eigen::Matrix<N, 4, 1> ToEigen() const { return Eigen::Matrix<N, 4, 1>(x, y, z, w); }

   public:
    N x, y, z, w;
};

//------------------------------------------------------------------------------
typedef Vector4<float> Vector4f;
typedef Vector4<double> Vector4d;

//------------------------------------------------------------------------------

template <typename N>
Vector4<N>::Vector4() {
    x = y = z = w = ValueType(0);
}

template <typename N>
Vector4<N>::Vector4(N xx, N yy, N zz, N ww) {
    x = xx;
    y = yy;
    z = zz;
    w = ww;
}

template <typename N>
Vector4<N>::Vector4(const Vector4 &vec) {
    x = vec.x;
    y = vec.y;
    z = vec.z;
    w = vec.w;
}

template <typename N>
void Vector4<N>::SetConstant(N val) {
    x = y = z = w = val;
}

template <typename N>
Vector4<N> Vector4<N>::Max(const Vector4 &vec) const {
    return Vector4(x > vec.x ? x : vec.x, y > vec.y ? y : vec.y, z > vec.z ? z : vec.z, w > vec.w ? w : vec.w);
}

template <typename N>
Vector4<N> Vector4<N>::Min(const Vector4 &vec) const {
    return Vector4(x < vec.x ? x : vec.x, y < vec.y ? y : vec.y, z < vec.z ? z : vec.z, w < vec.w ? w : vec.w);
}
}  // namespace HAMO

#endif  // HAMO_LINUX_VECTOR4_H
