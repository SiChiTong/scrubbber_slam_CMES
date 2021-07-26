//
// Created by gaoxiang on 2019/12/13.
//

#ifndef HAMO_LINUX_MATRIX44_H
#define HAMO_LINUX_MATRIX44_H

#include "algorithm/common.h"
#include "algorithm/mc_math.h"
#include "algorithm/quaternion.h"
#include "algorithm/vector3.h"

#include <Eigen/Core>

namespace HAMO {

/// 这个4x4是OpenGL格式的，col-major的！
template <typename N>
class Matrix4x4 {
   public:
    typedef N ValueType;
    typedef Vector3<N> VectorType;
    typedef Quaternion<N> QuatType;

   public:
    Matrix4x4();

    Matrix4x4(const Matrix4x4<N> &other);

    Matrix4x4(N f11, N f12, N f13, N f14, N f21, N f22, N f23, N f24, N f31, N f32, N f33, N f34, N f41, N f42, N f43,
              N f44);

    // from eigen
    Matrix4x4(const Eigen::Matrix<N, 4, 4> &);

    Matrix4x4<N> Multiply(const Matrix4x4<N> &mat);

    Matrix4x4<N> Inverse() const;
    Matrix4x4<N> Transpose() const;

    VectorType TransformCoord(const VectorType &vec) const;

    VectorType TransformNormal(const VectorType &vec) const;

    QuatType GetRotate() const;

    Eigen::Matrix<N, 4, 4> ToEigen() const {
        Eigen::Matrix<N, 4, 4> m;
        m << m11, m21, m31, m41, m12, m22, m32, m42, m13, m23, m33, m43, m14, m24, m34, m44;
        return m;
    }

   public:
    static Matrix4x4<N> MakeTrans(N tx, N ty, N tz);

    static Matrix4x4<N> MakeTrans(const VectorType &vec);

    static Matrix4x4<N> MakeScale(N sx, N sy, N sz);

    static Matrix4x4<N> MakeRotationZ(N Angle);

    static Matrix4x4<N> MakeRotationY(N Angle);

    static Matrix4x4<N> MakeRotationX(N Angle);

    static Matrix4x4<N> MakeRotation(const VectorType &vec, N angle);

    static Matrix4x4<N> MakeRotation(const QuatType &quat);

    static Matrix4x4<N> LookAt(const VectorType &eye, const VectorType &center, const VectorType &up);

    static Matrix4x4<N> Ortho(N left, N right, N bottom, N top, N zNear, N zFar);

    static Matrix4x4<N> Perspective(N fovy, N aspect, N zNear, N zFar);

    static Matrix4x4<N> Frustum(N left, N right, N bottom, N top, N zNear, N zFar);

   public:
    union {
        struct {
            N m11, m12, m13, m14;
            N m21, m22, m23, m24;
            N m31, m32, m33, m34;
            N m41, m42, m43, m44;
        };
        N m[4][4];
        N mat[16];
    };
};

//------------------------------------------------------------------------------
typedef Matrix4x4<float> Matrix4x4f;
typedef Matrix4x4<double> Matrix4x4d;

//------------------------------------------------------------------------------

template <typename N>
Matrix4x4<N>::Matrix4x4() {
    m12 = m13 = m14 = m21 = m23 = m24 = m31 = m32 = m34 = m41 = m42 = m43 = ValueType(0);
    m11 = m22 = m33 = m44 = ValueType(1);
}

template <typename N>
Matrix4x4<N>::Matrix4x4(const Matrix4x4<N> &other) {
    for (int i = 0; i < 16; i++) {
        mat[i] = other.mat[i];
    }
}

template <typename N>
Matrix4x4<N>::Matrix4x4(N f11, N f12, N f13, N f14, N f21, N f22, N f23, N f24, N f31, N f32, N f33, N f34, N f41,
                        N f42, N f43, N f44) {
    m11 = f11;
    m12 = f12;
    m13 = f13;
    m14 = f14;
    m21 = f21;
    m22 = f22;
    m23 = f23;
    m24 = f24;
    m31 = f31;
    m32 = f32;
    m33 = f33;
    m34 = f34;
    m41 = f41;
    m42 = f42;
    m43 = f43;
    m44 = f44;
}

template <typename N>
Matrix4x4<N>::Matrix4x4(const Eigen::Matrix<N, 4, 4> &mat) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            m[i][j] = mat(j, i);
        }
    }
}

template <typename N>
Matrix4x4<N> Matrix4x4<N>::Multiply(const Matrix4x4<N> &mat) {
    Matrix4x4<N> mat1(*this);

    for (int j = 0; j < 4; j++) {
        for (int i = 0; i < 4; i++) {
            m[j][i] = mat1.m[j][0] * mat.m[0][i] + mat1.m[j][1] * mat.m[1][i] + mat1.m[j][2] * mat.m[2][i] +
                      mat1.m[j][3] * mat.m[3][i];
        }
    }

    return *this;
}

template <typename N>
Matrix4x4<N> Matrix4x4<N>::Transpose() const {
    Matrix4x4<N> ret;
    Matrix4x4<N> mat1(*this);

    for (int j = 0; j < 4; j++) {
        for (int i = 0; i < 4; i++) {
            ret.m[i][j] = m[j][i];
        }
    }

    return ret;
}

template <typename N>
Matrix4x4<N> Matrix4x4<N>::Inverse() const {
    N m00 = m[0][0], m01 = m[0][1], m02 = m[0][2], m03 = m[0][3];
    N m10 = m[1][0], m11 = m[1][1], m12 = m[1][2], m13 = m[1][3];
    N m20 = m[2][0], m21 = m[2][1], m22 = m[2][2], m23 = m[2][3];
    N m30 = m[3][0], m31 = m[3][1], m32 = m[3][2], m33 = m[3][3];

    N v0 = m20 * m31 - m21 * m30;
    N v1 = m20 * m32 - m22 * m30;
    N v2 = m20 * m33 - m23 * m30;
    N v3 = m21 * m32 - m22 * m31;
    N v4 = m21 * m33 - m23 * m31;
    N v5 = m22 * m33 - m23 * m32;

    N t00 = +(v5 * m11 - v4 * m12 + v3 * m13);
    N t10 = -(v5 * m10 - v2 * m12 + v1 * m13);
    N t20 = +(v4 * m10 - v2 * m11 + v0 * m13);
    N t30 = -(v3 * m10 - v1 * m11 + v0 * m12);

    N invDet = 1 / (t00 * m00 + t10 * m01 + t20 * m02 + t30 * m03);

    N d00 = t00 * invDet;
    N d10 = t10 * invDet;
    N d20 = t20 * invDet;
    N d30 = t30 * invDet;

    N d01 = -(v5 * m01 - v4 * m02 + v3 * m03) * invDet;
    N d11 = +(v5 * m00 - v2 * m02 + v1 * m03) * invDet;
    N d21 = -(v4 * m00 - v2 * m01 + v0 * m03) * invDet;
    N d31 = +(v3 * m00 - v1 * m01 + v0 * m02) * invDet;

    v0 = m10 * m31 - m11 * m30;
    v1 = m10 * m32 - m12 * m30;
    v2 = m10 * m33 - m13 * m30;
    v3 = m11 * m32 - m12 * m31;
    v4 = m11 * m33 - m13 * m31;
    v5 = m12 * m33 - m13 * m32;

    N d02 = +(v5 * m01 - v4 * m02 + v3 * m03) * invDet;
    N d12 = -(v5 * m00 - v2 * m02 + v1 * m03) * invDet;
    N d22 = +(v4 * m00 - v2 * m01 + v0 * m03) * invDet;
    N d32 = -(v3 * m00 - v1 * m01 + v0 * m02) * invDet;

    v0 = m21 * m10 - m20 * m11;
    v1 = m22 * m10 - m20 * m12;
    v2 = m23 * m10 - m20 * m13;
    v3 = m22 * m11 - m21 * m12;
    v4 = m23 * m11 - m21 * m13;
    v5 = m23 * m12 - m22 * m13;

    N d03 = -(v5 * m01 - v4 * m02 + v3 * m03) * invDet;
    N d13 = +(v5 * m00 - v2 * m02 + v1 * m03) * invDet;
    N d23 = -(v4 * m00 - v2 * m01 + v0 * m03) * invDet;
    N d33 = +(v3 * m00 - v1 * m01 + v0 * m02) * invDet;

    return Matrix4x4<N>(d00, d01, d02, d03, d10, d11, d12, d13, d20, d21, d22, d23, d30, d31, d32, d33);
}

template <typename N>
Vector3<N> Matrix4x4<N>::TransformCoord(const VectorType &vec) const {
    VectorType vOut;
    N det = ValueType(1.0) / (m14 * vec[0] + m24 * vec[1] + m34 * vec[2] + m44);
    vOut[0] = (vec[0] * m11 + vec[1] * m21 + vec[2] * m31 + m41) * det;
    vOut[1] = (vec[0] * m12 + vec[1] * m22 + vec[2] * m32 + m42) * det;
    vOut[2] = (vec[0] * m13 + vec[1] * m23 + vec[2] * m33 + m43) * det;
    return vOut;
}

template <typename N>
Vector3<N> Matrix4x4<N>::TransformNormal(const VectorType &vec) const {
    return VectorType(m11 * vec[0] + m21 * vec[1] + m31 * vec[2], m12 * vec[0] + m22 * vec[1] + m32 * vec[2],
                      m13 * vec[0] + m23 * vec[1] + m33 * vec[2]);
}

template <typename N>
Quaternion<N> Matrix4x4<N>::GetRotate() const {
    N tq[4];
    tq[0] = 1 + m11 + m22 + m33;
    tq[1] = 1 + m11 - m22 - m33;
    tq[2] = 1 - m11 + m22 - m33;
    tq[3] = 1 - m11 - m22 + m33;

    int j = 0;
    for (int i = 1; i < 4; i++) {
        j = (tq[i] > tq[j]) ? i : j;
    }

    QuatType quat;
    if (j == 0) {
        quat.w = tq[0];
        quat.x = m23 - m32;
        quat.y = m31 - m13;
        quat.z = m12 - m21;
    } else if (j == 1) {
        quat.w = m23 - m32;
        quat.x = tq[1];
        quat.y = m12 + m21;
        quat.z = m31 + m13;
    } else if (j == 2) {
        quat.w = m31 - m13;
        quat.x = m12 + m21;
        quat.y = tq[2];
        quat.z = m23 + m32;
    } else {
        /* if (j==3) */
        quat.w = m12 - m21;
        quat.x = m31 + m13;
        quat.y = m23 + m32;
        quat.z = tq[3];
    }

    N s = Math<N>::Sqrt(N(0.25) / tq[j]);
    quat.w *= s;
    quat.x *= s;
    quat.y *= s;
    quat.z *= s;

    return quat;
}

//===================================================
template <typename N>
Matrix4x4<N> Matrix4x4<N>::MakeTrans(N tx, N ty, N tz) {
    Matrix4x4<N> trans;
    trans.m41 = tx;
    trans.m42 = ty;
    trans.m43 = tz;
    return trans;
}

template <typename N>
Matrix4x4<N> Matrix4x4<N>::MakeTrans(const VectorType &vec) {
    return MakeTrans(vec[0], vec[1], vec[2]);
}

template <typename N>
Matrix4x4<N> Matrix4x4<N>::MakeScale(N sx, N sy, N sz) {
    Matrix4x4<N> trans;
    trans.m11 = sx;
    trans.m22 = sy;
    trans.m33 = sz;
    return trans;
}

template <typename N>
Matrix4x4<N> Matrix4x4<N>::MakeRotationZ(N Angle) {
    N sinTheta = Math<N>::Sin(Angle);
    N cosTheta = Math<N>::Cos(Angle);

    Matrix4x4<N> rotateZ;
    rotateZ.m11 = cosTheta;
    rotateZ.m12 = sinTheta;
    rotateZ.m21 = -sinTheta;
    rotateZ.m22 = cosTheta;

    return rotateZ;
}

template <typename N>
Matrix4x4<N> Matrix4x4<N>::MakeRotationY(N Angle) {
    N sinTheta = Math<N>::Sin(Angle);
    N cosTheta = Math<N>::Cos(Angle);

    Matrix4x4f rotateY;
    rotateY.m11 = cosTheta;
    rotateY.m13 = -sinTheta;
    rotateY.m31 = sinTheta;
    rotateY.m33 = cosTheta;

    return rotateY;
}

template <typename N>
Matrix4x4<N> Matrix4x4<N>::MakeRotationX(N Angle) {
    N sinTheta = Math<N>::Sin(Angle);
    N cosTheta = Math<N>::Cos(Angle);

    Matrix4x4<N> rotateX;
    rotateX.m22 = cosTheta;
    rotateX.m23 = sinTheta;
    rotateX.m32 = -sinTheta;
    rotateX.m33 = cosTheta;

    return rotateX;
}

template <typename N>
Matrix4x4<N> Matrix4x4<N>::MakeRotation(const VectorType &vec, N angle) {
    N cosA = Math<N>::Cos(angle);
    N sinA = Math<N>::Sin(angle);

    VectorType axis(vec);
    axis.normalize();

    Matrix4x4<N> mat;
    mat.m11 = axis.x * axis.x * (1 - cosA) + cosA;
    mat.m12 = axis.x * axis.y * (1 - cosA) + axis.z * sinA;
    mat.m13 = axis.x * axis.z * (1 - cosA) - axis.y * sinA;

    mat.m21 = axis.x * axis.y * (1 - cosA) - axis.z * sinA;
    mat.m22 = axis.y * axis.y * (1 - cosA) + cosA;
    mat.m23 = axis.y * axis.z * (1 - cosA) + axis.x * sinA;

    mat.m31 = axis.x * axis.z * (1 - cosA) + axis.y * sinA;
    mat.m32 = axis.y * axis.z * (1 - cosA) - axis.x * sinA;
    mat.m33 = axis.z * axis.z * (1 - cosA) + cosA;

    return mat;
}

template <typename N>
Matrix4x4<N> Matrix4x4<N>::MakeRotation(const QuatType &quat) {
    Matrix4x4<N> mat;

    N length2 = quat.squaredNorm();

    if (Math<N>::Abs(length2) <= 2.2250738585072014e-308) {
        mat.m11 = 0.0;
        mat.m21 = 0.0;
        mat.m31 = 0.0;
        mat.m12 = 0.0;
        mat.m22 = 0.0;
        mat.m32 = 0.0;
        mat.m13 = 0.0;
        mat.m23 = 0.0;
        mat.m33 = 0.0;
    } else {
        N rlength2 = 2.0;

        if (length2 != N(1.0)) {
            rlength2 = rlength2 / length2;
        }

        N wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;

        x2 = rlength2 * quat.x;
        y2 = rlength2 * quat.y;
        z2 = rlength2 * quat.z;

        xx = quat.x * x2;
        xy = quat.x * y2;
        xz = quat.x * z2;

        yy = quat.y * y2;
        yz = quat.y * z2;
        zz = quat.z * z2;

        wx = quat.w * x2;
        wy = quat.w * y2;
        wz = quat.w * z2;

        mat.m11 = N(1.0) - (yy + zz);
        mat.m21 = xy - wz;
        mat.m31 = xz + wy;

        mat.m12 = xy + wz;
        mat.m22 = N(1.0) - (xx + zz);
        mat.m32 = yz - wx;

        mat.m13 = xz - wy;
        mat.m23 = yz + wx;
        mat.m33 = N(1.0) - (xx + yy);
    }

    return mat;
}

template <typename N>
Matrix4x4<N> Matrix4x4<N>::Ortho(N left, N right, N bottom, N top, N zNear, N zFar) {
    Matrix4x4<N> prjMat;

    prjMat.m11 = 2 / (right - left);
    prjMat.m22 = 2 / (top - bottom);
    prjMat.m33 = -2 / (zFar - zNear);
    prjMat.m41 = -(right + left) / (right - left);
    prjMat.m42 = -(top + bottom) / (top - bottom);
    prjMat.m43 = -(zFar + zNear) / (zFar - zNear);

    return prjMat;
}

template <typename N>
Matrix4x4<N> Matrix4x4<N>::Perspective(N fovy, N aspect, N zNear, N zFar) {
    N radians = fovy / 2;

    N sine = Math<N>::Sin(radians);
    N cotangent = Math<N>::Cos(radians) / sine;

    Matrix4x4<N> prjMarix;
    prjMarix.m11 = cotangent / aspect;
    prjMarix.m22 = cotangent;
    prjMarix.m33 = -(zFar + zNear) / (zFar - zNear);
    prjMarix.m34 = -1;
    prjMarix.m43 = -2 * zNear * zFar / (zFar - zNear);
    prjMarix.m44 = 0;

    return prjMarix;
}

template <typename N>
Matrix4x4<N> Matrix4x4<N>::Frustum(N left, N right, N bottom, N top, N zNear, N zFar) {
    Matrix4x4<N> prjMarix;

    prjMarix.m11 = 2 * zNear / (right - left);
    prjMarix.m22 = 2 * zNear / (top - bottom);
    prjMarix.m33 = -(zFar + zNear) / (zFar - zNear);
    prjMarix.m31 = (right + left) / (right - left);
    prjMarix.m32 = (top + bottom) / (top - bottom);
    prjMarix.m34 = -1;
    prjMarix.m43 = -2 * zFar * zNear / (zFar - zNear);
    prjMarix.m44 = 0;

    return prjMarix;
}

template <typename N>
Matrix4x4<N> Matrix4x4<N>::LookAt(const VectorType &eye, const VectorType &center, const VectorType &up) {
    VectorType fvec(center - eye);
    fvec.normalize();

    VectorType svec = fvec.cross(up);
    svec.normalize();

    VectorType uvec = svec.cross(fvec);

    Matrix4x4<N> viewMat;
    viewMat.m11 = svec[0];
    viewMat.m21 = svec[1];
    viewMat.m31 = svec[2];
    viewMat.m12 = uvec[0];
    viewMat.m22 = uvec[1];
    viewMat.m32 = uvec[2];
    viewMat.m13 = -fvec[0];
    viewMat.m23 = -fvec[1];
    viewMat.m33 = -fvec[2];
    viewMat.m41 = -svec.dot(eye);
    viewMat.m42 = -uvec.dot(eye);
    viewMat.m43 = fvec.dot(eye);

    return viewMat;
}

//=============================================================

template <typename N>
Matrix4x4<N> operator*(const Matrix4x4<N> &mat1, const Matrix4x4<N> &mat2) {
    Matrix4x4<N> mat;

    for (int j = 0; j < 4; j++) {
        for (int i = 0; i < 4; i++) {
            mat.m[j][i] = mat1.m[j][0] * mat2.m[0][i] + mat1.m[j][1] * mat2.m[1][i] + mat1.m[j][2] * mat2.m[2][i] +
                          mat1.m[j][3] * mat2.m[3][i];
        }
    }

    return mat;
}

}  // namespace HAMO

#endif  // HAMO_LINUX_MATRIX44_H
