//
// Created by gaoxiang on 2019/12/18.
//

#ifndef HAMO_LINUX_POINT_COLOR_H
#define HAMO_LINUX_POINT_COLOR_H

#include "algorithm/common.h"

namespace HAMO {

/// NOTE
/// 由于Eigen会自动将内部数据对齐到16bit，所以下面这个PointColor数据应该占32字节（8个float而非7个）
/// 因此，在访问color数据时，也不能直接在point数据偏移一个sizeof(V3f)，而得偏移sizeof(V4f)
/// 在windows下没有这个问题
struct PointColor {
   public:
    V3f point;
    V4f color;

   public:
    PointColor() {}

    PointColor(float x, float y, float z, const V4f &clr)
        : point(x, y, z), color(clr) {}

    PointColor(const V3f &vec, const V4f &clr) : point(vec), color(clr) {}
};

}  // namespace HAMO

#endif  // HAMO_LINUX_POINT_COLOR_H
