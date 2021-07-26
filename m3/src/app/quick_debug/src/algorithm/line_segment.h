//
// Created by gaoxiang on 2019/12/13.
//

#ifndef HAMO_LINUX_LINE_SEGMENT_H
#define HAMO_LINUX_LINE_SEGMENT_H

#include "algorithm/common.h"

namespace HAMO {

template <typename N>
class LineSegment3 {
   public:
    typedef Eigen::Matrix<N, 3, 1> VectorType;

    LineSegment3();

    LineSegment3(const LineSegment3 &seg);

    LineSegment3(const VectorType &s, const VectorType &e);

   public:
    VectorType Start() const;

    VectorType End() const;

    N GetLength() const;

   private:
    VectorType start_;
    VectorType end_;
};

//------------------------------------------------------------------------------
typedef LineSegment3<float> LineSegment3f;
typedef LineSegment3<double> LineSegment3d;

//------------------------------------------------------------------------------
template <typename N>
LineSegment3<N>::LineSegment3() {}

template <typename N>
LineSegment3<N>::LineSegment3(const LineSegment3 &seg)
    : start_(seg.start_), end_(seg.end_) {}

template <typename N>
LineSegment3<N>::LineSegment3(const VectorType &s, const VectorType &e)
    : start_(s), end_(e) {}

template <typename N>
Eigen::Matrix<N, 3, 1> LineSegment3<N>::Start() const {
    return start_;
}

template <typename N>
Eigen::Matrix<N, 3, 1> LineSegment3<N>::End() const {
    return end_;
}

template <typename N>
N LineSegment3<N>::GetLength() const {
    VectorType vec = end_ - start_;
    N distance = vec.Length();

    return distance;
}

}  // namespace HAMO

#endif  // HAMO_LINUX_LINE_SEGMENT_H
