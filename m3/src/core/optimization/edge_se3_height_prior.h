//
// Created by gaoxiang on 2020/9/16.
//

#ifndef MAPPING_EDGE_SE3_HEIGHT_PRIOR_H
#define MAPPING_EDGE_SE3_HEIGHT_PRIOR_H

#include "common/num_type.h"

#include <g2o/types/slam3d/types_slam3d.h>

namespace mapping::core {

/// 高度约束
class EdgeSE3HeightPrior : public g2o::BaseUnaryEdge<1, double, g2o::VertexSE3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3HeightPrior() = default;

    bool read(std::istream &is) override { return true; }

    bool write(std::ostream &os) const override { return true; }

    void computeError() override {
        auto v0 = dynamic_cast<g2o::VertexSE3 *>(_vertices[0]);
        _error[0] = v0->estimate().translation()[2] - _measurement;
    }

    void linearizeOplus() override {
        auto v0 = dynamic_cast<g2o::VertexSE3 *>(_vertices[0]);
        _jacobianOplusXi.setZero();
        V3d j = v0->estimate().rotation().matrix().block<1, 3>(2, 0).transpose();
        _jacobianOplusXi.head<3>() = j;
    }

   protected:
};

}  // namespace mapping::core

#endif  // MAPPING_EDGE_SE3_HEIGHT_PRIOR_H
