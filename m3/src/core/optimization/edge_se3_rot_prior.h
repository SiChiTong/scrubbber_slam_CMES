//
// Created by gaoxiang on 2020/9/16.
//

#ifndef MAPPING_EDGE_SE3_ROT_PRIOR_H
#define MAPPING_EDGE_SE3_ROT_PRIOR_H

#include "common/num_type.h"

#include <g2o/types/slam3d/types_slam3d.h>

namespace mapping::core {

/// 约束车辆Z轴与世界系Z轴对齐
class EdgeSE3RotPrior : public g2o::BaseUnaryEdge<2, V2d, g2o::VertexSE3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3RotPrior() {}

    virtual bool read(std::istream &is) { return true; }

    virtual bool write(std::ostream &os) const { return true; }

    // return the error estimate as a 3-vector
    void computeError() {
        auto v0 = static_cast<g2o::VertexSE3 *>(_vertices[0]);
        _error[0] = v0->estimate()(2, 0);
        _error[1] = v0->estimate()(2, 1);

        if (fabs(_error[0]) < dead_zone_) {
            _error(0, 0) = 0;
        }

        if (fabs(_error[1]) < dead_zone_) {
            _error(1, 0) = 0;
        }
    }

   protected:
    const double dead_zone_ = 5 * M_PI / 180;  // 死区以内不记误差
};
}  // namespace mapping::core
#endif  // MAPPING_EDGE_SE3_ROT_PRIOR_H
