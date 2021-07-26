//
// Created by gaoxiang on 2020/10/21.
//

#ifndef MAPPING_CALIB_EDGE_H
#define MAPPING_CALIB_EDGE_H

#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include "common/num_type.h"

namespace mapping::core {

/**
 * measurement: relative motion estimated by lidar
 * member: relative motion estimated by gnss
 */
class CalibEdge : public g2o::BaseUnaryEdge<6, SE3, g2o::VertexSE3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    CalibEdge(const SE3 &rel_pose_gnss) : rel_pose_gnss_(rel_pose_gnss) {}

    virtual void computeError() override {
        g2o::VertexSE3 *v = static_cast<g2o::VertexSE3 *>(_vertices[0]);
        SE3 TGL(v->estimate().matrix());
        _error = (_measurement.inverse() * (TGL.inverse() * rel_pose_gnss_ * TGL)).log();
    }

    virtual bool read(std::istream &in) override { return true; }

    virtual bool write(std::ostream &out) const override { return true; }

   private:
    SE3 rel_pose_gnss_;
};

}  // namespace mapping::core
#endif  // MAPPING_CALIB_EDGE_H
