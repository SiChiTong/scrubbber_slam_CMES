//
// Created by gaoxiang on 2019/12/19.
//

#ifndef HAMO_LINUX_POSITION_TRANSFORM_NODE_H
#define HAMO_LINUX_POSITION_TRANSFORM_NODE_H

#include "renderGL/group_node.h"

namespace HAMO {

class PositionTransformNode : public GroupNode {
   public:
    PositionTransformNode() = default;

    ~PositionTransformNode() override = default;

   public:
    void SetPosition(const V3d &pos) { position_ = pos; }

    const V3d &GetPosition() const { return position_; }

   private:
    V3d position_;
};

}  // namespace HAMO
#endif  // HAMO_LINUX_POSITION_TRANSFORM_NODE_H
