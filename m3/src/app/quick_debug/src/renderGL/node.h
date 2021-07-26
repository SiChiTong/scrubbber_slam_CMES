//
// Created by gaoxiang on 2019/12/19.
//

#ifndef HAMO_LINUX_NODE_H
#define HAMO_LINUX_NODE_H

#include "renderGL/object.h"

namespace HAMO {

class Node : public Object {
   public:
    Node() = default;
    ~Node() override {}
};

}  // namespace HAMO

#endif  // HAMO_LINUX_NODE_H
