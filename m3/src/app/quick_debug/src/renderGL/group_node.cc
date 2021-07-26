//
// Created by gaoxiang on 2019/12/19.
//

#include "renderGL/group_node.h"

namespace HAMO {

GroupNode::GroupNode() {}

GroupNode::~GroupNode() {
    for (auto &i : children_) {
        delete i;
    }
    children_.clear();
}

void GroupNode::AddChild(Object *drawable) { children_.push_back(drawable); }

int GroupNode::GetNumChildren() const {
    return static_cast<int>(children_.size());
}

Object *GroupNode::GetChild(int index) { return children_[index]; }

void GroupNode::RemoveAllChild() {
    for (auto &i : children_) {
        if (i != nullptr) {
            delete i;
        }
    }

    children_.clear();
}

}  // namespace HAMO
