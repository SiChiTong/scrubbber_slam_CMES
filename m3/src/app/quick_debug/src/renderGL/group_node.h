//
// Created by gaoxiang on 2019/12/19.
//

#ifndef HAMO_LINUX_GROUP_NODE_H
#define HAMO_LINUX_GROUP_NODE_H

#include <vector>
#include "renderGL/node.h"

namespace HAMO {

class Object;
class Drawable;

class GroupNode : public Node {
   public:
    GroupNode();

    virtual ~GroupNode();

   public:
    void AddChild(Object *drawable);

    int GetNumChildren() const;

    Object *GetChild(int index);

    void RemoveAllChild();

   private:
    std::vector<Object *> children_;
};

}  // namespace HAMO

#endif  // HAMO_LINUX_GROUP_NODE_H
