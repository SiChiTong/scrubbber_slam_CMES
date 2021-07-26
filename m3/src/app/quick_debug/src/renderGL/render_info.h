//
// Created by gaoxiang on 2019/12/18.
//

#ifndef HAMO_LINUX_RENDER_INFO_H
#define HAMO_LINUX_RENDER_INFO_H

#include <vector>
#include "renderGL/state.h"

namespace HAMO {

class RenderInfo {
   public:
    RenderInfo() {}

    void SetState(State *state) { state_ = state; }

    State *GetState() { return state_; }

   private:
    State *state_ = nullptr;

   public:
    Program *program_ = nullptr;
};

}  // namespace HAMO
#endif  // HAMO_LINUX_RENDER_INFO_H
