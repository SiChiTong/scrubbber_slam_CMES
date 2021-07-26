//
// Created by gaoxiang on 2019/12/16.
//

#ifndef HAMO_LINUX_STATE_H
#define HAMO_LINUX_STATE_H

#include "algorithm/matrix44.h"
#include "algorithm/viewport.h"
#include "renderGL/mc_program.h"

namespace HAMO {

class State {
   public:
    State();

    void initializeProcs();

    void ApplyProgram(const Program *program);

    void ApplyViewport(const Viewport *vp);

    const Viewport *GetViewPort();

    void ApplyProjectionMatrix(const Matrix4x4f *matrix);

    void ApplyModelViewMatrix(const Matrix4x4f *matrix);

   private:
    const Matrix4x4f *m_projection;
    const Matrix4x4f *m_modelView;
    const Viewport *m_viewport;
    const Program *m_program;
};
}  // namespace HAMO

#endif  // HAMO_LINUX_STATE_H
