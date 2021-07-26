//
// Created by gaoxiang on 2019/12/18.
//

#include <cstdio>

#include "renderGL/gl_api.h"
#include "renderGL/state.h"

namespace HAMO {
State::State()
    : m_projection(NULL),
      m_modelView(NULL),
      m_viewport(NULL),
      m_program(NULL) {}

void State::initializeProcs() {}

void State::ApplyViewport(const Viewport *vp) {
    if (vp) {
        m_viewport = vp;
    }
    glViewport(vp->X(), vp->Y(), vp->Width(), vp->Height());
}

const Viewport *State::GetViewPort() { return m_viewport; }

void State::ApplyProgram(const Program *program) {
    if (m_program != program) {
        if (program) {
            m_program = program;
        }
        glUseProgram(program->GetHandle());
    }
}

void State::ApplyProjectionMatrix(const Matrix4x4f *matrix) {
    if (m_projection != matrix) {
        if (matrix) {
            m_projection = matrix;
        }
        int prjLoc = 0;
        glUniformMatrix4fv(prjLoc, 1, GL_FALSE, &matrix->mat[0]);
    }
}

void State::ApplyModelViewMatrix(const Matrix4x4f *matrix) {
    if (m_modelView != matrix) {
        if (matrix) {
            m_modelView = matrix;
        }
        int modelLoc = 1;
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, &matrix->mat[0]);
    }
}
}  // namespace HAMO
