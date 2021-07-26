//
// Created by gaoxiang on 2019/12/18.
//

#include "renderGL/area_drawable.h"
#include "renderGL/gl_api.h"

namespace HAMO {

AreaDrawable::AreaDrawable() {
    m_pVertexformat = NULL;
    m_pVertexBuffer = NULL;
    m_pIndexBuffer = NULL;
}

AreaDrawable::~AreaDrawable() {
    if (m_pVertexformat != NULL) {
        delete m_pVertexBuffer;
        m_pVertexBuffer = NULL;
    }

    if (m_pVertexBuffer != NULL) {
        delete m_pVertexBuffer;
        m_pVertexBuffer = NULL;
    }

    if (m_pIndexBuffer != NULL) {
        delete m_pIndexBuffer;
        m_pIndexBuffer = NULL;
    }
}

void AreaDrawable::DrawImplementation(RenderInfo &renderInfo) {
    Program *pProgram = renderInfo.program_;
    if (pProgram != NULL) {
        DrawVertexAttr(pProgram, GL_TRIANGLES, m_pVertexformat, m_pVertexBuffer,
                       m_pIndexBuffer);
    }
}

}  // namespace HAMO
