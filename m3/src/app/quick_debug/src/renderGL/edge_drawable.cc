//
// Created by gaoxiang on 2019/12/19.
//

#include "renderGL/edge_drawable.h"
#include "map/projection_utm.h"
#include "renderGL/gl_api.h"
#include "renderGL/mc_buffer.h"
#include "renderGL/mc_render_layout.h"
#include "renderGL/mc_vertex_format.h"

namespace HAMO {

EgdeDrawable::EgdeDrawable() {
    m_vFormat = nullptr;
    m_vBuffer = nullptr;
    m_vIBuffer = nullptr;

    m_pRenderLayout = new RenderLayout();

    m_boundBox.Set(V3f(1000.0f, 1000.0f, 1000.0f),
                   V3f(-1000.0f, -1000.0f, -1000.0f));
}

EgdeDrawable::~EgdeDrawable() { delete m_pRenderLayout; }

void EgdeDrawable::Update(VertexFormat *vFormat, VertexBuffer *vBuffer,
                          IndexBuffer *vIBuffer) {
    m_pRenderLayout->SetTopologyType(RenderLayout::TT_TriangleList);
    m_pRenderLayout->BindVertexStream(vBuffer, vFormat);
    m_pRenderLayout->BindIndexStream(vIBuffer);

    delete m_vFormat;
    delete m_vBuffer;
    delete m_vIBuffer;

    m_vFormat = vFormat;
    m_vBuffer = vBuffer;
    m_vIBuffer = vIBuffer;
}

void EgdeDrawable::DrawImplementation(RenderInfo &renderInfo) {
    Program *pProgram = renderInfo.program_;
    if (pProgram != NULL) {
        DrawVertexAttr(pProgram, GL_LINES, m_vFormat, m_vBuffer, m_vIBuffer);
    }
}

}  // namespace HAMO
