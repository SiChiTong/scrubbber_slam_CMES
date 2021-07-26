//
// Created by xiang on 2019/12/24.
//

#include "renderGL/pointset_drawable.h"
#include "map/projection_utm.h"
#include "renderGL/gl_api.h"
#include "renderGL/mc_buffer.h"
#include "renderGL/mc_render_layout.h"
#include "renderGL/mc_vertex_format.h"

namespace HAMO {

PointSetDrawable::PointSetDrawable() {
    format_ = nullptr;
    buffer_ = nullptr;
    ibuffer_ = nullptr;

    render_layout_ = new RenderLayout();

    bound_box_.Set(V3f(1000.0f, 1000.0f, 1000.0f),
                   V3f(-1000.0f, -1000.0f, -1000.0f));
}

PointSetDrawable::~PointSetDrawable() { delete render_layout_; }

void PointSetDrawable::Update(VertexFormat *vFormat, VertexBuffer *vBuffer,
                              IndexBuffer *vIBuffer) {
    render_layout_->SetTopologyType(RenderLayout::TT_TriangleList);
    render_layout_->BindVertexStream(vBuffer, vFormat);
    render_layout_->BindIndexStream(vIBuffer);

    delete format_;
    delete buffer_;
    delete ibuffer_;

    format_ = vFormat;
    buffer_ = vBuffer;
    ibuffer_ = vIBuffer;
}

void PointSetDrawable::DrawImplementation(RenderInfo &renderInfo) {
    Program *pProgram = renderInfo.program_;
    if (pProgram != NULL) {
        DrawVertexAttr(pProgram, GL_TRIANGLES, format_, buffer_, ibuffer_);
    }
}

}  // namespace HAMO