//
// Created by gaoxiang on 2019/12/17.
//

#include "renderGL/mc_render_layout.h"
#include "renderGL/mc_buffer.h"
#include "renderGL/mc_program.h"
#include "renderGL/mc_vertex_format.h"

namespace HAMO {

RenderLayout::RenderLayout()
        : topo_type_(TT_PointList), buffer_(NULL),
          format_(NULL), ibuffer_(NULL) {

}

RenderLayout::~RenderLayout() {
}

void RenderLayout::SetTopologyType(TopologyType type) {
    topo_type_ = type;
}

RenderLayout::TopologyType RenderLayout::GetTopologyType() const {
    return topo_type_;
}

void RenderLayout::BindVertexStream(VertexBuffer *vBuffer, VertexFormat *vFormat) {
    buffer_ = vBuffer;
    format_ = vFormat;
}

void RenderLayout::BindIndexStream(IndexBuffer *iBuffer) {
    ibuffer_ = iBuffer;
}

bool RenderLayout::UseIndices() const {
    return (ibuffer_ != NULL);
}

void RenderLayout::Active(Program const &so) const {

}

}
