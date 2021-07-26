//
// Created by gaoxiang on 2019/12/17.
//

#include "renderGL/mc_vertex_buffer_accessor.h"
#include "renderGL/mc_vertex_format.h"
#include "renderGL/mc_buffer.h"

#include <cstdio>

namespace HAMO {

VertexBufferAccessor::VertexBufferAccessor() {

}

VertexBufferAccessor::VertexBufferAccessor(VertexFormat *vformat, VertexBuffer *vbuffer) {
    format_ = vformat;
    buffer_ = vbuffer;

    Apply();
}

VertexBufferAccessor::~VertexBufferAccessor() {

}

//----------------------------------------------------------------------------
bool VertexBufferAccessor::HasPosition() const {
    return position_ != 0;
}

bool VertexBufferAccessor::HasNormal() const {
    return normal_ != 0;
}

bool VertexBufferAccessor::HasColor(int unit) const {
    return color_[unit] != 0;
}

bool VertexBufferAccessor::HasTCoord(int unit) const {
    return tcoord_[unit] != 0;
}

void VertexBufferAccessor::Apply() {
    stride_ = format_->GetStride();
    data_ = buffer_->GetData();

    int baseType = (int) VertexFormat::MCAU_NONE;
    int type;

    int index = format_->GetIndex(VertexFormat::MCAU_POSITION);
    if (index >= 0) {
        position_ = data_ + format_->GetOffset(index);
        type = (int) format_->GetAttributeType(index);
        position_channels_ = type - baseType;
        if (position_channels_ > 4) {
            position_channels_ = 0;
        }
    } else {
        position_ = NULL;
        position_channels_ = 0;
    }

    for (int unit = 0; unit < VertexFormat::MC_MAX_TCOORD_UNITS; ++unit) {
        index = format_->GetIndex(VertexFormat::MCAU_TEXCOORD, unit);
        if (index >= 0) {
            tcoord_[unit] = data_ + format_->GetOffset(index);
            type = (int) format_->GetAttributeType(index);
            coord_channels_[unit] = type - baseType;
            if (coord_channels_[unit] > 4) {
                coord_channels_[unit] = 0;
            }
        } else {
            tcoord_[unit] = NULL;
            coord_channels_[unit] = 0;
        }
    }

    index = format_->GetIndex(VertexFormat::MCAU_NORMAL);
    if (index >= 0) {
        normal_ = data_ + format_->GetOffset(index);
        type = (int) format_->GetAttributeType(index);
        normal_channels_ = type - baseType;
        if (normal_channels_ > 4) {
            normal_channels_ = 0;
        }
    } else {
        normal_ = NULL;
        normal_channels_ = 0;
    }

    for (int unit = 0; unit < VertexFormat::MC_MAX_COLOR_UNITS; unit++) {
        index = format_->GetIndex(VertexFormat::MCAU_COLOR, unit);
        if (index >= 0) {
            color_[unit] = data_ + format_->GetOffset(index);
            type = (int) format_->GetAttributeType(index);
            color_channels_[unit] = type - baseType;
            if (color_channels_[unit] > 4) {
                color_channels_[unit] = 0;
            }
        } else {
            color_[unit] = NULL;
            color_channels_[unit] = 0;
        }
    }

}

}