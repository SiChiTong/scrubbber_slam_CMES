//
// Created by gaoxiang on 2019/12/17.
//

#include "renderGL/mc_vertex_format.h"
#include <cassert>

namespace HAMO {

int VertexFormat::type_size_[MCAT_QUANTITY] = {
        0,    // MCAT_NONE
        4,    // MCAT_FLOAT1
        8,    // MCAT_FLOAT2
        12,   // MCAT_FLOAT3
        16,   // MCAT_FLOAT4
        4,    // MCAT_UBYTE4
        2,    // MCAT_SHORT1
        4,    // MCAT_SHORT2
        8     // MCAT_SHORT2
};

int VertexFormat::component_size_[MCAT_QUANTITY] = {
        0,  // MCAT_NONE
        4,  // MCAT_FLOAT1
        4,  // MCAT_FLOAT2
        4,  // MCAT_FLOAT3
        4,  // MCAT_FLOAT4
        1,  // MCAT_UBYTE4
        2,  // MCAT_SHORT1
        2,  // MCAT_SHORT2
        2   // MCAT_SHORT4
};

int VertexFormat::num_components_[MCAT_QUANTITY] = {
        0,  // MCAT_NONE
        1,  // MCAT_FLOAT1
        2,  // MCAT_FLOAT2
        3,  // MCAT_FLOAT3
        4,  // MCAT_FLOAT4
        4,  // MCAT_UBYTE4
        1,  // MCAT_SHORT1
        2,  // MCAT_SHORT2
        4   // MCAT_SHORT2
};

VertexFormat::VertexFormat()
        : num_attributes_(0),
          stride_(0) {
    for (int i = 0; i < MC_MAX_ATTRIBUTES; ++i) {
        elements_[i].stream_index = 0;
        elements_[i].offset = 0;
        elements_[i].type = MCAT_NONE;
        elements_[i].usage = MCAU_NONE;
        elements_[i].usage_index = 0;
    }

}

VertexFormat::~VertexFormat() {

}

int VertexFormat::GetIndex(AttributeUsage usage, unsigned int usageIndex) const {
    for (int i = 0; i < num_attributes_; ++i) {
        if (elements_[i].usage == usage &&
            elements_[i].usage_index == usageIndex) {
            return i;
        }
    }

    return -1;
}

void VertexFormat::SetAttribute(int attribute, unsigned int streamIndex, unsigned int offset,
                                AttributeType type, AttributeUsage usage, unsigned int usageIndex) {
    assert(0 <= attribute && attribute < num_attributes_);

    Element &element = elements_[attribute];
    element.stream_index = streamIndex;
    element.offset = offset;
    element.type = type;
    element.usage = usage;
    element.usage_index = usageIndex;
}

void VertexFormat::GetAttribute(int attribute, unsigned int &streamIndex, unsigned int &offset,
                                AttributeType &type, AttributeUsage &usage, unsigned int &usageIndex) const {
    assert(0 <= attribute && attribute < num_attributes_);

    const Element &element = elements_[attribute];
    streamIndex = element.stream_index;
    offset = element.offset;
    type = element.type;
    usage = element.usage;
    usageIndex = element.usage_index;
}

int VertexFormat::GetStride() const {
    return stride_;
}

int VertexFormat::GetNumAttributes() const {
    return num_attributes_;
}

unsigned int VertexFormat::GetStreamIndex(int attribute) const {
    if (0 <= attribute && attribute < num_attributes_) {
        return elements_[attribute].stream_index;
    } else {
        assert(false);
        return 0;
    }
}

unsigned int VertexFormat::GetOffset(int attribute) const {
    if (0 <= attribute && attribute < num_attributes_) {
        return elements_[attribute].offset;
    } else {
        assert(false);
        return 0;
    }
}

VertexFormat::AttributeType VertexFormat::GetAttributeType(int attribute) const {
    if (0 <= attribute && attribute < num_attributes_) {
        return elements_[attribute].type;
    } else {
        assert(false);
        return MCAT_NONE;
    }
}

VertexFormat::AttributeUsage VertexFormat::GetAttributeUsage(int attribute) const {
    if (0 <= attribute && attribute < num_attributes_) {
        return elements_[attribute].usage;
    } else {
        assert(false);
        return MCAU_NONE;
    }
}

unsigned int VertexFormat::GetUsageIndex(int attribute) const {
    if (0 <= attribute && attribute < num_attributes_) {
        return elements_[attribute].usage_index;
    } else {
        assert(false);
        return 0;
    }
}

void VertexFormat::AppendAttribute(AttributeUsage usage, AttributeType type, unsigned int usageIndex) {
    assert(num_attributes_ < MC_MAX_ATTRIBUTES);

    int attribute = num_attributes_++;
    int streamIndex = 0;
    int offset = stride_;

    Element &element = elements_[attribute];
    element.stream_index = streamIndex;
    element.offset = offset;
    element.type = type;
    element.usage = usage;
    element.usage_index = usageIndex;

    stride_ += type_size_[type];
}

int VertexFormat::GetComponentSize(AttributeType type) {
    return component_size_[type];
}

int VertexFormat::GetNumComponents(AttributeType type) {
    return num_components_[type];
}

int VertexFormat::GetTypeSize(AttributeType type) {
    return type_size_[type];
}

}