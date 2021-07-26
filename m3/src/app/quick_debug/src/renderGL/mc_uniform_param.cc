//
// Created by gaoxiang on 2019/12/17.
//

#include "mc_uniform_param.h"

#include <assert.h>
#include <cstring>

namespace HAMO {

int UniformParam::uniform_size_[UNV_QUANTITY] = {
        0,  // UNV_UNKOWN,
        16, // UNV_INT_VEC4,
        4,  // UNV_FLOAT_VALUE,
        8,  // UNV_FLOAT_VEC2,
        12, // UNV_FLOAT_VEC3,
        16, // UNV_FLOAT_VEC4,
        64  // UNV_FLOAT_MAT4,
};

UniformParam::UniformParam()
        : element_num_(0), buffer_(NULL), length_(0) {

}

UniformParam::~UniformParam() {
    if (buffer_) {
        delete[] buffer_;
        buffer_ = NULL;
    }

}

void UniformParam::UniformElement(unsigned int location, UNIFORM_TYPE type, int count) {
    UniformVariable *element = GetUniformVariable(location);

    if (!element) {
        element = &elements_[element_num_++];

        element->type = type;
        element->count = count;
        element->location = location;
        element->offset = length_;

        length_ += uniform_size_[type] * count;
        if (buffer_) {
            delete[] buffer_;
        }
        buffer_ = new unsigned char[length_];
    }
}

void UniformParam::SetParameter(unsigned int index, const float &value) {
    UniformVariable *element = GetUniformByIndex(index);

    assert(element != NULL && element->type == UNV_FLOAT_VALUE);

    element->count = 1;

    unsigned char *pData = buffer_ + element->offset;
    memcpy(pData, &value, sizeof(float));

}

void UniformParam::SetParameter(unsigned int index, const V2f &vec2) {
    UniformVariable *element = GetUniformByIndex(index);

    if (element != NULL && element->type == UNV_FLOAT_VEC2) {
    } else {
        return;
    }
    element->count = 1;

    unsigned char *pData = buffer_ + element->offset;
    memcpy(pData, &vec2, sizeof(V2f));
}

void UniformParam::SetParameter(unsigned int index, const V3f &vec3) {
    UniformVariable *element = GetUniformByIndex(index);

    assert(element != NULL && element->type == UNV_FLOAT_VEC3);

    element->count = 1;

    unsigned char *pData = buffer_ + element->offset;
    memcpy(pData, &vec3, sizeof(V3f));
}

void UniformParam::SetParameter(unsigned int index, const V4f &vec4) {
    UniformVariable *element = GetUniformByIndex(index);

    assert(element != NULL && element->type == UNV_FLOAT_VEC4);

    element->count = 1;

    unsigned char *pData = buffer_ + element->offset;
    memcpy(pData, &vec4, sizeof(V4f));
}

void UniformParam::SetParameter(unsigned int index, const Matrix4x4f &mat) {
    UniformVariable *element = GetUniformByIndex(index);

    assert(element != NULL && element->type == UNV_FLOAT_MAT4);

    element->count = 1;

    unsigned char *pData = buffer_ + element->offset;
    memcpy(pData, &mat, sizeof(Matrix4x4f));
}

UniformParam::UniformVariable *UniformParam::GetUniformVariable(int location) {
    for (unsigned int i = 0; i < element_num_; i++) {
        if (elements_[i].location == location) {
            return &elements_[i];
        }
    }
    return nullptr;
}

UniformParam::UniformVariable *UniformParam::GetUniformByIndex(int index) {
    assert(index >= 0 && index < element_num_);

    return &elements_[index];
}

unsigned int UniformParam::GetUniformNumber() const {
    return element_num_;
}

const void *UniformParam::GetUniformAddress(int offset) const {
    assert(offset >= 0 && offset < length_);

    return (buffer_ + offset);
}

}

