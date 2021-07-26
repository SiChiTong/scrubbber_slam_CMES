//
// Created by gaoxiang on 2019/12/17.
//

#ifndef HAMO_LINUX_MC_UNIFORM_PROGRAM_H
#define HAMO_LINUX_MC_UNIFORM_PROGRAM_H

#include "algorithm/matrix44.h"
#include "algorithm/common.h"

namespace HAMO {

class UniformParam {
public:
    enum {
        MAX_UNIFORM_UNITS = 8,
    };

    enum UNIFORM_TYPE {
        UNV_UNKOWN,
        UNV_INT_VEC4,
        UNV_FLOAT_VALUE,
        UNV_FLOAT_VEC2,
        UNV_FLOAT_VEC3,
        UNV_FLOAT_VEC4,
        UNV_FLOAT_MAT4,
        UNV_QUANTITY
    };

    struct UniformVariable {
        unsigned char name[32];
        unsigned int location;
        UNIFORM_TYPE type;
        unsigned int count;
        unsigned int offset;
    };

public:
    UniformParam();

    ~UniformParam();

public:
    void UniformElement(unsigned int location, UNIFORM_TYPE type, int count = 1);

    void SetParameter(unsigned int index, const float &value);

    void SetParameter(unsigned int index, const V2f &vec2);

    void SetParameter(unsigned int index, const V3f &vec3);

    void SetParameter(unsigned int index, const V4f &vec4);

    void SetParameter(unsigned int index, const Matrix4x4f &mat);

    unsigned int GetUniformNumber() const;

    const void *GetUniformAddress(int offset) const;

    UniformVariable *GetUniformByIndex(int index);

    UniformVariable *GetUniformVariable(int location);

private:
    UniformVariable elements_[MAX_UNIFORM_UNITS];
    unsigned int element_num_;

    unsigned int length_;
    unsigned char *buffer_ = nullptr;

    static int uniform_size_[UNV_QUANTITY];
};

}

#endif //HAMO_LINUX_MC_UNIFORM_PROGRAM_H
