//
// Created by gaoxiang on 2019/12/17.
//

#include "renderGL/mc_program.h"
#include "renderGL/mc_shader.h"
#include "renderGL/gl_api.h"

#include <stdio.h>

namespace HAMO {
Program::Program()
        : handle_(0) {

}

Program::~Program() {

}

bool Program::CreateProgram(VertexShader *vsShader, FragmentShader *psShader) {
    unsigned int programObj = glCreateProgram();
    if (programObj == 0) {
        return false;
    }

    glAttachShader(programObj, vsShader->GetHandle());
    glAttachShader(programObj, psShader->GetHandle());

    glLinkProgram(programObj);

    if (CheckStatus(programObj)) {
        handle_ = programObj;
        return true;
    } else {
        glDeleteProgram(programObj);
        return false;
    }
}

void Program::DeleteProgram() {
    if (handle_ != 0) {
        glDeleteProgram(handle_);
        handle_ = 0;
    }
}

void Program::BindUniformLocation(ConstantDefinition *argment, int nCount) {
    for (int i = 0; i < nCount; i++) {
        int location = glGetUniformLocation(handle_, argment[i].name);
        param_.UniformElement(location, argment[i].type);
    }
}

void Program::ActiveUniform() {
    int nCount = param_.GetUniformNumber();
    for (int i = 0; i < nCount; ++i) {
        UniformParam::UniformVariable *element = param_.GetUniformByIndex(i);

        switch (element->type) {
            case UniformParam::UNV_FLOAT_VALUE: {
                const float *pData = (const float *) param_.GetUniformAddress(element->offset);
                glUniform1fv(element->location, element->count, pData);
            }
                break;
            case UniformParam::UNV_FLOAT_VEC2: {
                const float *pData = (const float *) param_.GetUniformAddress(element->offset);
                glUniform2fv(element->location, element->count, pData);
            }
            case UniformParam::UNV_FLOAT_VEC3: {
                const float *pData = (const float *) param_.GetUniformAddress(element->offset);
                glUniform3fv(element->location, element->count, pData);
            }
                break;
            case UniformParam::UNV_FLOAT_VEC4: {
                const float *pData = (const float *) param_.GetUniformAddress(element->offset);
                glUniform4fv(element->location, element->count, pData);
            }
                break;
            case UniformParam::UNV_FLOAT_MAT4: {
                const float *pData = (const float *) param_.GetUniformAddress(element->offset);
                glUniformMatrix4fv(element->location, element->count, GL_FALSE, pData);
            }
                break;
            case UniformParam::UNV_INT_VEC4: {
                const int *pData = (const int *) param_.GetUniformAddress(element->offset);
                glUniform4iv(element->location, element->count, pData);
            }
                break;
        }
    }

}

void Program::UseProgram() {
    glUseProgram(handle_);
}

bool Program::CheckStatus(unsigned int handle) {
    GLint success;
    glGetProgramiv(handle, GL_LINK_STATUS, &success);

    if (!success) {
        GLint infoLen;
        glGetProgramiv(handle, GL_INFO_LOG_LENGTH, &infoLen);

        if (infoLen > 0) {
            char *pLogMsg = new char[infoLen];
            glGetProgramInfoLog(handle, infoLen, NULL, pLogMsg);
            printf("Error linking shader program: \n%s\n", pLogMsg);

            delete[] pLogMsg;
        }
        return false;
    }
    return true;
}
}
