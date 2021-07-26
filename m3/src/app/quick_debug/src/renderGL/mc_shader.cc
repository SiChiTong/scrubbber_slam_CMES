//
// Created by gaoxiang on 2019/12/17.
//

#include "renderGL/mc_shader.h"
#include <cstdio>
#include "renderGL/gl_api.h"

namespace HAMO {

Shader::Shader() : handle_(0) {}

Shader::~Shader() {}

bool Shader::LoadShaderSource(const char *shaderSource) {
    unsigned int shader = CreateShader();
    if (shader == 0) {
        return false;
    }

    glShaderSource(shader, 1, &shaderSource, NULL);
    glCompileShader(shader);

    if (CheckCompileStatus(shader)) {
        handle_ = shader;
        return true;
    } else {
        glDeleteShader(shader);

        return false;
    }
}

bool Shader::LoadShaderFile(const char *fileName) {
    char *shaderSource = NULL;

    FILE *pfile = fopen(fileName, "rb");
    if (pfile) {
        fseek(pfile, 0, SEEK_END);
        int length = ftell(pfile);
        fseek(pfile, 0, SEEK_SET);

        char *shaderSource = new char[length + 1];

        size_t nReadByte = fread(shaderSource, sizeof(char), length, pfile);
        if (nReadByte != length) {
            delete[] shaderSource;
            fclose(pfile);

            return false;
        } else {
            shaderSource[nReadByte] = 0;

            fclose(pfile);
        }

        if (LoadShaderSource(shaderSource)) {
            delete[] shaderSource;
            return true;
        }

        delete[] shaderSource;
    }

    return false;
}

void Shader::DeleteShader() {
    if (handle_ != 0) {
        glDeleteShader(handle_);
        handle_ = 0;
    }
}

bool Shader::CheckCompileStatus(unsigned int shader) {
    GLint compiled;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);

    if (!compiled) {
        GLint infoLen;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLen);

        if (infoLen > 0) {
            char *pLogMsg = new char[infoLen];

            glGetShaderInfoLog(shader, infoLen, NULL, pLogMsg);
            printf("Error compiling shader: \n%s\n", pLogMsg);

            delete[] pLogMsg;
        }
        return false;
    }
    return true;
}

//-----------------------------------------------------------

//-----------------------------------------------------------

VertexShader::VertexShader() {}

VertexShader::~VertexShader() {}

unsigned int VertexShader::CreateShader() {
    unsigned int shader = glCreateShader(GL_VERTEX_SHADER);

    if (shader != 0) {
        return shader;
    }
    return 0;
}

//-----------------------------------------------------------

FragmentShader::FragmentShader() {}

FragmentShader::~FragmentShader() {}

unsigned int FragmentShader::CreateShader() {
    unsigned int shader = glCreateShader(GL_FRAGMENT_SHADER);

    if (shader != 0) {
        return shader;
    }
    return 0;
}

}  // namespace HAMO