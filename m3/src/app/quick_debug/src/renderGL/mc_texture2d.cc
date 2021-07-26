//
// Created by gaoxiang on 2019/12/17.
//

#include "renderGL/mc_texture2d.h"
#include "renderGL/gl_api.h"

namespace HAMO {

Texture2D::Texture2D()
        : handle_(0) {

}

Texture2D::~Texture2D() {

}

bool Texture2D::Create(const Image &image) {
    glGenTextures(1, &handle_);
    if (handle_ > 0) {
        width_ = image.GetWidth();
        height_ = image.GetHeight();

        unsigned char *data = image.GetData();

        glBindTexture(GL_TEXTURE_2D, handle_);

        int glinternalFormat;
        int glformat;
        int gltype;

        MappingFormat(glinternalFormat, glformat, gltype, image.GetFormat());

        glTexImage2D(GL_TEXTURE_2D, 0, glinternalFormat, width_, height_, 0, glformat, gltype, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        return true;
    }

    return false;
}

void Texture2D::MappingFormat(int &internalFormat, int &glformat, int &gltype, Image::PixelFormat ef) {
    switch (ef) {
        case Image::PF_R8G8B8:
            internalFormat = GL_RGB8;
            glformat = GL_RGB;
            gltype = GL_UNSIGNED_BYTE;
            break;
        case Image::PF_R8G8B8A8:
            internalFormat = GL_RGBA8;
            glformat = GL_RGBA;
            gltype = GL_UNSIGNED_BYTE;
            break;
        default:
            break;
    }
}

void Texture2D::Destroy() {
    if (handle_ > 0) {
        glDeleteTextures(1, &handle_);
        handle_ = 0;
    }
}

}