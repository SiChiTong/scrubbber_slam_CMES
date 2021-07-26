//
// Created by gaoxiang on 2019/12/17.
//

#ifndef HAMO_LINUX_MC_TEXTURE2D_H
#define HAMO_LINUX_MC_TEXTURE2D_H

#include "renderGL/mc_image.h"

namespace HAMO {
class Texture2D {
public:
    Texture2D();

    virtual ~Texture2D();

    int GetTextureWidth() const { return width_; }

    int GetTextureHeight() const { return height_; }

    bool IsValidate() { return (handle_ != 0); }

    unsigned int GetHandle() { return handle_; }

    bool Create(const Image &image);

    void Destroy();

private:
    void MappingFormat(int &internalFormat, int &glformat, int &gltype, Image::PixelFormat ef);

private:
    int width_;
    int height_;
    unsigned int handle_;
};

}

#endif //HAMO_LINUX_MC_TEXTURE2D_H
