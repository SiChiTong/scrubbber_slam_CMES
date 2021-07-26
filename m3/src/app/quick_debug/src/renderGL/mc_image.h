//
// Created by gaoxiang on 2019/12/17.
//

#ifndef HAMO_LINUX_MC_IMAGE_H
#define HAMO_LINUX_MC_IMAGE_H

namespace HAMO {

class Image {
public:
    enum PixelFormat {
        PF_UNKNOWN = 0,
        PF_R8G8B8 = 10,
        PF_B8G8R8 = 11,
        PF_A8R8G8B8 = 12,
        PF_A8B8G8R8 = 13,
        PF_B8G8R8A8 = 14,
        PF_R8G8B8A8 = 28,
        PF_COUNT = 44
    };

public:
    Image();

    Image(const Image &img);

    virtual ~Image();

public:
    Image &LoadImage(unsigned char *data, unsigned int width, unsigned int height, PixelFormat format);

    bool LoadImage(const char *filename);

    Image &FlipAroundX();

    void FreeMemory();

    unsigned char *GetData(void) const;

    unsigned int GetSize() const;

    unsigned int GetWidth(void) const;

    unsigned int GetHeight(void) const;

    unsigned int GetRowSpan(void) const;

    PixelFormat GetFormat() const;

    unsigned int GetBPP() const;

    unsigned int calculatePixelSize(PixelFormat format);

protected:
    unsigned char pixel_size_;
    PixelFormat format_;

    unsigned int width_;
    unsigned int height_;

    int buf_size_;
    unsigned char *buffer_;
};

}

#endif //HAMO_LINUX_MC_IMAGE_H
