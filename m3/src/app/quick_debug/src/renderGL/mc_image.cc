//
// Created by gaoxiang on 2019/12/17.
//

#include <stdio.h>
#include <memory.h>
#include "libpng/png.h"
#include "renderGL/mc_image.h"

namespace HAMO {

//-----------------------------------------------------------------------------
Image::Image()
        : width_(0), height_(0),
          buf_size_(0), buffer_(NULL),
          format_(PF_UNKNOWN) {

}

//-----------------------------------------------------------------------------
Image::Image(const Image &img)
        : buffer_(NULL) {
    FreeMemory();

    width_ = img.width_;
    height_ = img.height_;
    format_ = img.format_;
    buf_size_ = img.buf_size_;
    pixel_size_ = img.pixel_size_;

    buffer_ = new unsigned char[buf_size_];
    memcpy(buffer_, img.buffer_, buf_size_);
}

//-----------------------------------------------------------------------------
Image::~Image() {
    FreeMemory();
}

//---------------------------------------------------------------------
void Image::FreeMemory() {
    if (buffer_) {
        delete[] buffer_;
        buffer_ = NULL;
    }

}

//-----------------------------------------------------------------------------
Image &Image::FlipAroundX() {
    if (buffer_ != NULL) {

        size_t rowSpan = width_ * pixel_size_;

        unsigned char *pTempBuffer = new unsigned char[rowSpan * height_];
        unsigned char *ptr1 = buffer_;
        unsigned char *ptr2 = pTempBuffer + ((height_ - 1) * rowSpan);

        for (size_t i = 0; i < height_; i++) {
            memcpy(ptr2, ptr1, rowSpan);
            ptr1 += rowSpan;
            ptr2 -= rowSpan;
        }

        memcpy(buffer_, pTempBuffer, rowSpan * height_);

        delete[] pTempBuffer;
    }
    return *this;
}

//-----------------------------------------------------------------------------

Image &Image::LoadImage(unsigned char *pData, unsigned int uWidth, unsigned int uHeight, PixelFormat eFormat) {

    FreeMemory();

    width_ = uWidth;
    height_ = uHeight;
    format_ = eFormat;
    pixel_size_ = calculatePixelSize(format_);

    buf_size_ = uHeight * (uWidth * pixel_size_);
    buffer_ = pData;

    return *this;

}

//-----------------------------------------------------------------------------
bool Image::LoadImage(const char *strFileName) {
    FILE *fp;
    png_structp png_ptr;
    png_infop info_ptr;
    png_bytep *row_pointers;

    int w, h, x, y, color_type;

    fp = fopen(strFileName, "rb");
    if (!fp) {
        return false;
    }

    png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
    if (!png_ptr) {
        fclose(fp);
        return false;
    }

    info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        fclose(fp);
        return false;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        fclose(fp);
        return false;
    }

    png_init_io(png_ptr, fp);

    png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_EXPAND, 0);

    color_type = png_get_color_type(png_ptr, info_ptr);

    w = png_get_image_width(png_ptr, info_ptr);
    h = png_get_image_height(png_ptr, info_ptr);

    row_pointers = png_get_rows(png_ptr, info_ptr);

    unsigned char *pData = NULL;
    PixelFormat eFormat = PF_UNKNOWN;

    if (color_type == PNG_COLOR_TYPE_RGB_ALPHA) {
        pData = new unsigned char[h * w * 4];
        eFormat = PF_R8G8B8A8;

        for (y = 0; y < h; ++y) {
            unsigned char *ptrRow = pData + (w * 4) * y;
            for (x = 0; x < w; x++) {
                ptrRow[4 * x + 0] = row_pointers[y][x * 4 + 0]; // red
                ptrRow[4 * x + 1] = row_pointers[y][x * 4 + 1]; // green
                ptrRow[4 * x + 2] = row_pointers[y][x * 4 + 2]; // blue
                ptrRow[4 * x + 3] = row_pointers[y][x * 4 + 3]; // alpha
            }
        }
    } else if (color_type == PNG_COLOR_TYPE_RGB) {
        pData = new unsigned char[h * w * 3];
        eFormat = PF_R8G8B8;

        for (y = 0; y < h; ++y) {
            unsigned char *ptrRow = pData + (w * 3) * y;
            for (x = 0; x < w; x++) {
                ptrRow[3 * x + 0] = row_pointers[y][x * 3 + 0]; // red
                ptrRow[3 * x + 1] = row_pointers[y][x * 3 + 1]; // green
                ptrRow[3 * x + 2] = row_pointers[y][x * 3 + 2]; // blue
            }
        }
    }
    png_destroy_read_struct(&png_ptr, &info_ptr, 0);

    fclose(fp);

    if (pData != NULL && eFormat != PF_UNKNOWN) {
        LoadImage(pData, w, h, eFormat);
        return true;
    }

    return false;
}

//-----------------------------------------------------------------------------
unsigned char *Image::GetData() const {
    return buffer_;
}

//-----------------------------------------------------------------------------
unsigned int Image::GetSize() const {
    return buf_size_;
}

//-----------------------------------------------------------------------------
unsigned int Image::GetWidth() const {
    return width_;
}

//-----------------------------------------------------------------------------
unsigned int Image::GetHeight() const {
    return height_;
}

//-----------------------------------------------------------------------------
unsigned int Image::GetRowSpan() const {
    return width_ * pixel_size_;
}

//-----------------------------------------------------------------------------
Image::PixelFormat Image::GetFormat() const {
    return format_;
}

//----------------------------------------------------------------------------
unsigned int Image::GetBPP() const {
    return pixel_size_ * 8;
}

//-----------------------------------------------------------------------------
unsigned int Image::calculatePixelSize(PixelFormat format) {
    int numElem = 0;

    switch (format) {
        case Image::PF_R8G8B8:
        case Image::PF_B8G8R8:
            numElem = 3;
            break;
        case Image::PF_A8R8G8B8:
        case Image::PF_A8B8G8R8:
        case Image::PF_B8G8R8A8:
        case Image::PF_R8G8B8A8:
            numElem = 4;
            break;
    }
    return numElem;
}

}

