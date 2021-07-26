//
// Created by gaoxiang on 2019/12/12.
//

#include "algorithm/viewport.h"

namespace HAMO {

Viewport::Viewport() : x_(0), width_(0), y_(0), height_(0) {}

Viewport::Viewport(float x, float y, float width, float height)
    : x_(x), y_(y), width_(width), height_(height) {}

Viewport::Viewport(const Viewport &vp)
    : x_(vp.x_), y_(vp.y_), width_(vp.width_), height_(vp.height_) {}

void Viewport::SetViewport(float x, float y, float width, float height) {
    x_ = x;
    y_ = y;
    width_ = width;
    height_ = height;
}

bool Viewport::Valid() const { return (width_ > 0.0f && height_ > 0.0f); }

float Viewport::AspectRatio() const {
    if (height_ != 0) {
        return width_ / height_;
    } else {
        return 1.0f;
    }
}

}  // namespace HAMO
