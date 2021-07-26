//
// Created by gaoxiang on 2019/12/12.
//

#ifndef HAMO_LINUX_VIEWPORT_H
#define HAMO_LINUX_VIEWPORT_H

namespace HAMO {
class Viewport {
   public:
    Viewport();

    Viewport(float x, float y, float width, float height);

    Viewport(const Viewport &vp);

   public:
    void SetViewport(float x, float y, float width, float height);

    bool Valid() const;

    float AspectRatio() const;

    float X() const { return x_; }

    float Y() const { return y_; }

    float Width() const { return width_; }

    float Height() const { return height_; }

    float CenterX() const { return x_ + width_ * 0.5f; };

    float CenterY() const { return y_ + height_ * 0.5f; };

   protected:
    float x_;
    float y_;
    float width_;
    float height_;
};

}  // namespace HAMO
#endif  // HAMO_LINUX_VIEWPORT_H
