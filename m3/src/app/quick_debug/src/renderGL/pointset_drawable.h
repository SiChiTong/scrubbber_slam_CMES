//
// Created by xiang on 2019/12/24.
//

#ifndef HAMO_LINUX_POINTSET_DRAWABLE_H
#define HAMO_LINUX_POINTSET_DRAWABLE_H

#include "renderGL/drawable.h"
#include "renderGL/point3d.h"
#include "renderGL/point_color.h"

#include "algorithm/bound_box.h"

#include <vector>

namespace HAMO {

class RenderLayout;

class VertexFormat;

class VertexBuffer;

class IndexBuffer;

class PointSetDrawable : public Drawable {
   public:
    PointSetDrawable();

    virtual ~PointSetDrawable();

   public:
    void Update(VertexFormat *vFormat, VertexBuffer *vBuffer,
                IndexBuffer *vIBuffer);

    BoundBox3f getBound() { return bound_box_; }

    virtual void DrawImplementation(RenderInfo &renderInfo);

   private:
    BoundBox3f bound_box_;
    RenderLayout *render_layout_;
    VertexFormat *format_;
    VertexBuffer *buffer_;
    IndexBuffer *ibuffer_;
};
}  // namespace HAMO

#endif  // HAMO_LINUX_POINTSET_DRAWABLE_H
