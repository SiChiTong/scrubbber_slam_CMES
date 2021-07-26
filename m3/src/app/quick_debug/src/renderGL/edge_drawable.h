//
// Created by gaoxiang on 2019/12/19.
//

#ifndef HAMO_LINUX_EDGE_DRAWABLE_H
#define HAMO_LINUX_EDGE_DRAWABLE_H

#include "algorithm/bound_box.h"
#include "renderGL/drawable.h"
#include "renderGL/point3d.h"
#include "renderGL/point_color.h"

#include <vector>

namespace HAMO {

class RenderLayout;

class VertexFormat;

class VertexBuffer;

class IndexBuffer;

class EgdeDrawable : public Drawable {
   public:
    EgdeDrawable();

    virtual ~EgdeDrawable();

   public:
    void Update(VertexFormat *vFormat, VertexBuffer *vBuffer,
                IndexBuffer *vIBuffer);

    BoundBox3f getBound() { return m_boundBox; }

    virtual void DrawImplementation(RenderInfo &renderInfo);

   private:
    BoundBox3f m_boundBox;
    RenderLayout *m_pRenderLayout;
    VertexFormat *m_vFormat;
    VertexBuffer *m_vBuffer;
    IndexBuffer *m_vIBuffer;
};

}  // namespace HAMO
#endif  // HAMO_LINUX_EDGE_DRAWABLE_H
