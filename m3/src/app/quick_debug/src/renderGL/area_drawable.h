//
// Created by gaoxiang on 2019/12/18.
//

#ifndef HAMO_LINUX_AREA_DRAWABLE_H
#define HAMO_LINUX_AREA_DRAWABLE_H

#include "renderGL/drawable.h"
#include "renderGL/mc_buffer.h"
#include "renderGL/mc_hdw_vertex_buffer.h"
#include "renderGL/mc_vertex_buffer_accessor.h"
#include "renderGL/mc_vertex_format.h"
#include "renderGL/point3d.h"
#include "renderGL/point_color.h"

#include "algorithm/bound_box.h"

#include <vector>

namespace HAMO {

class AreaDrawable : public Drawable {
   public:
    AreaDrawable();

    virtual ~AreaDrawable();

   public:
    virtual void DrawImplementation(RenderInfo &renderInfo);

   private:
    BoundBox3f m_boundBox;

   public:
    VertexFormat *m_pVertexformat;
    VertexBuffer *m_pVertexBuffer;
    IndexBuffer *m_pIndexBuffer;
};

}  // namespace HAMO

#endif  // HAMO_LINUX_AREA_DRAWABLE_H
