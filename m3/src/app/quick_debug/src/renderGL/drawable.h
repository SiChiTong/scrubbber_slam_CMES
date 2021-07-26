//
// Created by gaoxiang on 2019/12/18.
//

#ifndef HAMO_LINUX_DRAWABLE_H
#define HAMO_LINUX_DRAWABLE_H

#include "algorithm/bound_box.h"
#include "renderGL/object.h"
#include "renderGL/render_info.h"
#include "renderGL/state_set.h"

#include "renderGL/mc_buffer.h"
#include "renderGL/mc_program.h"
#include "renderGL/mc_vertex_format.h"

namespace HAMO {

class Drawable : public Object {
   public:
    Drawable();

    virtual ~Drawable();

   public:
    void Draw(RenderInfo &renderInfo);

   public:
    virtual bool IsValid() const;

    virtual void DrawImplementation(RenderInfo &renderInfo) = 0;
    inline StateSet &getStateSet() { return m_stateset; };

   protected:
    void DrawVertexAttr(Program *pProgram, int drawMode,
                        VertexFormat *pVertexformat,
                        VertexBuffer *pVertexBuffer, IndexBuffer *pIndexBuffer);

   protected:
    StateSet m_stateset;
    BoundBox3f m_boundbox;
};

}  // namespace HAMO

#endif  // HAMO_LINUX_DRAWABLE_H
