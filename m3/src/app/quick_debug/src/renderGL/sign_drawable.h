//
// Created by xiang on 2019/12/24.
//

#ifndef HAMO_LINUX_SIGN_DRAWABLE_H
#define HAMO_LINUX_SIGN_DRAWABLE_H

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

class SignDrawable : public Drawable {
   public:
    SignDrawable();

    virtual ~SignDrawable();

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

   public:
    V3f m_stateParam;  //状态参数，目前标示是否选中
};

}  // namespace HAMO

#endif  // HAMO_LINUX_SIGN_DRAWABLE_H
