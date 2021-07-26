//
// Created by gaoxiang on 2019/12/19.
//

#ifndef HAMO_LINUX_RENDER_LEAF_H
#define HAMO_LINUX_RENDER_LEAF_H

#include "renderGL/drawable.h"
#include "renderGL/object.h"

#include "renderGL/mc_render_technique.h"

namespace HAMO {

class RenderLeaf : public Object {
   public:
    RenderLeaf();

    virtual ~RenderLeaf();

    bool Render(RenderInfo &rendinfo, RenderLeaf *previous) const;

    void SetDrawable(Drawable *drawable);

    Drawable *GetDrawable();

    void SetViewport(const Viewport &view);

    void SetModelViewMatrix(const Matrix4x4f &modelview);

    void SetProjectionMatrix(const Matrix4x4f &modelview);

    void SetRenderTechnique(RenderTechnique *technique);

   public:
    Drawable *m_drawable = nullptr;

    Matrix4x4f m_projection;
    Matrix4x4f m_modelview;
    Viewport m_viewport;

    RenderTechnique *m_technique = nullptr;

    float m_depth;
    bool m_dynamic;
    unsigned int m_traversalNumber;

   private:
    RenderLeaf(const RenderLeaf &) {}

    RenderLeaf &operator=(const RenderLeaf &) { return *this; }
};

}  // namespace HAMO

#endif  // HAMO_LINUX_RENDER_LEAF_H
