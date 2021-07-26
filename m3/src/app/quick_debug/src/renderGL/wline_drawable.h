//
// Created by gaoxiang on 2019/12/19.
//

#ifndef HAMO_LINUX_WLINE_DRAWABLE_H
#define HAMO_LINUX_WLINE_DRAWABLE_H

#include "algorithm/bound_box.h"
#include "renderGL/drawable.h"
#include "renderGL/point3d.h"
#include "renderGL/point_color.h"

namespace HAMO {

class WLineDrawable : public Drawable {
   public:
    WLineDrawable();

    virtual ~WLineDrawable();

   public:
    int size() { return m_PointArray.size(); }

    void modify(int index, const V3f &vec);

    void remove(int index);

    void add(const V3f &vec, const V3f &color);

    void add(float x, float y, float z, const V3f &color);

    void reset() { m_PointArray.clear(); }

    virtual void DrawImplementation(RenderInfo &renderInfo);

   private:
    bool CreateVBO();

    void DeleteVBO();

   private:
    std::vector<PointColor> m_PointArray;
    BoundBox3f m_boundBox;

    unsigned int m_vboId;
    unsigned int m_vxCount;

   public:
    VertexFormat *m_pVertexformat;
    VertexBuffer *m_pVertexBuffer;
    IndexBuffer *m_pIndexBuffer;

    V3f m_lineParam;
};

}  // namespace HAMO

#endif  // HAMO_LINUX_WLINE_DRAWABLE_H
