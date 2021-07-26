//
// Created by gaoxiang on 2019/12/18.
//

#ifndef HAMO_LINUX_POINT_DRAWABLE_H
#define HAMO_LINUX_POINT_DRAWABLE_H

#include <vector>
#include "algorithm/bound_box.h"
#include "renderGL/drawable.h"
#include "renderGL/point3d.h"
#include "renderGL/point_color.h"

namespace HAMO {

class PointDrawable : public Drawable {
   public:
    PointDrawable();

    virtual ~PointDrawable();

    std::vector<PointColor> GetPointArray() const { return m_PointArray; }

   public:
    void add(float x, float y, float z, const V4f &color);

    BoundBox3f getBound() { return m_boundBox; }

    virtual void DrawImplementation(RenderInfo &renderInfo);

   private:
    bool CreateVBO();

    void DeleteVBO();

   private:
    std::vector<PointColor> m_PointArray;
    BoundBox3f m_boundBox;

    unsigned int m_vboId;
    unsigned int m_vxCount;
};

}  // namespace HAMO
#endif  // HAMO_LINUX_POINT_DRAWABLE_H
