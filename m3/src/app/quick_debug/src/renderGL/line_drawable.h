//
// Created by gaoxiang on 2019/12/19.
//

#ifndef HAMO_LINUX_LINE_DRAWABLE_H
#define HAMO_LINUX_LINE_DRAWABLE_H

#include "algorithm/bound_box.h"
#include "renderGL/drawable.h"
#include "renderGL/point3d.h"
#include "renderGL/point_color.h"

#include <vector>

namespace HAMO {

class LineDrawable : public Drawable {
   public:
    LineDrawable();

    virtual ~LineDrawable();

   public:
    int size() { return m_PointArray.size(); }

    void modify(int index, const V3f &vec);

    void add(const V3f &vec, const V3f &color);

    void add(float x, float y, float z, const V3f &color);

    void addline(int start, int count);

    void reset() { m_PointArray.clear(); }

    void DrawImplementation(RenderInfo &renderInfo) override;

    std::vector<PointColor> GetPointArray() { return m_PointArray; }

   private:
    std::vector<std::pair<int, int>> m_lines;

    std::vector<PointColor> m_PointArray;
    BoundBox3f m_boundBox;
};

}  // namespace HAMO

#endif  // HAMO_LINUX_LINE_DRAWABLE_H
