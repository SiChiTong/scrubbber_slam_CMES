//
// Created by idriver on 2021/03/25.
//

#ifndef HAMO_LINUX_GRAPH_DRAWABLE_H
#define HAMO_LINUX_GRAPH_DRAWABLE_H

#include "algorithm/bound_box.h"
#include "renderGL/drawable.h"
#include "renderGL/point3d.h"
#include "renderGL/point_color.h"

#include <vector>

namespace HAMO {

class GraphDrawable : public Drawable {
   public:
    GraphDrawable();

    virtual ~GraphDrawable();

   public:
    int size() { return m_PointArray.size(); }

    void modify(int index, const V3f &vec);

    void add(const V3f &vec, const V3f &color);

    void add(float x, float y, float z, const V3f &color);

    void reset() { m_PointArray.clear(); }

    void DrawImplementation(RenderInfo &renderInfo) override;

    std::vector<PointColor> GetPointArray() { return m_PointArray; }

    void Init();

   private:
    std::vector<std::pair<int, int>> m_lines;

    std::vector<PointColor> m_PointArray;

    GLuint vao;  // vertex array object
    GLuint vbo;  // vertices
    GLuint cbo;  // colors
    GLuint ebo;  // elements

    int num_indices;
};

}  // namespace HAMO

#endif  // HAMO_LINUX_GRAPH_DRAWABLE_H
