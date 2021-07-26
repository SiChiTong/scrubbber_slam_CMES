//
// Created by idriver on 2021/03/25.
//

#include <Eigen/Geometry>

#include "renderGL/gl_api.h"
#include "renderGL/graph_drawable.h"
#include "renderGL/point_color.h"

namespace HAMO {

GraphDrawable::GraphDrawable() { vao = vbo = cbo = 0; }
GraphDrawable::~GraphDrawable() {
    glDeleteBuffers(1, &vbo);
    if (cbo) {
        glDeleteBuffers(1, &cbo);
    }

    glDeleteBuffers(1, &ebo);
    glDeleteVertexArrays(1, &vao);
}

void GraphDrawable::add(float x, float y, float z, const V3f &color) {
    m_PointArray.push_back(PointColor(x, y, z, V4f(color[0], color[1], color[2], 1.0)));
}

void GraphDrawable::add(const V3f &vec, const V3f &color) {
    m_PointArray.push_back(PointColor(vec, V4f(color[0], color[1], color[2], 1.0)));
}

void GraphDrawable::modify(int index, const V3f &vec) {
    if (index < m_PointArray.size()) {
        m_PointArray[index].point = vec;
    }
}

void GraphDrawable::DrawImplementation(RenderInfo &renderInfo) {
    Program *pProgram = renderInfo.program_;
    int position_loc = pProgram->attrib_locs_[0];
    int color_loc = pProgram->attrib_locs_[1];

    glBindVertexArray(vao);

    glEnableVertexAttribArray(position_loc);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(position_loc, 3, GL_FLOAT, GL_FALSE, 0, 0);

    if (cbo) {
        glEnableVertexAttribArray(color_loc);
        glBindBuffer(GL_ARRAY_BUFFER, cbo);
        glVertexAttribPointer(color_loc, 4, GL_FLOAT, GL_FALSE, 0, 0);
    }

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glDrawElements(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glDisableVertexAttribArray(position_loc);

    if (cbo) {
        glDisableVertexAttribArray(color_loc);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void GraphDrawable::Init() {
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices_ext(m_PointArray.size() * 4);
    const float line_width = 0.1f;
    for (int i = 0; i < m_PointArray.size(); i += 2) {
        Eigen::Vector3f direction = (m_PointArray[i + 1].point - m_PointArray[i].point).ToEigen();
        Eigen::Vector3f axis = std::abs(direction.normalized().dot(Eigen::Vector3f::UnitZ())) < 0.9f
                                   ? Eigen::Vector3f::UnitZ()
                                   : Eigen::Vector3f::UnitX();

        Eigen::Vector3f x = axis.cross(direction).normalized();
        Eigen::Vector3f y = x.cross(direction).normalized();

        vertices_ext[i * 4] = m_PointArray[i].point.ToEigen() - x * line_width;
        vertices_ext[i * 4 + 1] = m_PointArray[i + 1].point.ToEigen() - x * line_width;
        vertices_ext[i * 4 + 2] = m_PointArray[i].point.ToEigen() + x * line_width;
        vertices_ext[i * 4 + 3] = m_PointArray[i + 1].point.ToEigen() + x * line_width;

        vertices_ext[i * 4 + 4] = m_PointArray[i].point.ToEigen() - y * line_width;
        vertices_ext[i * 4 + 5] = m_PointArray[i + 1].point.ToEigen() - y * line_width;
        vertices_ext[i * 4 + 6] = m_PointArray[i].point.ToEigen() + y * line_width;
        vertices_ext[i * 4 + 7] = m_PointArray[i + 1].point.ToEigen() + y * line_width;
    }

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices_ext.size() * 3, vertices_ext.data(), GL_STATIC_DRAW);

    if (!m_PointArray.empty()) {
        std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors_ext(m_PointArray.size() * 4);
        for (int i = 0; i < m_PointArray.size(); i += 2) {
            for (int j = 0; j < 4; j++) {
                colors_ext[i * 4 + j * 2] = m_PointArray[i].color.ToEigen();
                colors_ext[i * 4 + j * 2 + 1] = m_PointArray[i + 1].color.ToEigen();
            }
        }
        glGenBuffers(1, &cbo);
        glBindBuffer(GL_ARRAY_BUFFER, cbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * colors_ext.size() * 4, colors_ext.data(), GL_STATIC_DRAW);
    }

    std::vector<int> sub_indices = {0, 1, 4, 1, 5, 4, 4, 5, 2, 5, 3, 2, 2, 3, 6, 3, 6, 7, 6, 7, 0, 7, 1, 0};

    std::vector<int> indices;
    for (int i = 0; i < vertices_ext.size(); i += 8) {
        for (int j = 0; j < sub_indices.size(); j++) {
            indices.push_back(sub_indices[j] + i);
        }
    }
    num_indices = indices.size();

    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * indices.size(), indices.data(), GL_STATIC_DRAW);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

}  // namespace HAMO
