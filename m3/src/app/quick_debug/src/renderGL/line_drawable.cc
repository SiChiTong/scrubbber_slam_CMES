//
// Created by gaoxiang on 2019/12/19.
//

#include "renderGL/line_drawable.h"
#include "renderGL/gl_api.h"
#include "renderGL/mc_buffer.h"
#include "renderGL/mc_hdw_vertex_buffer.h"
#include "renderGL/mc_vertex_buffer_accessor.h"
#include "renderGL/mc_vertex_format.h"
#include "renderGL/point_color.h"

namespace HAMO {

LineDrawable::LineDrawable() {}

LineDrawable::~LineDrawable() {}

void LineDrawable::add(float x, float y, float z, const V3f &color) {
    m_boundBox.ExpandBy(x, y, z);
    m_PointArray.push_back(PointColor(x, y, z, V4f(color[0], color[1], color[2], 1.0)));
}

void LineDrawable::add(const V3f &vec, const V3f &color) {
    m_boundBox.ExpandBy(vec[0], vec[1], vec[2]);
    m_PointArray.push_back(PointColor(vec, V4f(color[0], color[1], color[2], 1.0)));
}

void LineDrawable::addline(int start, int count) { m_lines.push_back(std::pair<int, int>(start, count)); }

void LineDrawable::modify(int index, const V3f &vec) {
    if (index < m_PointArray.size()) {
        m_boundBox.ExpandBy(vec[0], vec[1], vec[2]);
        m_PointArray[index].point = vec;
    }
}

void LineDrawable::DrawImplementation(RenderInfo &renderInfo) {
    Program *pProgram = renderInfo.program_;
    int posLoc = pProgram->attrib_locs_[0];
    int clorLoc = pProgram->attrib_locs_[1];
    State *pState = renderInfo.GetState();
    StateAttribute::GLModeValue islineStipple = m_stateset.getMode(StateAttribute::Type::LINESTIPPLE);
    UniformParam *param = pProgram->GetUniformParam();
    if (param->GetUniformNumber() > 2) {  // >2 ????
        const Viewport *viewport = pState->GetViewPort();
        param->SetParameter(2, V2f(viewport->Width(), viewport->Height()));
    }
    pProgram->ActiveUniform();
    if (m_PointArray.size() > 1) {
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glLineWidth(3.0);
        if (StateAttribute::ON == islineStipple) {
            glEnable(GL_LINE_STIPPLE);
            glLineStipple(1, 0x3F3F);
        }
        char *vertex = (char *)&m_PointArray[0];
        glVertexAttribPointer(posLoc, 3, GL_FLOAT, false, sizeof(PointColor), vertex);
        glVertexAttribPointer(clorLoc, 4, GL_FLOAT, false, sizeof(PointColor), vertex + sizeof(V3f));

        glEnableVertexAttribArray(posLoc);
        glEnableVertexAttribArray(clorLoc);

        if (m_lines.size() > 0) {
            for (int i = 0; i < m_lines.size(); i++) {
                int start = m_lines[i].first;
                int count = m_lines[i].second;

                glDrawArrays(GL_LINE_STRIP, start, count);
            }
        } else {
            int vxCount = m_PointArray.size();
            glDrawArrays(GL_LINE_STRIP, 0, vxCount);
        }

        glLineWidth(1.0);
        glDisable(GL_LINE_STIPPLE);
        glDisableVertexAttribArray(posLoc);
        glDisableVertexAttribArray(clorLoc);
    }
}

}  // namespace HAMO
