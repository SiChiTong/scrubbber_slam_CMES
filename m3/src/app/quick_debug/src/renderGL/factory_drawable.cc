//
// Created by gaoxiang on 2019/12/19.
//

#include "factory_drawable.h"
#include "area_drawable.h"
#include "mc_vertex_buffer_accessor.h"

namespace HAMO {

Drawable *FactoryDrawable::CreateNodeDrawable(const std::vector<PointColor> &items, float size, const V4f &clr) {
    if (items.size() > 0) {
        std::vector<V3f> position;
        std::vector<int> indices_array;

        for (const auto &i : items) {
            V3f pnt = i.point;

            V3f origin;
            origin[0] = pnt.x;
            origin[1] = pnt.y;
            //            origin[2] = pnt.z;
            origin[2] = 0;

            int startIdx = position.size();

            position.push_back(V3f(origin[0] - size, origin[1] - size, origin[2]));
            position.push_back(V3f(origin[0] + size, origin[1] - size, origin[2]));
            position.push_back(V3f(origin[0] + size, origin[1] + size, origin[2]));
            position.push_back(V3f(origin[0] - size, origin[1] + size, origin[2]));

            indices_array.push_back(startIdx);
            indices_array.push_back(startIdx + 1);
            indices_array.push_back(startIdx + 3);

            indices_array.push_back(startIdx + 3);
            indices_array.push_back(startIdx + 1);
            indices_array.push_back(startIdx + 2);
        }

        //---------------------------------------------------------------

        VertexFormat *vertexformat = new VertexFormat();
        vertexformat->AppendAttribute(VertexFormat::MCAU_POSITION, VertexFormat::MCAT_FLOAT3, 0);
        vertexformat->AppendAttribute(VertexFormat::MCAU_COLOR, VertexFormat::MCAT_FLOAT4, 0);

        VertexBuffer *vertexBuffer = new VertexBuffer(position.size(), vertexformat->GetStride());
        VertexBufferAccessor vba(vertexformat, vertexBuffer);

        for (size_t i = 0; i < position.size(); i++) {
            vba.Position<V3f>(i) = position[i];
            vba.Color<V4f>(0, i) = clr;
        }

        IndexBuffer *indexBuffer = new IndexBuffer(indices_array.size(), sizeof(int));
        int *indices = (int *)indexBuffer->GetData();
        for (size_t i = 0; i < indices_array.size(); i++) {
            indices[i] = indices_array[i];
        }

        AreaDrawable *pLine = new AreaDrawable();

        pLine->m_pVertexBuffer = vertexBuffer;
        pLine->m_pVertexformat = vertexformat;
        pLine->m_pIndexBuffer = indexBuffer;

        return pLine;
    }
    return NULL;
}

}  // namespace HAMO
