//
// Created by gaoxiang on 2019/12/18.
//

#include "renderGL/drawable.h"
#include "renderGL/mc_vertex_buffer_accessor.h"

namespace HAMO {

Drawable::Drawable() {}

Drawable::~Drawable() {}

bool Drawable::IsValid() const { return true; }

void Drawable::Draw(RenderInfo &renderInfo) { DrawImplementation(renderInfo); }

void Drawable::DrawVertexAttr(Program *pProgram, int drawMode,
                              VertexFormat *pVertexformat,
                              VertexBuffer *pVertexBuffer,
                              IndexBuffer *pIndexBuffer) {
    int posLoc = pProgram->attrib_locs_[0];
    int clorLoc = pProgram->attrib_locs_[1];

    if (pVertexformat != NULL && pVertexBuffer != NULL &&
        pIndexBuffer != NULL) {
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        VertexBufferAccessor vba(pVertexformat, pVertexBuffer);
        char *pData = pVertexBuffer->GetData();
        glVertexAttribPointer(posLoc, 3, GL_FLOAT, false,
                              pVertexformat->GetStride(),
                              (void *)&(vba.Position<V3f>(0)));
        glVertexAttribPointer(clorLoc, 4, GL_FLOAT, false,
                              pVertexformat->GetStride(),
                              (void *)&(vba.Color<V4f>(0, 0)));
        glEnableVertexAttribArray(posLoc);
        glEnableVertexAttribArray(clorLoc);
        glDrawElements(drawMode, pIndexBuffer->GetNumElements(),
                       GL_UNSIGNED_INT, pIndexBuffer->GetData());
        glDisableVertexAttribArray(posLoc);
        glDisableVertexAttribArray(clorLoc);
    }
}

}  // namespace HAMO
