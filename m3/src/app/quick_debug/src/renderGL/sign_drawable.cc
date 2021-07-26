//
// Created by xiang on 2019/12/24.
//
#include "renderGL/sign_drawable.h"
#include "map/projection_utm.h"
#include "platform/system.h"
#include "renderGL/gl_api.h"
#include "renderGL/mc_buffer.h"
#include "renderGL/mc_image.h"
#include "renderGL/mc_render_layout.h"
#include "renderGL/mc_texture2d.h"
#include "renderGL/mc_vertex_buffer_accessor.h"
#include "renderGL/mc_vertex_format.h"
#include "renderGL/render_info.h"

#include <glog/logging.h>

namespace HAMO {

SignDrawable::SignDrawable() {
    m_vFormat = 0;
    m_vBuffer = 0;
    m_vIBuffer = 0;

    m_pRenderLayout = new RenderLayout();

    m_boundBox.Set(V3f(1000.0f, 1000.0f, 1000.0f),
                   V3f(-1000.0f, -1000.0f, -1000.0f));
}

SignDrawable::~SignDrawable() {
    if (m_pRenderLayout) {
        delete m_pRenderLayout;
    }
}

void SignDrawable::Update(VertexFormat *vFormat, VertexBuffer *vBuffer,
                          IndexBuffer *vIBuffer) {
    m_pRenderLayout->SetTopologyType(RenderLayout::TT_TriangleList);
    m_pRenderLayout->BindVertexStream(vBuffer, vFormat);
    m_pRenderLayout->BindIndexStream(vIBuffer);

    m_vFormat = vFormat;
    m_vBuffer = vBuffer;
    m_vIBuffer = vIBuffer;
}

Texture2D texture;

void SignDrawable::DrawImplementation(RenderInfo &renderInfo) {
    Program *pProgram = renderInfo.program_;

    int posLoc = pProgram->attrib_locs_[0];
    int clorLoc = pProgram->attrib_locs_[1];

    UniformParam *param = pProgram->GetUniformParam();
    param->SetParameter(2, m_stateParam);
    pProgram->ActiveUniform();
    if (m_vFormat != NULL && m_vBuffer != NULL && m_vIBuffer != NULL) {
        if (!texture.IsValidate()) {
            Image image;

            std::string path;
            System::GetAppPath(path);
            path += "/Config/signboard.png";
            if (image.LoadImage(path.c_str())) {
                texture.Create(image);
            } else {
                LOG(ERROR) << "failed to load signboard image";
            }
        }

        glBindTexture(GL_TEXTURE_2D, texture.GetHandle());

        // set the texture wrapping parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                        GL_LINEAR_MIPMAP_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                        GL_LINEAR_MIPMAP_NEAREST);

        VertexBufferAccessor vba(m_vFormat, m_vBuffer);

        float *points = (float *)&(vba.Position<V3f>(0));
        float *ctx = (float *)&(vba.TCoord<V3f>(0, 0));

        glVertexAttribPointer(posLoc, 3, GL_FLOAT, false,
                              m_vFormat->GetStride(),
                              (void *)&(vba.Position<V3f>(0)));
        glVertexAttribPointer(clorLoc, 2, GL_FLOAT, false,
                              m_vFormat->GetStride(),
                              (void *)&(vba.TCoord<V2f>(0, 0)));

        glEnableVertexAttribArray(posLoc);
        glEnableVertexAttribArray(clorLoc);

        glDrawElements(GL_TRIANGLES, m_vIBuffer->GetNumElements(),
                       GL_UNSIGNED_INT, m_vIBuffer->GetData());
        glDisableVertexAttribArray(posLoc);
        glDisableVertexAttribArray(clorLoc);
    }
}

}  // namespace HAMO
