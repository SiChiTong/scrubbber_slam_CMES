//
// Created by gaoxiang on 2019/12/19.
//

#include "renderGL/render_leaf.h"

#include "renderGL/mc_program.h"
#include "renderGL/mc_render_pass.h"
#include "renderGL/mc_render_technique.h"
#include "renderGL/mc_shader.h"
#include "renderGL/mc_uniform_param.h"

#include <cstdio>

namespace HAMO {

RenderLeaf::RenderLeaf() : m_drawable(NULL) {}

void RenderLeaf::SetDrawable(Drawable *drawable) {
    if (m_drawable) {
        delete m_drawable;
    }
    m_drawable = drawable;
}

Drawable *RenderLeaf::GetDrawable() { return m_drawable; }

void RenderLeaf::SetViewport(const Viewport &view) { m_viewport = view; }

void RenderLeaf::SetModelViewMatrix(const Matrix4x4f &modelview) { m_modelview = modelview; }

void RenderLeaf::SetProjectionMatrix(const Matrix4x4f &projection) { m_projection = projection; }

void RenderLeaf::SetRenderTechnique(RenderTechnique *technique) { m_technique = technique; }

bool RenderLeaf::Render(RenderInfo &rendinfo, RenderLeaf *previous) const {
    State *pState = rendinfo.GetState();

    if (NULL != m_technique && NULL != m_drawable && m_drawable->IsValid()) {
        int nPassCount = m_technique->GetPassCount();
        for (int i = 0; i < nPassCount; i++) {
            RenderPass *pRenderPass = m_technique->GetPass(i);
            pRenderPass->Bind();
            Program *pProgram = pRenderPass->GetShaderProgram();
            UniformParam *param = pProgram->GetUniformParam();
            rendinfo.program_ = pProgram;
            param->SetParameter(0, m_modelview);
            param->SetParameter(1, m_projection);
            if (param->GetUniformNumber() > 2) {
                //一类drawble使用，放在这里不合适，移动到line_drawble
                param->SetParameter(2, V2f(m_viewport.Width(), m_viewport.Height()));
            }
            pProgram->ActiveUniform();
            m_drawable->Draw(rendinfo);
        }
        return true;
    } else {
        return false;
    }
}

RenderLeaf::~RenderLeaf() {
    if (m_drawable != nullptr) {
        delete m_drawable;
        m_drawable = nullptr;
    }
}

}  // namespace HAMO
