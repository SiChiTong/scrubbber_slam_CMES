//
// Created by gaoxiang on 2019/12/17.
//

#include "renderGL/mc_render_state_object.h"
#include "renderGL/gl_api.h"

namespace HAMO {

RenderStateObject::RenderStateObject(const RasterizerStateDesc &rs_desc, const DepthStencilStateDesc &dss_desc,
                                     const BlendStateDesc &bs_desc)
        : rs_desc_(rs_desc), dss_desc_(dss_desc), bs_desc_(bs_desc) {
}

RenderStateObject::~RenderStateObject() {
}

RasterizerStateDesc RenderStateObject::GetRasterizerStateDesc() const {
    return rs_desc_;
}

DepthStencilStateDesc RenderStateObject::GetDepthStencilStateDesc() const {
    return dss_desc_;
}

BlendStateDesc RenderStateObject::GetBlendStateDesc() const {
    return bs_desc_;
}

void RenderStateObject::ActiveRenderState() {

    //RasterizerStateDesc
    glShadeModel(rs_desc_.shade_mode);

    if (rs_desc_.multisample_enable) {
        glEnable(GL_MULTISAMPLE);
    } else {
        glDisable(GL_MULTISAMPLE);
    }

    glFrontFace(rs_desc_.front_face_ccw);

    if (rs_desc_.depth_clip_enable) {
        glDisable(GL_DEPTH_CLAMP);
    } else {
        glEnable(GL_DEPTH_CLAMP);
    }

    switch (rs_desc_.cull_mode) {
        case CM_None:
            glDisable(GL_CULL_FACE);
            break;

        case CM_Front:
            glEnable(GL_CULL_FACE);
            glCullFace(GL_FRONT);
            break;

        case CM_Back:
            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);
            break;
    }

    if (rs_desc_.scissor_enable) {
        glEnable(GL_SCISSOR_TEST);
    } else {
        glDisable(GL_SCISSOR_TEST);
    }

    //--------------------------------------------
    //DepthStencilStateDesc
    if (dss_desc_.depth_enable) {
        glEnable(GL_DEPTH_TEST);
    } else {
        glDisable(GL_DEPTH_TEST);
    }

    //-----------------------------------------------------------
    //BlendStateDesc
    if (bs_desc_.blend_enable) {
        glEnable(GL_BLEND);
    } else {
        glDisable(GL_BLEND);
    }
    glBlendFuncSeparate(bs_desc_.src_blend, bs_desc_.dest_blend, bs_desc_.src_blend_alpha,
                        bs_desc_.dest_blend_alpha);

    glBlendEquationSeparate(bs_desc_.blend_op, bs_desc_.blend_op_alpha);

    glLogicOp(bs_desc_.logic_op);
}

//========================================================


SamplerStateObject::SamplerStateObject(SamplerStateDesc const &desc)
        : desc_(desc) {

}

SamplerStateObject::~SamplerStateObject() {

}

SamplerStateDesc SamplerStateObject::GetDesc() const {
    return desc_;
}

void SamplerStateObject::ActiveSamplerState() {

}
}
