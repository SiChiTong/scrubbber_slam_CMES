//
// Created by gaoxiang on 2019/12/17.
//

#include "renderGL/mc_render_state_desc.h"
#include "renderGL/gl_api.h"

namespace HAMO {

RasterizerStateDesc::RasterizerStateDesc() {
    polygon_mode = static_cast<PolygonMode>(GL_SMOOTH);
    shade_mode = static_cast<ShadeMode>(GL_SMOOTH);

    cull_mode = CM_None;
    front_face_ccw = false;

    polygon_offset_factor = 1.0f;
    polygon_offset_units = 1.0f;

    multisample_enable = true;
    depth_clip_enable = false;
    scissor_enable = false;
}


//---------------------------------------------------------

DepthStencilStateDesc::DepthStencilStateDesc() {
    depth_enable = true;

}


//---------------------------------------------------------

BlendStateDesc::BlendStateDesc() {
    blend_enable = true;

    src_blend = static_cast<AlphaBlendFactor>(GL_ONE);
    dest_blend = static_cast<AlphaBlendFactor>(GL_ZERO);
    src_blend_alpha = static_cast<AlphaBlendFactor>(GL_ONE);
    dest_blend_alpha = static_cast<AlphaBlendFactor>(GL_ZERO);

    blend_op_alpha = static_cast<BlendOperation>(GL_FUNC_ADD);
    blend_op = static_cast<BlendOperation>(GL_FUNC_ADD);
}

//-------------------------------------------------------------


SamplerStateDesc::SamplerStateDesc() {
    borderColor = 0;

    filter = TFO_Min_Mag_Mip_Point;

    addr_mode_u = TAM_Wrap;
    addr_mode_v = TAM_Wrap;
    addr_mode_w = TAM_Wrap;

    cmp_func = CF_AlwaysFail;

    max_anisotropy = 16;

    min_lod;
    max_lod;
    mip_map_lod_bias = 0.0f;

    //----------------------------
}
}
