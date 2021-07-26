//
// Created by gaoxiang on 2019/12/17.
//

#ifndef HAMO_LINUX_MC_RENDER_STATE_DESC_H
#define HAMO_LINUX_MC_RENDER_STATE_DESC_H

//====================================

namespace HAMO {
enum ShadeMode {
    SM_Flat,
    SM_Gouraud
};

enum PolygonMode {
    PM_Point,
    PM_Line,
    PM_Fill
};

enum CullMode {
    CM_None,
    CM_Front,
    CM_Back
};

//===================================================

enum CompareFunction {
    CF_AlwaysFail,
    CF_AlwaysPass,
    CF_Less,
    CF_LessEqual,
    CF_Equal,
    CF_NotEqual,
    CF_GreaterEqual,
    CF_Greater
};

enum StencilOperation {
    // Leave the stencil buffer unchanged
            SOP_Keep,
    // Set the stencil value to zero
            SOP_Zero,
    // Set the stencil value to the reference value
            SOP_Replace,
    // Increase the stencil value by 1, clamping at the maximum value
            SOP_Incr,
    // Decrease the stencil value by 1, clamping at 0
            SOP_Decr,
    // Invert the bits of the stencil buffer
            SOP_Invert,
    // Increase the stencil value by 1, wrap the result if necessary
            SOP_Incr_Wrap,
    // Decrease the stencil value by 1, wrap the result if necessary
            SOP_Decr_Wrap
};

//=================================================
enum AlphaBlendFactor {
    ABF_Zero,
    ABF_One,
    ABF_Src_Alpha,
    ABF_Dst_Alpha,
    ABF_Inv_Src_Alpha,
    ABF_Inv_Dst_Alpha,
    ABF_Src_Color,
    ABF_Dst_Color,
    ABF_Inv_Src_Color,
    ABF_Inv_Dst_Color,
    ABF_Src_Alpha_Sat,
    ABF_Blend_Factor,
    ABF_Inv_Blend_Factor,
    ABF_Src1_Alpha,
    ABF_Inv_Src1_Alpha,
    ABF_Src1_Color,
    ABF_Inv_Src1_Color
};

enum BlendOperation {
    BOP_Add,
    BOP_Sub,
    BOP_Rev_Sub,
    BOP_Min,
    BOP_Max,
};

enum LogicOperation {
    LOP_Clear,
    LOP_Set,
    LOP_Copy,
    LOP_CopyInverted,
    LOP_Noop,
    LOP_Invert,
    LOP_And,
    LOP_NAnd,
    LOP_Or,
    LOP_NOR,
    LOP_XOR,
    LOP_Equiv,
    LOP_AndReverse,
    LOP_AndInverted,
    LOP_OrReverse,
    LOP_OrInverted
};
//================================================

// Sampler addressing modes - default is TAM_Wrap.
enum TexAddressingMode {
    // Texture wraps at values over 1.0
            TAM_Wrap,
    // Texture mirrors (flips) at joins over 1.0
            TAM_Mirror,
    // Texture clamps at 1.0
            TAM_Clamp,
    // Texture coordinates outside the range [0.0, 1.0] are set to the border color.
            TAM_Border
};

enum TexFilterOp {
    // Dont' use these enum directly
            TFOE_Mip_Point = 0x0,
    TFOE_Mip_Linear = 0x1,

    TFOE_Mag_Point = 0x0,
    TFOE_Mag_Linear = 0x2,

    TFOE_Min_Point = 0x0,
    TFOE_Min_Linear = 0x4,

    TFOE_Anisotropic = 0x08,

    // Use these
            TFO_Min_Mag_Mip_Point = TFOE_Min_Point | TFOE_Mag_Point | TFOE_Mip_Point,
    TFO_Min_Mag_Point_Mip_Linear = TFOE_Min_Point | TFOE_Mag_Point | TFOE_Mip_Linear,
    TFO_Min_Point_Mag_Linear_Mip_Point = TFOE_Min_Point | TFOE_Mag_Linear | TFOE_Mip_Point,
    TFO_Min_Point_Mag_Mip_Linear = TFOE_Min_Point | TFOE_Mag_Linear | TFOE_Mip_Linear,
    TFO_Min_Linear_Mag_Mip_Point = TFOE_Min_Linear | TFOE_Mag_Point | TFOE_Mip_Point,
    TFO_Min_Linear_Mag_Point_Mip_Linear = TFOE_Min_Linear | TFOE_Mag_Point | TFOE_Mip_Linear,
    TFO_Min_Mag_Linear_Mip_Point = TFOE_Min_Linear | TFOE_Mag_Linear | TFOE_Mip_Point,
    TFO_Min_Mag_Mip_Linear = TFOE_Min_Linear | TFOE_Mag_Linear | TFOE_Mip_Linear,
    TFO_Anisotropic = TFOE_Anisotropic,

};

//==============================================

struct RasterizerStateDesc {
    PolygonMode polygon_mode;
    ShadeMode shade_mode;

    CullMode cull_mode;
    bool front_face_ccw;

    float polygon_offset_factor;
    float polygon_offset_units;

    bool depth_clip_enable;
    bool scissor_enable;
    bool multisample_enable;

public:
    RasterizerStateDesc();
};

struct DepthStencilStateDesc {
    bool depth_enable;
    bool depth_write_mask;
    CompareFunction depth_func;

    bool front_stencil_enable;
    CompareFunction front_stencil_func;
    unsigned short front_stencil_ref;
    unsigned short front_stencil_read_mask;
    unsigned short front_stencil_write_mask;
    StencilOperation front_stencil_fail;
    StencilOperation front_stencil_depth_fail;
    StencilOperation front_stencil_pass;

    bool back_stencil_enable;
    CompareFunction back_stencil_func;
    unsigned short back_stencil_ref;
    unsigned short back_stencil_read_mask;
    unsigned short back_stencil_write_mask;
    StencilOperation back_stencil_fail;
    StencilOperation back_stencil_depth_fail;
    StencilOperation back_stencil_pass;

public:
    DepthStencilStateDesc();
};

struct BlendStateDesc {
    bool blend_enable;

    AlphaBlendFactor src_blend;
    AlphaBlendFactor dest_blend;
    AlphaBlendFactor src_blend_alpha;
    AlphaBlendFactor dest_blend_alpha;

    BlendOperation blend_op_alpha;
    BlendOperation blend_op;

    bool logic_op_enable;
    LogicOperation logic_op;

public:
    BlendStateDesc();

};

struct SamplerStateDesc {
    int borderColor;

    TexFilterOp filter;

    TexAddressingMode addr_mode_u;
    TexAddressingMode addr_mode_v;
    TexAddressingMode addr_mode_w;

    CompareFunction cmp_func;

    unsigned int max_anisotropy;

    float min_lod;
    float max_lod;
    float mip_map_lod_bias;

public:
    SamplerStateDesc();

};

}
#endif //HAMO_LINUX_MC_RENDER_STATE_DESC_H
