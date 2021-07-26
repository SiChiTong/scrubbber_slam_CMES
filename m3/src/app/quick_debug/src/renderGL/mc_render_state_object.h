//
// Created by gaoxiang on 2019/12/17.
//

#ifndef HAMO_LINUX_MC_RENDER_STATE_OBJECT_H
#define HAMO_LINUX_MC_RENDER_STATE_OBJECT_H

#include "renderGL/mc_render_state_desc.h"

namespace HAMO {

class RenderStateObject {
public:
    RenderStateObject(const RasterizerStateDesc &rs_desc, const DepthStencilStateDesc &dss_desc,
                      const BlendStateDesc &bs_desc);

    virtual ~RenderStateObject();

    RasterizerStateDesc GetRasterizerStateDesc() const;

    DepthStencilStateDesc GetDepthStencilStateDesc() const;

    BlendStateDesc GetBlendStateDesc() const;

    void ActiveRenderState();

protected:
    RasterizerStateDesc rs_desc_;
    DepthStencilStateDesc dss_desc_;
    BlendStateDesc bs_desc_;

};

class SamplerStateObject {
public:
    SamplerStateObject(SamplerStateDesc const &desc);

    virtual ~SamplerStateObject();

    SamplerStateDesc GetDesc() const;

    void ActiveSamplerState();

protected:
    SamplerStateDesc desc_;
};
}

#endif //HAMO_LINUX_MC_RENDER_STATE_OBJECT_H
