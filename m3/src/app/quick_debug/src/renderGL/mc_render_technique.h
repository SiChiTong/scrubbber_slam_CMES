//
// Created by gaoxiang on 2019/12/17.
//

#ifndef HAMO_LINUX_MC_RENDER_TECHNIQUE_H
#define HAMO_LINUX_MC_RENDER_TECHNIQUE_H

#include <vector>

namespace HAMO {

class RenderPass;

class RenderTechnique {
public:
    RenderTechnique();

    ~RenderTechnique();

    void AddPass(RenderPass *pRenderPass);

    int GetPassCount() const;

    RenderPass *const &GetPass(int nIndex) const;

private:
    std::vector<RenderPass *> m_rendPasses;

};

}

#endif //HAMO_LINUX_MC_RENDER_TECHNIQUE_H
