//
// Created by gaoxiang on 2019/12/17.
//

#include "renderGL/mc_technique_manager.h"
#include "platform/system.h"
#include "renderGL/gl_api.h"
#include "renderGL/mc_program.h"
#include "renderGL/mc_render_pass.h"
#include "renderGL/mc_render_state_object.h"
#include "renderGL/mc_render_technique.h"
#include "renderGL/mc_shader.h"

#include <glog/logging.h>
#include <cassert>

namespace HAMO {

TechniqueManager::TechniqueManager() {}

TechniqueManager::~TechniqueManager() {}

TechniqueManager *TechniqueManager::GetInstance() {
    static TechniqueManager mgr;
    return &mgr;
}

RenderPass *CreateCommonProgram(const char *szDir) {
    char vsfile[256];
    char psfile[256];

    sprintf(vsfile, "%s/Config/common.vs", szDir);
    sprintf(psfile, "%s/Config/common.ps", szDir);

    VertexShader *vsShader = new VertexShader;
    if (!vsShader->LoadShaderFile(vsfile)) {
        LOG(ERROR) << "load v shader failed";
        return nullptr;
    }

    FragmentShader *psShader = new FragmentShader;
    if (!psShader->LoadShaderFile(psfile)) {
        LOG(ERROR) << "load f shader failed";
        return nullptr;
    }

    Program *program = new Program();
    if (!program->CreateProgram(vsShader, psShader)) {
        return nullptr;
    }

    int posLoc = glGetAttribLocation(program->GetHandle(), "vPosition");
    int clorLoc = glGetAttribLocation(program->GetHandle(), "vColor");

    program->attrib_locs_.emplace(0, posLoc);
    program->attrib_locs_.emplace(1, clorLoc);

    ConstantDefinition argment[3] = {
        {"matModel", UniformParam::UNV_FLOAT_MAT4},
        {"matProject", UniformParam::UNV_FLOAT_MAT4}};

    program->BindUniformLocation(argment, 2);

    RenderPass *pRenderPass = new RenderPass();
    pRenderPass->SetShaderProgram(program);

    RasterizerStateDesc rs_desc;
    DepthStencilStateDesc dss_desc;

    BlendStateDesc bs_desc;

    dss_desc.depth_enable = false;
    RenderStateObject *pRenderStateObj =
        new RenderStateObject(rs_desc, dss_desc, bs_desc);
    pRenderPass->SetRenderStateObject(pRenderStateObj);

    return pRenderPass;
}

RenderPass *CreateMapBoxProgram(const char *szDir) {
    char vsfile[256];
    char psfile[256];

    sprintf(vsfile, "%s/Config/thinLine.vs", szDir);
    sprintf(psfile, "%s/Config/thinLine.ps", szDir);

    auto *vsShader = new VertexShader;
    if (!vsShader->LoadShaderFile(vsfile)) {
        return nullptr;
    }

    auto *psShader = new FragmentShader;
    if (!psShader->LoadShaderFile(psfile)) {
        return nullptr;
    }

    auto *program = new Program();
    if (!program->CreateProgram(vsShader, psShader)) {
        return nullptr;
    }

    int posLoc = glGetAttribLocation(program->GetHandle(), "vPosition");
    int clorLoc = glGetAttribLocation(program->GetHandle(), "vColor");

    program->attrib_locs_.emplace(0, posLoc);
    program->attrib_locs_.emplace(1, clorLoc);

    ConstantDefinition argment[3] = {
        {"matModel", UniformParam::UNV_FLOAT_MAT4},
        {"matProject", UniformParam::UNV_FLOAT_MAT4},
        {"uWorld", UniformParam::UNV_FLOAT_VEC2}};

    program->BindUniformLocation(argment, 3);

    //������Ⱦͨ��
    RenderPass *pRenderPass = new RenderPass();
    pRenderPass->SetShaderProgram(program);

    RasterizerStateDesc rs_desc;
    DepthStencilStateDesc dss_desc;

    BlendStateDesc bs_desc;

    bs_desc.blend_enable = true;
    bs_desc.src_blend = static_cast<AlphaBlendFactor>(GL_SRC_ALPHA);
    bs_desc.dest_blend = static_cast<AlphaBlendFactor>(GL_ONE_MINUS_SRC_ALPHA);
    bs_desc.dest_blend_alpha = static_cast<AlphaBlendFactor>(GL_ONE);
    bs_desc.src_blend_alpha = static_cast<AlphaBlendFactor>(GL_ZERO);

    bs_desc.blend_op_alpha = static_cast<BlendOperation>(GL_FUNC_ADD);
    bs_desc.blend_op = static_cast<BlendOperation>(GL_FUNC_ADD);

    dss_desc.depth_enable = false;
    RenderStateObject *pRenderStateObj =
        new RenderStateObject(rs_desc, dss_desc, bs_desc);
    pRenderPass->SetRenderStateObject(pRenderStateObj);

    return pRenderPass;
}

RenderPass *CreateLineProgram(const char *szDir) {
    char vsfile[256];
    char psfile[256];

    sprintf(vsfile, "%s/Config/line.vs", szDir);
    sprintf(psfile, "%s/Config/line.ps", szDir);

    VertexShader *vsShader = new VertexShader;
    if (!vsShader->LoadShaderFile(vsfile)) {
        return NULL;
    }

    FragmentShader *psShader = new FragmentShader;
    if (!psShader->LoadShaderFile(psfile)) {
        return NULL;
    }

    Program *program = new Program();
    if (!program->CreateProgram(vsShader, psShader)) {
        return nullptr;
    }

    int posLoc = glGetAttribLocation(program->GetHandle(), "vPosition");
    int clorLoc = glGetAttribLocation(program->GetHandle(), "vColor");
    int normalLoc = glGetAttribLocation(program->GetHandle(), "vNomral");

    program->attrib_locs_.emplace(0, posLoc);
    program->attrib_locs_.emplace(1, clorLoc);
    program->attrib_locs_.emplace(2, normalLoc);

    ConstantDefinition argment[3] = {
        {"matModel", UniformParam::UNV_FLOAT_MAT4},
        {"matProject", UniformParam::UNV_FLOAT_MAT4},
        {"vecParam", UniformParam::UNV_FLOAT_VEC3}};

    program->BindUniformLocation(argment, 3);

    RenderPass *pRenderPass = new RenderPass();
    pRenderPass->SetShaderProgram(program);

    RasterizerStateDesc rs_desc;
    DepthStencilStateDesc dss_desc;

    BlendStateDesc bs_desc;

    bs_desc.blend_enable = true;
    bs_desc.src_blend = static_cast<AlphaBlendFactor>(GL_SRC_ALPHA);
    bs_desc.dest_blend = static_cast<AlphaBlendFactor>(GL_ONE_MINUS_SRC_ALPHA);
    bs_desc.dest_blend_alpha = static_cast<AlphaBlendFactor>(GL_ONE);
    bs_desc.src_blend_alpha = static_cast<AlphaBlendFactor>(GL_ZERO);
    bs_desc.blend_op_alpha = static_cast<BlendOperation>(GL_FUNC_ADD);
    bs_desc.blend_op = static_cast<BlendOperation>(GL_FUNC_ADD);
    dss_desc.depth_enable = false;

    RenderStateObject *pRenderStateObj =
        new RenderStateObject(rs_desc, dss_desc, bs_desc);
    pRenderPass->SetRenderStateObject(pRenderStateObj);

    return pRenderPass;
}

RenderPass *CreatePointCloudProgram(const char *szDir) {
    char vsfile[256];
    char psfile[256];

    sprintf(vsfile, "%s/Config/pointCloud.vs", szDir);
    sprintf(psfile, "%s/Config/pointCloud.ps", szDir);

    VertexShader *vsShader = new VertexShader;
    if (!vsShader->LoadShaderFile(vsfile)) {
        LOG(ERROR) << "Load vshader failed at " << vsfile;
        return NULL;
    }

    FragmentShader *psShader = new FragmentShader;
    if (!psShader->LoadShaderFile(psfile)) {
        LOG(ERROR) << "Load fshader failed at " << vsfile;
        return NULL;
    }

    Program *program = new Program();
    if (!program->CreateProgram(vsShader, psShader)) {
        LOG(INFO) << "failed to create program.";
        return nullptr;
    }

    int posLoc = glGetAttribLocation(program->GetHandle(), "vPosition");
    int clorLoc = glGetAttribLocation(program->GetHandle(), "vColor");
    program->attrib_locs_.emplace(0, posLoc);
    program->attrib_locs_.emplace(1, clorLoc);

    ConstantDefinition argment[4] = {
        {"matModel", UniformParam::UNV_FLOAT_MAT4},
        {"matProject", UniformParam::UNV_FLOAT_MAT4},
        {"vHighLimit", UniformParam::UNV_FLOAT_VEC3},
        {"vLimitIntensity", UniformParam::UNV_FLOAT_VEC3}};

    program->BindUniformLocation(argment, 4);

    RenderPass *pRenderPass = new RenderPass();
    pRenderPass->SetShaderProgram(program);

    RasterizerStateDesc rs_desc;
    DepthStencilStateDesc dss_desc;
    BlendStateDesc bs_desc;

    RenderStateObject *pRenderStateObj =
        new RenderStateObject(rs_desc, dss_desc, bs_desc);
    pRenderPass->SetRenderStateObject(pRenderStateObj);

    return pRenderPass;
}

RenderPass *CreateTextureModelProgram(const char *szDir) {
    char vsfile[256];
    char psfile[256];

    sprintf(vsfile, "%s/Config/textureModel.vs", szDir);
    sprintf(psfile, "%s/Config/textureModel.ps", szDir);

    VertexShader *vsShader = new VertexShader;
    if (!vsShader->LoadShaderFile(vsfile)) {
        return NULL;
    }

    FragmentShader *psShader = new FragmentShader;
    if (!psShader->LoadShaderFile(psfile)) {
        return NULL;
    }

    Program *program = new Program();
    if (!program->CreateProgram(vsShader, psShader)) {
        return nullptr;
    }

    int posLoc = glGetAttribLocation(program->GetHandle(), "vPosition");
    int texLoc = glGetAttribLocation(program->GetHandle(), "texcoord");
    int colLoc = glGetAttribLocation(program->GetHandle(), "scolor");
    program->attrib_locs_.emplace(0, posLoc);
    program->attrib_locs_.emplace(1, texLoc);
    program->attrib_locs_.emplace(2, colLoc);

    ConstantDefinition argment[4] = {
        {"matModel", UniformParam::UNV_FLOAT_MAT4},
        {"matProject", UniformParam::UNV_FLOAT_MAT4},
        {"vecParam", UniformParam::UNV_FLOAT_VEC3},
        {"texSampler", UniformParam::UNV_FLOAT_VEC3}};

    program->BindUniformLocation(argment, 4);

    RenderPass *pRenderPass = new RenderPass();
    pRenderPass->SetShaderProgram(program);

    RasterizerStateDesc rs_desc;
    DepthStencilStateDesc dss_desc;
    BlendStateDesc bs_desc;

    // Ci = (Cs * S) + (Cd * D)

    bs_desc.blend_enable = true;
    bs_desc.src_blend = static_cast<AlphaBlendFactor>(GL_SRC_ALPHA);
    bs_desc.dest_blend = static_cast<AlphaBlendFactor>(GL_ONE_MINUS_SRC_ALPHA);
    bs_desc.dest_blend_alpha = static_cast<AlphaBlendFactor>(GL_ONE);
    bs_desc.src_blend_alpha = static_cast<AlphaBlendFactor>(GL_ZERO);
    bs_desc.blend_op_alpha = static_cast<BlendOperation>(GL_FUNC_ADD);
    bs_desc.blend_op = static_cast<BlendOperation>(GL_FUNC_ADD);

    RenderStateObject *pRenderStateObj =
        new RenderStateObject(rs_desc, dss_desc, bs_desc);
    pRenderPass->SetRenderStateObject(pRenderStateObj);

    return pRenderPass;
}

void TechniqueManager::Initialize() {
    std::string path;
    if (System::GetAppPath(path) > 0) {
        RenderPass *renderPass[5];
        std::string buf;
        renderPass[0] = CreatePointCloudProgram(path.c_str());
        renderPass[1] = CreateCommonProgram(path.c_str());
        renderPass[2] = CreateTextureModelProgram(path.c_str());
        renderPass[3] = CreateLineProgram(path.c_str());
        renderPass[4] = CreateMapBoxProgram(path.c_str());

        for (int i = 0; i < 5; i++) {
            RenderTechnique *pRenderTechnique = new RenderTechnique();
            pRenderTechnique->AddPass(renderPass[i]);
            AddTechnique(i, pRenderTechnique);
        }
    } else {
        LOG(ERROR) << "cannot get app path";
    }
}

void TechniqueManager::AddTechnique(int dataType,
                                    RenderTechnique *renderTechnique) {
    render_techniques_.push_back(renderTechnique);
}

RenderTechnique *TechniqueManager::GetTechnique(int type) {
    assert(type >= 0 && size_t(type) < render_techniques_.size());

    return render_techniques_[type];
}

}  // namespace HAMO
