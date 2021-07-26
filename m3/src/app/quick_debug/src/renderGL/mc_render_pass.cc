//
// Created by gaoxiang on 2019/12/17.
//
#include "renderGL/mc_render_pass.h"
#include "renderGL/mc_program.h"
#include "renderGL/gl_api.h"
#include "renderGL/mc_render_state_object.h"

#include <stdio.h>

namespace HAMO {

RenderPass::RenderPass()
        : render_state_obj_(NULL), shader_program_(NULL) {

}

RenderPass::~RenderPass() {

}

RenderStateObject *const &RenderPass::GetRenderStateObject() const {
    return render_state_obj_;
}

void RenderPass::SetRenderStateObject(RenderStateObject *const renderStateObj) {
    render_state_obj_ = renderStateObj;
}

void RenderPass::Bind() const {

    glUseProgram(shader_program_->GetHandle());
    render_state_obj_->ActiveRenderState();
}

void RenderPass::Unbind() const {

}

void RenderPass::SetShaderProgram(Program *shader) {
    shader_program_ = shader;
}

Program *RenderPass::GetShaderProgram() const {
    return shader_program_;
}

}