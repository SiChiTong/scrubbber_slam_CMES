//
// Created by gaoxiang on 2019/12/17.
//

#ifndef HAMO_LINUX_MC_PROGRAM_H
#define HAMO_LINUX_MC_PROGRAM_H

#include <map>
#include "renderGL/mc_uniform_param.h"

namespace HAMO {

class VertexShader;

class FragmentShader;

struct ConstantDefinition {
    char name[32];
    UniformParam::UNIFORM_TYPE type;
};

class Program {
   public:
    Program();

    virtual ~Program();

   public:
    bool CreateProgram(VertexShader *vsShader, FragmentShader *psShader);

    void BindUniformLocation(ConstantDefinition *argment, int nCount);

    UniformParam *GetUniformParam() { return &param_; }

    bool IsValidate() { return (handle_ != 0); }

    unsigned int GetHandle() const { return handle_; }

    void ActiveUniform();

    void UseProgram();

    void DeleteProgram();

   private:
    bool CheckStatus(unsigned int handle);

   private:
    unsigned int handle_;

    UniformParam param_;

   public:
    std::map<int, int> attrib_locs_;
};

}  // namespace HAMO

#endif  // HAMO_LINUX_MC_PROGRAM_H
