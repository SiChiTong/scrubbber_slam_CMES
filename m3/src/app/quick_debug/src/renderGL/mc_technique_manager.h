//
// Created by gaoxiang on 2019/12/17.
//

#ifndef HAMO_LINUX_MC_TECHNIQUE_MANAGER_H
#define HAMO_LINUX_MC_TECHNIQUE_MANAGER_H

#include <vector>

namespace HAMO {

class RenderTechnique;

/**
 * 渲染器管理，单例模式
 *
 * 通过 GetTechnique函数获取对应的渲染器
 *
 * 索引：
 * 0-点云
 * 1-common
 * 2-texture
 * 3-line
 * 4-map box
 */
class TechniqueManager {
public:
    TechniqueManager();

    ~TechniqueManager();

public:
    static TechniqueManager *GetInstance();

    void Initialize();

    void AddTechnique(int dataType, RenderTechnique *renderTechnique);

    RenderTechnique *GetTechnique(int type);

private:
    std::vector<RenderTechnique *> render_techniques_;
};

}
#endif //HAMO_LINUX_MC_TECHNIQUE_MANAGER_H
