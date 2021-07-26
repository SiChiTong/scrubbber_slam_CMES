//
// Created by gaoxiang on 2020/10/22.
//

#ifndef MAPPING_VALIDATION_H
#define MAPPING_VALIDATION_H

#include "common/keyframe.h"
#include "common/num_type.h"
#include "io/yaml_io.h"
#include "pipeline/pipeline_context.h"

namespace mapping::pipeline {

struct ValidationImpl;

/// 验证环节
class Validation : public PipelineContext {
   public:
    explicit Validation(const io::YAML_IO &yaml_file);

    ~Validation() override;

    /// Context 接口
    /// 初始化，成功返回 true
    bool Init() override;

    /// 开始处理，完成后返回true，无论结果是否成功
    /// 要求完成后更新context status状态，以便 engine 确定它的完成情况
    /// NOTE: 模块内容应阻塞此Start函数，Start函数返回表明已执行完毕
    bool Start() override;

   private:
    /// 将建图相关的pose放入问题
    void BuildProblem();

    void AddDRFactors();

    void AddMatchingFactors();

    /// 添加高度约束
    void AddHeightFactors();

    /// 建立建图轨迹的kD树
    void BuildMappingKDTree();

    /// 求解问题
    void Solve();

    /* 对GPS良好的区域，对地图进行匹配
     * @return 每条轨迹上的良好的匹配数量
     */
    std::vector<int> CheckKfsWithGoodGPS();

    /// 利用GPS平滑的结果进行搜索
    /// 可以指定轨迹ID，为-1时，指定所有轨迹
    static constexpr int search_all_ = -1;
    int CheckWithSmoothingResults(int traj_id);

    /// 使用广域增量搜索
    int CheckWithExhaustiveSearch(int traj_id);

    /// 保存结果
    void SaveResults();

    io::YAML_IO yaml_file_;
    std::string local_data_path_;
    std::string local_db_path_;

    std::unique_ptr<ValidationImpl> impl_;
};

}  // namespace mapping::pipeline

#endif  // MAPPING_VALIDATION_H
