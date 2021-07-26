//
// Created by gaoxiang on 2020/9/16.
//

#ifndef MAPPING_OPTIMIZATION_S2_H
#define MAPPING_OPTIMIZATION_S2_H

#include "common/keyframe.h"
#include "common/num_type.h"
#include "io/yaml_io.h"
#include "pipeline/optimization_s1/optimization_params.h"
#include "pipeline/pipeline_context.h"

namespace mapping::pipeline {

struct OptimizationStage2Impl;

/**
 * Optimization on stage 2 第二轮优化
 * 使用回环约束，并且使用第一轮的GPS inlier结果
 *
 * 相比于3.0，单独把这一步拎出来了
 */
class OptimizationStage2 : public PipelineContext {
   public:
    explicit OptimizationStage2(const io::YAML_IO& yaml_file, RunMode run_mode = RunMode::PIPELINE);

    ~OptimizationStage2() override;

    /// Context 接口
    /// 初始化，成功返回 true
    bool Init() override;

    /// 开始处理，完成后返回true，无论结果是否成功
    /// 要求完成后更新context status状态，以便 engine 确定它的完成情况
    /// NOTE: 模块内容应阻塞此Start函数，Start函数返回表明已执行完毕
    bool Start() override;

    void LoadFixedKeyframesIndexes(const std::vector<int>& keyframes_indexes);

    void LoadFixedKeyframesIndex(const int keyframe_index);

    void LoadKeyframesFromQuickDebug(const std::map<IdType, common::KFPtr>& kfs);

    void SetOptimizationParams(const OptimizationParams& params);

    void SetNotSavePcd() { save_pcd_ = false; }

    void SetNotSaveDb() { save_db_ = false; }

    void SetIteration(uint8_t num);

    void UpdateDb();

    void SavePcd();

   private:
    /// 建立优化问题
    void BuildProblem();

    /// 求解问题
    void Solve();

    /// 剔除错误点
    void RemoveOutliers();

    // 添加顶点
    void AddVertex();

    // 添加各种边
    void AddGpsFactors();
    void AddMatchingFactors();
    void AddDRFactors();
    void AddLoopClosureFactors();
    void AddGlobalRotationFactors();
    void AddHeightFactors();
    void AddFixFactors();

    // 输出优化统计信息
    void CollectOptimizationStatistics();

    bool UpdateMapHeight(double_t map_height);

    io::YAML_IO yaml_file_;
    std::string local_data_path_;
    std::string local_db_path_;

    bool if_merge_maps_;

    std::unique_ptr<OptimizationStage2Impl> impl_;

    bool get_keyframes_from_quick_debug_ = false;

    bool save_pcd_ = true;
    bool save_db_ = true;
    bool reset_iteration_ = false;

    uint8_t iteration_ = 0;
};
}  // namespace mapping::pipeline

#endif  // MAPPING_OPTIMIZATION_S1_H
