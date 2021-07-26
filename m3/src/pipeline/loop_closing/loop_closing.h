//
// Created by gaoxiang on 2020/9/22.
//

#ifndef MAPPING_LOOP_CLOSING_H
#define MAPPING_LOOP_CLOSING_H

#include "common/num_type.h"
#include "io/yaml_io.h"
#include "pipeline/pipeline_context.h"

namespace mapping::pipeline {

struct LoopClosingImpl;

/**
 * Optimization on stage 1 第一轮优化
 * 不带回环约束，使用GPS、DR、Matching计算粗略的位姿
 *
 * 相比于3.0，对不同gnss精度使用不同的噪声
 * 并且多了一步找优化初值的计算过程
 */
class LoopClosing : public PipelineContext {
   public:
    explicit LoopClosing(const io::YAML_IO &yaml_file, RunMode run_mode = RunMode::PIPELINE, bool only_detect = false);

    ~LoopClosing() override;

    void SetStartEndIdx(IdType start_id, IdType end_id);

    /// Context 接口
    /// 初始化，成功返回 true
    bool Init() override;

    /// 开始处理，完成后返回true，无论结果是否成功
    /// 要求完成后更新context status状态，以便 engine 确定它的完成情况
    /// NOTE: 模块内容应阻塞此Start函数，Start函数返回表明已执行完毕
    bool Start() override;

    /// 检测可能存在回环的约束
    void DetectLoopCandidates();

    /// 计算候选帧之间的相对运动
    void ComputeRelativeMotionMT();

    /**
     * 存储回环信息
     * @param save_all_in_one 是否把所有回环存储在一个文件中
     */
    void SaveLoopCandidates(bool save_all_in_one = true);

    /// 无GPS状态下，第一轮轨迹之间的顺序未知，检测首尾连接性
    void DetectByContinuous();

    /// 清空数据
    void Clear();

   private:
    /// 根据第一轮优化轨迹计算回环检测
    void DetectByStage1();

    io::YAML_IO yaml_file_;
    std::string local_data_path_;
    std::string local_db_path_;

    bool if_merge_maps_;

    std::unique_ptr<LoopClosingImpl> impl_;
};
}  // namespace mapping::pipeline

#endif  // MAPPING_LOOP_CLOSING_H
