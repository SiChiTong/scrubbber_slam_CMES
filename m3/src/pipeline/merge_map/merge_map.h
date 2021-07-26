//
// Created by pengguoqi on 21-1-10.
//

#ifndef MAPPING_MERGEMAP_H
#define MAPPING_MERGEMAP_H

#include "common/merge_info.h"
#include "common/num_type.h"
#include "io/db_io.h"
#include "io/yaml_io.h"
#include "pipeline/pipeline_context.h"

namespace mapping::pipeline {

/**
 * Optimization on stage 1 第一轮优化
 * 用于合成主地图和新增地图db和keyframes数据
 * 说明：此新增目前只支持乘用车，即没有贴边数据包的地图数据，若支持低速车，另外需开发
 */

class MergeMap : public PipelineContext {
   public:
    explicit MergeMap(const io::YAML_IO &yaml_file, RunMode run_mode = RunMode::PIPELINE);

    virtual ~MergeMap() override;

    /// Context 接口
    /// 初始化，成功返回 true
    virtual bool Init() override;

    /// 开始处理，完成后返回true，无论结果是否成功
    /// 要求完成后更新context status状态，以便 engine 确定它的完成情况
    /// NOTE: 模块内容应阻塞此Start函数，Start函数返回表明已执行完毕
    virtual bool Start() override;

   private:
    void CopyAndMoveDBFiles();

    /// 合并关键数据
    bool MerageKeyFrames();

    /// 合并db数据
    bool MerageDB();

    /// 清空数据
    void Clear();

   private:
    const io::YAML_IO &yaml_file_;
    std::string local_data_path_;
    std::string target_data_path_;
    std::string local_db_path_;

    std::map<IdType, std::vector<common::KFPtr>> target_keyframes_;
    std::map<IdType, std::vector<common::KFPtr>> current_keyframes_;

    common::MergeInfoVec merge_info_vec_;

    int target_keyframes_end_id_;
    int target_trajectory_end_id_;
};

}  // namespace mapping::pipeline
#endif  // MAPPING_MERGEMAP_H