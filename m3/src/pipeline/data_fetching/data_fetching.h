//
// Created by gaoxiang on 19-7-12.
//

#ifndef MAPPING_DATAFETCHING_H
#define MAPPING_DATAFETCHING_H

#include "io/yaml_io.h"
#include "pipeline/pipeline_context.h"

namespace mapping::pipeline {

/// 数据拉取
class DataFetching : public PipelineContext {
   public:
    /// 给定配置文件
    DataFetching(const io::YAML_IO &yaml_file, RunMode run_mode = RunMode::PIPELINE);

    ~DataFetching() override;

    /// Context 接口
    /// 初始化，成功返回 true
    bool Init() override;

    /// 开始处理，完成后返回true，无论结果是否成功
    /// 要求完成后更新context status状态，以便 engine 确定它的完成情况
    /// NOTE: 模块内容应阻塞此Start函数，Start函数返回表明已执行完毕
    bool Start() override;

    /// 缓存中间结果
    bool Save() override;

    bool Load() override;

    /// 填写任务信息
    bool FillTaskInfo(TaskInfo &info) override;

   private:
    std::string data_url_;
    std::string user_name_;
    std::string password_;
    std::string local_data_path_;
    std::string map_name_;

    io::YAML_IO yaml_file_;

    std::string report_;
    std::string data_fetching_failed_reason_;

    int data_fetching_passed_;

    unsigned long int data_size_ = 0;
};

}  // namespace mapping::pipeline

#endif  // MAPPING_DATAFETCHING_H
