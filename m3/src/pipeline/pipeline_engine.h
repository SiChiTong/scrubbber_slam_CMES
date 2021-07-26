//
// Created by gaoxiang on 19-7-12.
//

#ifndef MAPPING_PIPELINE_ENGINE_H
#define MAPPING_PIPELINE_ENGINE_H

#include <fstream>
#include "common/num_type.h"
#include "common/std_headers.h"
#include "io/yaml_io.h"
#include "pipeline/pipeline_context.h"

namespace mapping::pipeline {

/// Pipeline执行结果
struct PipelineResult {
    bool finished = false;                 // 是否已结束
    bool succeed = false;                  // 最终结果是否成功
    std::string report;                    // 执行报告
    std::string failed_at;                 // 若失败，失败在哪个阶段
    std::string current_working = "None";  // 当前执行阶段
};

/// Pipeline 引擎
/// 按既定顺序执行整条流水线
class PipelineEngine {
   public:
    explicit PipelineEngine(const std::string &ip = "127.0.0.1");

    ~PipelineEngine();

    /// 初始化，填入配置文件
    bool Init(const std::string &config_file = "./config/mapping.yaml");

    /// 启动流水线
    void StartPipeline();

    /// 从中间某步启动流水线
    /// 为使交互友好，取step为从1开始的整数
    void StartPipeline(int step, const std::string &report = "");

    /// 关闭流水线
    /// 会等待当前任务结束
    void ClosePipeline();

    /// 判断是否结束
    bool IsFinished();

    /// 等待流水线运行结束
    void WaitForFinish();

    /// 获取流水线结果
    PipelineResult GetResult();

    /// 生成pdf格式报告
    void GeneratePdfReport();

    /// 获取以GB为单位的数据包大小
    double GetBagSize() const { return task_info_.total_bag_size / (1024.0 * 1024.0 * 1024.0); }

    void SetTaskInfo(const TaskInfo &info) {
        std::unique_lock<std::mutex> lock(info_mutex_);
        task_info_ = info;
    }

    TaskInfo GetTaskInfo() {
        std::unique_lock<std::mutex> lock(info_mutex_);
        return task_info_;
    }

    /// 上报建图进度
    void SendMappingStatus();

   private:
    /// 建立默认建图流水线
    void MakeDefaultPipeline();

    // 建立地图更新流水线
    void MakePipelineWithMapUpdating();

    // 建立地图合并流水线
    void MakePipelineWithMergingMap();

    /// 流水线线程
    void Work();

    /// 将建图结果挪至下载目录
    void CopyResultFiles();

    /// 获取建图等级，从难倒易分5级
    int AnalysisMappingLevel();

    /// 将db转到pdb
    void ConvertDB2PDB(const std::string &tool_binary, const std::string &map_name);

    /// 将db切分bin
    void SqlitDB2Bin(const std::string &tool_binary, const std::string &map_name);

    void SetResult(const PipelineResult &result);

    void SetTaskInfo(std::shared_ptr<PipelineContext> job) {
        std::unique_lock<std::mutex> lock(info_mutex_);
        job->FillTaskInfo(task_info_);
    }

    std::deque<std::shared_ptr<PipelineContext>> context_;
    std::deque<std::shared_ptr<PipelineContext>>::iterator current_job_;

    /// 执行pipeline的线程和锁
    std::shared_ptr<std::thread> pipeline_thread;
    std::mutex pipeline_mutex;
    bool close_pipeline_ = false;

    /// 执行结果
    PipelineResult result_;

    /// 配置文件
    io::YAML_IO config_;

    /// 任务基本信息
    TaskInfo task_info_;
    std::mutex info_mutex_;
    std::vector<std::string> mapping_level_;

    std::string http_server_ = "127.0.0.1";

    bool is_running_in_cloud_ = true;
    bool send_email_ = false;
    bool open_pdf_ = true;
    bool is_full_pipeline_ = true;
    bool if_updating_maps_ = false;
    bool if_merge_maps_ = false;

    /// 其他需要临时保存在内存中的结果
    static constexpr int min_step_range_ = 0;  // 最小的步骤号
    static constexpr int max_step_range_ = 9;  // 最大的步骤号
};

}  // namespace mapping::pipeline

#endif  // MAPPING_PIPELINE_ENGINE_H
