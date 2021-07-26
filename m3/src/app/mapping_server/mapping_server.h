//
// Created by gaoxiang on 19-7-15.
//

#ifndef MAPPING_MAPPING_SERVER_H
#define MAPPING_MAPPING_SERVER_H

#include <atomic>
#include <boost/format.hpp>
#include <utility>
#include "pipeline/pipeline_context.h"
#include "pipeline/pipeline_engine.h"

namespace mapping::app {

using mapping::pipeline::PipelineEngine;
using mapping::pipeline::PipelineResult;

struct PipelineInfo {
    PipelineInfo(unsigned long id, std::string name, std::string url)
        : id_(id), name_(std::move(name)), data_url_(std::move(url)) {}

    bool is_new_pipeline_ = true;  // 如果是新开的pipeline，则此处为true
    int step_ = 0;                 // 如果是重启的，则从中断的step开始
    std::string report;            // 如果是重启的，应该有重启前的report

    unsigned long id_;
    std::string name_;
    std::string data_url_;

    pipeline::TaskInfo task_info_;
};

/// 建图服务器端
class MappingServer {
   public:
    MappingServer();

    /// 启动服务器线程
    bool Start();

    /// 执行命令
    bool RunCommand(std::string &command);

    void Spin();

   private:
    /// 服务器主线程
    void ServerLoop();

    /// 轮询线程
    void QueryLoop();

    /// 解析指令
    bool ParseCommand(const std::string &cmd);

    /// 打印各engine的信息
    void PrintPipelineStatus();

    /// 打印pipeline report
    void ShowPipelineReport();

    /// 新建流水线
    void CreateNewPipeline();

    /// 新建TODO list中的流水线
    void CreateTODOPipeline();

    /// 定期将服务器状态写入文件
    void LogStatus();

    /// 重开某条流水线
    void RestartPipeline();

    /// 新地图建图和拼接的流水线
    void BuildAndMergeMapsPipeline();

    /// 根据内存占用情况设置最大pipeline数量
    void SetMaxPipelineNumByMemory();

    /// 保存服务器中各pipeline的状态信息
    void SavePipelineStatus(const std::string &file_name);

    /// 读取状态文件中的pipeline信息并尝试恢复
    bool LoadPipelineStatus(const std::string &file_name);

    /// 将字符串step转为int的step
    int ToIntStep(const std::string &step);

    std::string LoadReport(std::ifstream &fin);

    /// 将某ip记入黑名单
    void SetBlackList(const std::string &ip);

    std::shared_ptr<std::thread> server_thread_ = nullptr;  // 服务器线程
    std::shared_ptr<std::thread> query_thread_ = nullptr;   // 查询线程
    std::mutex pipeline_mutex_;
    std::atomic<bool> server_running_;

    std::string server_ip_ = "127.0.0.1";
    int port_ = 6666;  // 建图服务器端口
    bool use_internal_ip_ = false;
    std::string internal_ip_ = "0.0.0.0";  // 在云端，绑定0.0.0.0

    int max_pipeline_num_ = 2;               // 最大流水线数量
    std::atomic<int> running_pipeline_num_;  // 正在运行的流水线

    std::map<unsigned long, std::shared_ptr<PipelineEngine>> pipelines_;  // 实际运行时流水线
    std::map<unsigned long, PipelineResult> archived_result_;             // 已归档的流水线
    std::map<unsigned long, PipelineInfo> pipeline_todo_;                 // 未执行的流水线
    std::map<std::string, unsigned long> pipeline_name_to_id_;            // 流水线名称到id的映射

    int num_failed_ = 0, num_success_ = 0;

    /// 内存占用相关
    bool dynamic_allocate_ = false;
    static const double memory_cost_per_engine_;     // 每条流水线内存使用
    static const double memory_cost_bag_size_ratio;  // 硬盘包大小和内存大小的比例
    double memory_used_ = 0.0;
    double memory_limit_ = 32.0;
    const int hard_pipeline_limit = 12;  // 硬性限制,pipeline太多可能导致硬盘跟不上

    /// 自动保存和恢复
    bool auto_save_ = true;

    std::string base_command_;         // 基础指令
    std::vector<std::string> params_;  // 指令参数
    std::string answer_;               // 对服务端指令的应答

    std::map<std::string, int> blacklist_ip_;  // ip黑名单及非法操作次数
    const int blacklist_cnt_ = 1;              // 超过该次数非法操作会被列入黑名单
};

}  // namespace mapping::app

#endif  // MAPPING_MAPPING_SERVER_H
