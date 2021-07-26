//
// Created by gaoxiang on 19-7-12.
//

#ifndef MAPPING_PIPELINE_CONTEXT_H
#define MAPPING_PIPELINE_CONTEXT_H

#include <fstream>

#include "common/std_headers.h"
#include "pipeline/task_info.h"

namespace mapping::pipeline {

/// Pipeline内容接口
class PipelineContext {
   public:
    /// 每一步的状态
    enum class ContextStatus {
        PENDING = 0,  // 等待开始
        WORKING,      // 处理中
        SUCCEED,      // 已成功
        FAILED,       // 已失败
        OTHER         // 其他结果
    };

    enum class RunMode {
        PIPELINE,        // 作为流水线模块
        GEM_EXECUTABLE,  // 作为GEM二进制
    };

    explicit PipelineContext(RunMode run_mode = RunMode::PIPELINE) { run_mode_ = run_mode; }
    virtual ~PipelineContext() {}

    /// 获取当前步骤的状态
    ContextStatus Status() {
        std::unique_lock<std::mutex> lock(status_mutex_);
        return context_status_;
    }

    /// 设定自身的状态
    void SetStatus(ContextStatus status) {
        std::unique_lock<std::mutex> lock(status_mutex_);
        context_status_ = status;
    }

    /// 获取当前步骤的名称
    std::string GetName() const { return context_name_; }

    /// Context 接口
    /// 初始化，成功返回 true
    virtual bool Init() = 0;

    /// 开始处理，完成后返回true，无论结果是否成功
    /// 要求完成后更新context status状态，以便 engine 确定它的完成情况
    /// NOTE: 模块内容应阻塞此Start函数，Start函数返回表明已执行完毕
    virtual bool Start() = 0;

    /// 生成执行报告, 报告内容在report中
    /// 用verbose参数控制是否产生详细报告
    virtual bool GenerateReport(std::string &report, bool verbose = true) {
        report += context_name_ + ":" + report_;
        return true;
    }

    void GenerateGemReport() {
        report_ += context_name_ + ":";
        return;
    }

    /// 缓存中间结果
    virtual bool Save() { return true; }

    /// 读取中间结果
    virtual bool Load() { return true; }

    /// 填写任务信息
    virtual bool FillTaskInfo(TaskInfo &info) { return true; }

    RunMode GetRunMode() const { return run_mode_; }

    void SaveGemReport(std::string report_path) {
        std::ofstream tex_out;
        tex_out.open(report_path, std::ios::app);
        std::stringstream ss;
        report_ += "\n";
        report_ += "\n\\hrulefill\n\n";
        ss << report_;
        while (!ss.eof()) {
            char line_data[1024];
            ss.getline(line_data, 1024);
            tex_out << std::string(line_data) << std::endl << std::endl;
        }
        tex_out.close();
    }

   protected:
    /// 打印并记录在report中
    void LogAndReport(const std::string &log);

   protected:
    std::mutex status_mutex_;                                // 状态锁
    ContextStatus context_status_ = ContextStatus::PENDING;  // 状态
    RunMode run_mode_ = RunMode::PIPELINE;                   // 运行模式
    std::string context_name_;                               // 名称
    std::string report_;                                     // 自身的report
    std::string gem_report_name_ = "gem_report.tex";
    std::mutex gem_mutex_;  // 锁定gem's report
};

}  // namespace mapping::pipeline

#endif  // MAPPING_PIPELINE_CONTEXT_H
