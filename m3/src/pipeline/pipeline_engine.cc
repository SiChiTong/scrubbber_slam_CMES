//
// Created by gaoxiang on 19-7-12.
//

#include "pipeline/pipeline_engine.h"
#include "check_in/check_in.h"
#include "check_out/check_out.h"
#include "common/version.h"
#include "dr_frontend/dr_frontend.h"
#include "io/file_io.h"
#include "lidar_frontend/lidar_frontend.h"
#include "loop_closing/loop_closing.h"
#include "merge_map/merge_map.h"
#include "optimization_s1/optimization_s1.h"
#include "optimization_s2/optimization_s2.h"
#include "preprocessing/preprocessing.h"
#include "src/pipeline/data_fetching/data_fetching.h"
#include "validation/validation.h"

#include <glog/logging.h>
#include <boost/format.hpp>

namespace mapping::pipeline {
using std::endl;

PipelineEngine::PipelineEngine(const std::string &ip) : http_server_(ip) {
    if (http_server_ == "127.0.0.1" || http_server_.find("192.168") != std::string::npos) {
        is_running_in_cloud_ = false;
    } else {
        is_running_in_cloud_ = true;
    }

    mapping_level_.emplace_back(R"($\bigstar$ )");
    mapping_level_.emplace_back(R"($\bigstar$ $\bigstar$ )");
    mapping_level_.emplace_back(R"($\bigstar$ $\bigstar$ $\bigstar$ )");
    mapping_level_.emplace_back(R"($\bigstar$ $\bigstar$ $\bigstar$ $\bigstar$ )");
    mapping_level_.emplace_back(R"($\bigstar$ $\bigstar$ $\bigstar$ $\bigstar$ $\bigstar$ )");
}

PipelineEngine::~PipelineEngine() { LOG(INFO) << "Deconstructing pipeline engine"; }

bool PipelineEngine::Init(const std::string &config_file) {
    config_ = io::YAML_IO(config_file);
    bool config_success = config_.IsOpened();
    LOG(INFO) << "loading yaml";
    if (!config_success) {
        LOG(ERROR) << "failed to open config file at " << config_file;
        return false;
    }

    task_info_.map_name = config_.GetValue<std::string>("map_name");
    send_email_ = config_.GetValue<bool>("send_email_after_complete");
    open_pdf_ = config_.GetValue<bool>("open_report_after_complete");
    if_updating_maps_ = config_.GetValue<bool>("if_map_updating");
    if_merge_maps_ = config_.GetValue<bool>("if_merge_maps");

    auto pipeline_contents = config_.GetValue<std::string>("pipeline_contents");

    LOG(INFO) << "making pipeline contents";
    if ((pipeline_contents.empty() || pipeline_contents == "default") && !if_updating_maps_ && !if_merge_maps_) {
        MakeDefaultPipeline();
    } else if (if_updating_maps_) {
        MakePipelineWithMapUpdating();
    } else if (if_merge_maps_) {
        MakePipelineWithMergingMap();
    } else {
        LOG(ERROR) << "Unknown pipeline contents: " << pipeline_contents;
        return false;
    }

    time_t now = time(nullptr);
    task_info_.start_time = *localtime(&now);

    return true;
}

void PipelineEngine::StartPipeline() {
    pipeline_thread = std::make_shared<std::thread>([this] { Work(); });
}

void PipelineEngine::StartPipeline(int step, const std::string &report) {
    int real_step = step - 1;
    if (real_step < min_step_range_ || real_step > max_step_range_) {
        // 出了什么问题的话，从头开始
        real_step = 0;
    }

    if (real_step >= context_.size()) {
        result_.finished = true;
        LOG(ERROR) << "step is not right: " << step;
        return;
    }

    current_job_ = context_.begin() + real_step;
    result_.report = report;

    // pop out
    while (context_.begin() != current_job_) {
        context_.pop_front();
    }

    // start pipeline
    pipeline_thread = std::make_shared<std::thread>([this] { Work(); });
}

bool PipelineEngine::IsFinished() { return GetResult().finished; }

void PipelineEngine::WaitForFinish() { pipeline_thread->join(); }

void PipelineEngine::Work() {
    PipelineResult result;
    LOG(INFO) << "Pipeline start ... ";
    boost::format fmt("总用时: %7.3f秒.\n");
    std::chrono::steady_clock::time_point pipeline_start_time = std::chrono::steady_clock::now();

    while (current_job_ != context_.end() && !close_pipeline_) {
        auto &job = *current_job_;
        LOG(INFO) << "Working on " << job->GetName();
        std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now(), t_end;
        boost::format fmt_time_usage("%s用时: %7.3f秒.\n");

        if (task_info_.enable_cloud_config) {
            SendMappingStatus();
        }

        if (!job->Init()) {
            result.failed_at = job->GetName();
            result.report += "Initialization failed at " + job->GetName() + "\n";
            std::string report;
            job->GenerateReport(report, true);
            result.report += report;
            t_end = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used =
                std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start);
            result.report += (fmt_time_usage % job->GetName() % time_used.count()).str();
            result.finished = true;
            SetResult(result);
            SetTaskInfo(job);
            break;
        }

        result.current_working = job->GetName();
        SetResult(result);

        if (!job->Start()) {
            /// 运行失败
            result.failed_at = job->GetName();
            std::string report;
            job->GenerateReport(report, true);
            result.report += report;

            t_end = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used =
                std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start);
            result.report += (fmt_time_usage % job->GetName() % time_used.count()).str();
            result.finished = true;

            SetResult(result);
            SetTaskInfo(job);
            job->FillTaskInfo(task_info_);
            break;
        }

        /// 运行成功
        std::string report;
        job->GenerateReport(report, true);
        result.report += report;
        t_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start);
        result.report += (fmt_time_usage % job->GetName() % time_used.count()).str();
        result.report += "\n\\hrulefill\n\n";
        SetResult(result);

        /// 更新任务信息
        SetTaskInfo(job);

        /// 缓存结果
        job->Save();
        current_job_++;

        job = nullptr;
    }

    auto pipeline_end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(pipeline_end_time - pipeline_start_time);
    result.report += (fmt % time_used.count()).str();
    task_info_.time_usage = time_used.count();
    SetResult(result);

    if (current_job_ == context_.end()) {
        result.finished = true;
        result.succeed = true;
        result.current_working = "";
        SetResult(result);
    }

    task_info_.success = result.succeed;

    time_t now = time(nullptr);
    task_info_.end_time = *localtime(&now);

    // clean
    context_.clear();
    current_job_ = context_.end();

    CopyResultFiles();
    GeneratePdfReport();

    if (task_info_.enable_cloud_config) {
        // 发送一个进度100
        std::string cmd =
            std::string(R"(curl http://61.148.198.86:2012/index.php/mapCollection/process/map-build-status)") +
            R"(\?access-token\=)" + task_info_.access_token + R"(\&task_id\=)" + std::to_string(task_info_.task_id) +
            R"(\&area_id\=)" + task_info_.area_id + R"(\&version\=)" + task_info_.version + R"(\&process\=100)" +
            R"(\&status\=2)" + R"(\&finish_time\=0)";
        LOG(INFO) << cmd;
        system(cmd.c_str());
    }
}

void PipelineEngine::SendMappingStatus() {
    LOG(INFO) << "Sending mapping status";
    int process = 0;
    auto result = GetResult();

    std::string job_name = result.current_working;
    if (job_name == "DataFetching") {
        process = 0;
    } else if (job_name == "CheckIn") {
        process = 5;
    } else if (job_name == "Preprocessing") {
        process = 10;
    } else if (job_name == "DRFrontend") {
        process = 15;
    } else if (job_name == "LidarFrontend") {
        process = 50;
    } else if (job_name == "OptimizationStage1") {
        process = 55;
    } else if (job_name == "LoopClosing"){
        process = 85;
    } else if (job_name == "OptimizationStage2"){
        process = 90;
    } else if (job_name == "Validation"){
        process = 95;
    } else if (job_name == "CheckOut"){
        process = 98;
    }

    if(result.finished){
        process = 100;
    }

    int status = 2;
    if (!result.finished) {
        status = 1;
    } else if (result.succeed) {
        status = 2;
    }

    std::string cmd =
        std::string(R"(curl http://61.148.198.86:2012/index.php/mapCollection/process/map-build-status)") +
        R"(\?access-token\=)" + task_info_.access_token + R"(\&task_id\=)" + std::to_string(task_info_.task_id) +
        R"(\&area_id\=)" + task_info_.area_id + R"(\&version\=)" + task_info_.version + R"(\&process\=)" +
        std::to_string(process) + R"(\&status\=)" + std::to_string(status) + R"(\&finish_time\=)" + std::to_string(100);
    LOG(INFO) << cmd;
    system(cmd.c_str());
}

void PipelineEngine::CopyResultFiles() {
    auto local_path = config_.GetValue<std::string>("data_fetching", "local_data_path");
    auto map_name = config_.GetValue<std::string>("map_name");

    if (!io::PathExists("/home/idriver/results/" + map_name)) {
        std::string cmd = "mkdir -p /home/idriver/results/" + map_name;
        system(cmd.c_str());
    }

    // map.db
    std::string cmd = "cp " + local_path + "/map.db /home/idriver/results/" + map_name + "/";
    system(cmd.c_str());

    // gmm
    cmd = "cp " + local_path + "/gmm.zip /home/idriver/results/" + map_name + "/gmm.zip";
    system(cmd.c_str());

    // keyframes.txt
    if (if_merge_maps_) {
        cmd = "cp " + local_path + "/merge_keyframes.txt /home/idriver/results/" + map_name + "/keyframes.txt";
    } else {
        cmd = "cp " + local_path + "/keyframes.txt /home/idriver/results/" + map_name + "/keyframes.txt";
    }
    system(cmd.c_str());

    std::ofstream fout("/home/idriver/results/" + map_name + "/task_info.txt");
    task_info_.Save(fout);
    fout.close();

    if (io::PathExists("./db2pdb")) {
        ConvertDB2PDB("./db2pdb", map_name);
    } else if (io::PathExists("./bin/db2pdb")) {
        ConvertDB2PDB("./bin/db2pdb", map_name);
    } else if (io::PathExists("./bin/sqlit_db_to_bin")) {
        SqlitDB2Bin("./bin/sqlit_db_to_bin", map_name);
    }

    if (is_running_in_cloud_) {
        // cloud server has apache http
        task_info_.map_db_url = "http://" + http_server_ + "/results/" + map_name + "/map.db";
        task_info_.pcd_url = "http://" + http_server_ + "/results/" + map_name + "/result.pcd";
        task_info_.gmm_url = "http://" + http_server_ + "/results/" + map_name + "/gmm.zip";
        task_info_.video_url = "http://" + http_server_ + "/results/" + map_name + "/simulation.flv";
    } else {
        // local is using python
        task_info_.map_db_url = "http://" + http_server_ + ":8000/" + map_name + "/map.db";
        task_info_.pcd_url = "http://" + http_server_ + ":8000/" + map_name + "/result.pcd";
        task_info_.gmm_url = "http://" + http_server_ + ":8000/" + map_name + "/gmm.zip";
        task_info_.video_url = "http://" + http_server_ + ":8000/" + map_name + "/simulation.flv";
    }
}

void PipelineEngine::ConvertDB2PDB(const std::string &tool_binary, const std::string &map_name) {
    std::string cmd = tool_binary + " --db_path=/home/idriver/results/" + map_name +
                      "/map.db --pdb_path=/home/idriver/results/" + map_name + "/map.pdb";
    system(cmd.c_str());
}

void PipelineEngine::SqlitDB2Bin(const std::string &tool_binary, const std::string &map_name) {
    std::string cmd = tool_binary + " --db_path=/home/idriver/results/" + map_name +
                      "/map.db --out_path=/home/idriver/results/" + map_name;
    system(cmd.c_str());
}

void PipelineEngine::ClosePipeline() {
    close_pipeline_ = true;
    LOG(INFO) << "Closing pipeline ... ";
    pipeline_thread->join();
}

PipelineResult PipelineEngine::GetResult() {
    std::unique_lock<std::mutex> lock(pipeline_mutex);
    return result_;
}

void PipelineEngine::SetResult(const PipelineResult &result) {
    std::unique_lock<std::mutex> lock(pipeline_mutex);
    result_ = result;
}

void PipelineEngine::MakeDefaultPipeline() {
    context_ = {
        std::make_shared<DataFetching>(config_),  std::make_shared<CheckIn>(config_),
        std::make_shared<Preprocessing>(config_), std::make_shared<DRFrontend>(config_),
        std::make_shared<LidarFrontend>(config_), std::make_shared<OptimizationStage1>(config_),
        std::make_shared<LoopClosing>(config_),   std::make_shared<OptimizationStage2>(config_),
        std::make_shared<Validation>(config_),    std::make_shared<CheckOut>(config_),
    };

    current_job_ = context_.begin();
}

void PipelineEngine::MakePipelineWithMapUpdating() {
    LOG(ERROR) << "To be determined...";
    context_ = {};
    // current_job_ = context_.begin();
}

void PipelineEngine::MakePipelineWithMergingMap() {
    context_ = {
        std::make_shared<DataFetching>(config_),       std::make_shared<CheckIn>(config_),
        std::make_shared<Preprocessing>(config_),      std::make_shared<DRFrontend>(config_),
        std::make_shared<LidarFrontend>(config_),      std::make_shared<OptimizationStage1>(config_),
        std::make_shared<MergeMap>(config_),           std::make_shared<LoopClosing>(config_),
        std::make_shared<OptimizationStage2>(config_), std::make_shared<Validation>(config_),
        std::make_shared<CheckOut>(config_),
    };

    current_job_ = context_.begin();
}

int PipelineEngine::AnalysisMappingLevel() {
    if (task_info_.scene_type == "室内" && task_info_.total_bag_size > 4 * 1024.0 * 1024.0 * 1024.0) {
        return 5;
    } else if (task_info_.scene_type == "室内" && ((task_info_.total_bag_size <= 4 * 1024.0 * 1024.0 * 1024.0) &&
                                                   (task_info_.total_bag_size > 1.5 * 1024.0 * 1024.0 * 1024.0))) {
        return 4;
    } else if (task_info_.scene_type == "室内" && task_info_.total_bag_size <= 1.5 * 1024.0 * 1024.0 * 1024.0) {
        return 3;
    } else if (task_info_.scene_type == "半室内" && task_info_.total_bag_size > 10 * 1024.0 * 1024.0 * 1024.0) {
        return 5;
    } else if (task_info_.scene_type == "半室内" && ((task_info_.total_bag_size <= 10 * 1024.0 * 1024.0 * 1024.0) &&
                                                     (task_info_.total_bag_size > 5 * 1024.0 * 1024.0 * 1024.0))) {
        return 4;
    } else if (task_info_.scene_type == "半室内" && ((task_info_.total_bag_size <= 5 * 1024.0 * 1024.0 * 1024.0) &&
                                                     (task_info_.total_bag_size > 3 * 1024.0 * 1024.0 * 1024.0))) {
        return 3;
    } else if (task_info_.scene_type == "半室内" && ((task_info_.total_bag_size <= 3 * 1024.0 * 1024.0 * 1024.0) &&
                                                     (task_info_.total_bag_size > 1.5 * 1024.0 * 1024.0 * 1024.0))) {
        return 2;
    } else if (task_info_.scene_type == "半室内" && task_info_.total_bag_size <= 1.5 * 1024.0 * 1024.0 * 1024.0) {
        return 1;
    } else if (task_info_.scene_type == "室外" && task_info_.total_bag_size > 15 * 1024.0 * 1024.0 * 1024.0) {
        return 4;
    } else if (task_info_.scene_type == "室外" && ((task_info_.total_bag_size <= 15 * 1024.0 * 1024.0 * 1024.0) &&
                                                   (task_info_.total_bag_size > 7 * 1024.0 * 1024.0 * 1024.0))) {
        return 3;
    } else if (task_info_.scene_type == "室外" && ((task_info_.total_bag_size <= 7 * 1024.0 * 1024.0 * 1024.0) &&
                                                   (task_info_.total_bag_size > 3 * 1024.0 * 1024.0 * 1024.0))) {
        return 2;
    } else if (task_info_.scene_type == "室外" && task_info_.total_bag_size <= 3 * 1024.0 * 1024.0 * 1024.0) {
        return 1;
    } else
        return 2;
}

void PipelineEngine::GeneratePdfReport() {
    std::string tex_file;
    auto local_path = config_.GetValue<std::string>("data_fetching", "local_data_path");
    std::string path = local_path + "/report.tex";
    LOG(INFO) << "saving report to " << path;

    // Brief Report
    std::ofstream tex_out(path);
    if (!tex_out) {
        LOG(ERROR) << "Cannot save file at " << path;
        return;
    }

    tex_out << R"(\documentclass{article})" << endl
            << R"(\usepackage[BoldFont, SlantFont]{xeCJK})" << endl
            << R"(\usepackage[margin = 2.4cm]{geometry})" << endl
            << R"(\usepackage{graphicx})" << endl
            << R"(\usepackage{subfigure})" << endl
            << R"(\usepackage{hyperref})" << endl
            << R"(\usepackage{array})" << endl
            << R"(\usepackage{color})" << endl
            << R"(\usepackage{booktabs})" << endl
            << R"(\usepackage{underscore})" << endl
            << R"(\usepackage{alphalph})" << endl
            << R"(\usepackage{amssymb})" << endl
            << R"(\renewcommand\arraystretch{1.2})" << endl;

    tex_out << R"(\renewcommand*{\thesubfigure}{%)" << endl;
    tex_out << R"(    \alphalph{\value{subfigure}}%)" << endl;
    tex_out << R"(}%)" << endl;
    tex_out << R"(\renewcommand\figurename{图} %)" << endl;
    tex_out << R"(\renewcommand\tablename{表} %)" << endl;

    std::replace(task_info_.map_name.begin(), task_info_.map_name.end(), '_', ' ');  // 替换下划线

    std::string cloud_or_on_site;
    if (common::mapping_in_cloud && is_running_in_cloud_) {
        cloud_or_on_site = "本报告由云端建图产生";
    } else {
        cloud_or_on_site = "本报告由现场建图产生";
    }

    if (task_info_.enable_cloud_config) {
        tex_out << R"(\title{)" << task_info_.project_name << R"(~场地ID:)" << task_info_.area_id << R"( 建图报告})"
                << endl;
        tex_out << R"(\author{建图软件版本 )" << common::mapping_major_version << "." << common::mapping_minor_version
                << "." << common::mapping_revision << R"(\\)" + cloud_or_on_site + "}" << endl;
    } else {
        tex_out << R"(\title{)" << task_info_.map_name << R"( 建图报告})" << endl;
        tex_out << R"(\author{建图软件版本 )" << common::mapping_major_version << "." << common::mapping_minor_version
                << "." << common::mapping_revision
                << R"(, 联系方式：\href{mailto:mapping@idriverplus.com}{mapping@idriverplus.com})"
                << R"(\\)" + cloud_or_on_site + "}" << endl;
    }

    tex_out << R"(\newfontfamily\urlfontfamily{Noto Sans CJK SC})" << endl;
    tex_out << R"(\def\UrlFont{\urlfontfamily})" << endl;

    tex_out << R"(\begin{document})" << endl;
    tex_out << R"(\maketitle)" << endl;

    tex_out << R"(\section{简报} )" << endl;
    tex_out << R"(\begin{table*}[!htp])" << endl;
    tex_out << R"(\caption{综合信息})" << endl;
    tex_out << R"(\centering)" << endl;
    tex_out << R"(\begin{tabular}{c|c|c|c|c|c|c|c})" << endl;
    tex_out << R"(	\hline\hline)" << endl;
    tex_out << R"(地图名称(ID) & \multicolumn{7}{c}{)" << task_info_.map_name << R"(} \\\hline)" << endl;

    int minutes = task_info_.time_usage / 60;
    int hours = minutes / 60;
    minutes -= hours * 60;
    double seconds = task_info_.time_usage - minutes * 60 - hours * 3600;
    double volumn = 0;
    std::string volumn_unit;
    if (task_info_.total_bag_size > 1024.0 * 1024.0 * 1024.0) {
        volumn = double(task_info_.total_bag_size) / (1024.0 * 1024.0 * 1024.0);
        volumn_unit = "GB";
    } else {
        volumn = task_info_.total_bag_size / (1024.0 * 1024.0);
        volumn_unit = "MB";
    }

    double length = 0;
    std::string length_unit;
    if (task_info_.length > 1000) {
        length = task_info_.length / 1000;
        length_unit = "km";
    } else {
        length = task_info_.length;
        length_unit = "m";
    }

    std::string checkout_str;
    if (task_info_.checkout_passed) {
        checkout_str = R"(\textcolor[rgb]{0.125,0.529,0.027}{成功})";
    } else {
        checkout_str = R"(\textcolor[rgb]{0.667,0.0,0.0}{失败})";
    }

    tex_out << R"(用时 & )";
    if (hours == 0) {
        tex_out << std::setprecision(3) << minutes << "分" << seconds << "秒 & 有效包数 & " << task_info_.valid_bags
                << R"( & 数据量 & \multicolumn{3}{c}{)" << volumn << volumn_unit << R"(} \\\hline)" << endl;
    } else {
        tex_out << std::setprecision(3) << hours << "小时" << minutes << "分" << seconds << "秒 & 有效包数 & "
                << task_info_.valid_bags << R"( & 数据量 & \multicolumn{3}{c}{)" << volumn << volumn_unit
                << R"(} \\\hline)" << endl;
    }
    tex_out << R"(场景类型 & )" << task_info_.scene_type << R"(& ODD & )"
            << (task_info_.scene_type == "室内" ? R"(\color{red}{ODD外})" : R"(\color{green}{ODD内})")
            << R"( & 难度 & \multicolumn{3}{c}{)" << mapping_level_.at(AnalysisMappingLevel() - 1) << R"(} \\\hline)"
            << endl;
    tex_out << R"(长度 & )" << length << length_unit << " & 推算面积 & " << task_info_.area << R"(平米 & 是否成功 & )"
            << (task_info_.success ? R"(\color{green}{是})" : R"(\color{red}{否})") << "& 关键帧数 & "
            << task_info_.total_keyframes << R"(\\\hline)" << endl;
    tex_out << R"(准出 & \multicolumn{2}{c|}{)" << checkout_str << R"(}& {失败理由（若有）} & \multicolumn{4}{c}{)"
            << (task_info_.failed_reason) << R"(} \\\hline)" << endl;
    tex_out << R"(地图数据 & \multicolumn{7}{c}{\footnotesize \url{)" << task_info_.map_db_url << R"(}} \\\hline)"
            << endl;
    tex_out << R"(GMM地图链接 & \multicolumn{7}{c}{\footnotesize \url{)" << task_info_.gmm_url
            << R"(}} \\ \hline\hline)" << endl;
    tex_out << R"(    \end{tabular})" << endl;
    tex_out << R"(\end{table*})" << endl;

    // Trajectory
    tex_out << R"(\begin{figure}[!htp])" << endl;
    tex_out << R"(\centering)" << endl;
    tex_out << R"(\includegraphics[width=0.6\textwidth]{trajectory.pdf})" << endl;
    tex_out << R"(\caption{各包轨迹})" << endl;
    tex_out << R"(\end{figure})" << endl;

    // GPS trajectory
    tex_out << R"(\begin{figure}[!htp])" << endl;
    tex_out << R"(\centering)" << endl;
    tex_out << R"(\includegraphics[width=0.6\textwidth]{gps-path.pdf})" << endl;
    tex_out << R"(\caption{GPS轨迹})" << endl;
    tex_out << R"(\end{figure})" << endl;

    // Detailed report
    tex_out << R"(\section{详细报告})" << endl;
    auto res = GetResult();
    auto rep = res.report;

    std::stringstream ss;
    ss << rep;
    while (!ss.eof()) {
        char line_data[1024];
        ss.getline(line_data, 1024);
        tex_out << std::string(line_data) << endl << endl;
    }

    tex_out << R"(\end{document})" << endl;
    tex_out.close();

    std::string cmd = "cd " + local_path + " && xelatex -synctex=1 -interaction=nonstopmode report.tex";
    system(cmd.c_str());

    if (open_pdf_) {
        cmd = "cd " + local_path + " && xdg-open report.pdf ";
        system(cmd.c_str());
    }

    cmd = "cp " + local_path + "/report.pdf /home/idriver/results/" + config_.GetValue<std::string>("map_name") + "/";
    system(cmd.c_str());

    if (send_email_) {
        std::string send_email_py = "./scripts/send_email.py";
        if (access(send_email_py.c_str(), F_OK) == 0) {
            LOG(INFO) << "Sending email ... ";
            cmd = "python3 scripts/send_email.py " + config_.GetValue<std::string>("map_name") +
                  " /home/idriver/results/" + config_.GetValue<std::string>("map_name");
            system(cmd.c_str());
        } else {
            LOG(INFO) << "Sending email ... ";
            cmd = "python3 scripts/send_email.pyc " + config_.GetValue<std::string>("map_name") +
                  " /home/idriver/results/" + config_.GetValue<std::string>("map_name");
            system(cmd.c_str());
        }
    }
}

}  // namespace mapping::pipeline
