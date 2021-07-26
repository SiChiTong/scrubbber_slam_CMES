//
// Created by gaoxiang on 19-7-15.
//

#include <arpa/inet.h>
#include <glog/logging.h>
#include <netinet/in.h>
#include <cstdlib>
#include <fstream>
#include <regex>

#include "app/mapping_server/mapping_server.h"
#include "io/file_io.h"
#include "pipeline/pipeline_engine.h"

namespace mapping::app {

unsigned long s_pipeline_id = 0;
using namespace mapping::pipeline;

const double MappingServer::memory_cost_per_engine_ = 8.0;
const double MappingServer::memory_cost_bag_size_ratio = 2.0;

MappingServer::MappingServer() {
    // load server ip and port in config/server.yaml
    io::YAML_IO yaml("./config/server.yaml");
    port_ = yaml.GetValue<int>("server.port");
    server_ip_ = yaml.GetValue<std::string>("server.ip");
    use_internal_ip_ = yaml.GetValue<bool>("server.use_internal_ip");
    internal_ip_ = yaml.GetValue<std::string>("server.internal_ip");
    max_pipeline_num_ = yaml.GetValue<int>("server.max_pipelines");
    auto_save_ = yaml.GetValue<bool>("server.auto_save");

    if (max_pipeline_num_ == -1) {
        // 动态分配最大并发数
        dynamic_allocate_ = true;
        max_pipeline_num_ = 1;
        memory_limit_ = yaml.GetValue<double>("server.max_memory");
    }

    running_pipeline_num_.store(0);

    if (!io::PathExists("/home/idriver/results")) {
        LOG(INFO) << "creating results dir";
        std::string cmd = "mkdir -p /home/idriver/results";
        system(cmd.c_str());
    }
}

bool MappingServer::Start() {
    server_running_ = true;

    if (!use_internal_ip_) {
        LOG(INFO) << "Launch http server";
        std::string cmd = "cd /home/idriver/results && python -m SimpleHTTPServer 1>/dev/null 2>&1 &";
        system(cmd.c_str());
    }

    if (auto_save_) {
        LoadPipelineStatus("./happy");
    }

    server_thread_ = std::make_shared<std::thread>(&MappingServer::ServerLoop, this);
    query_thread_ = std::make_shared<std::thread>(&MappingServer::QueryLoop, this);
    return true;
}

void MappingServer::Spin() {
    server_thread_->join();
    query_thread_->join();
}

void MappingServer::QueryLoop() {
    while (server_running_.load()) {
        sleep(5);
        {
            std::unique_lock<std::mutex> lock(pipeline_mutex_);
            running_pipeline_num_.store(int(pipelines_.size()));
            int num_running = running_pipeline_num_.load();

            double memory_used = 0;

            for (auto iter = pipelines_.begin(); iter != pipelines_.end();) {
                if (iter->second->IsFinished()) {
                    auto pipeline = *iter;
                    auto result = pipeline.second->GetResult();
                    archived_result_.insert({pipeline.first, result});
                    pipeline.second->ClosePipeline();
                    iter = pipelines_.erase(iter);
                    running_pipeline_num_.store(num_running - 1);

                    if (result.succeed) {
                        num_success_++;
                    } else {
                        num_failed_++;
                    }

                } else {
                    double memory_pipeline = iter->second->GetBagSize();
                    memory_pipeline =
                        memory_pipeline <= 1e-2 ? memory_cost_per_engine_ : memory_pipeline;  // in case of zero
                    memory_used += memory_pipeline * memory_cost_bag_size_ratio;

                    iter++;
                }
            }

            memory_used_ = memory_used;
            if (dynamic_allocate_) {
                SetMaxPipelineNumByMemory();
            }

            if (num_running < max_pipeline_num_ && !pipeline_todo_.empty()) {
                CreateTODOPipeline();
            }
        }

        LogStatus();
        if (auto_save_) {
            SavePipelineStatus("./happy");
        }
    }
}

void MappingServer::LogStatus() {
    PrintPipelineStatus();
    std::ofstream fout("./server.log");
    if (fout && !answer_.empty()) {
        fout << answer_;
        fout.close();
    } else {
        LOG(ERROR) << "cannot open file of ./server.log";
    }
}

void MappingServer::SetMaxPipelineNumByMemory() {
    double free_memory = memory_limit_ - memory_used_;
    if (free_memory >= 0) {
        // enlarge it or keep same
        max_pipeline_num_ =
            running_pipeline_num_.load() + int(free_memory / (memory_cost_per_engine_ * memory_cost_bag_size_ratio));
    } else {
        max_pipeline_num_ = running_pipeline_num_.load() - 1;  // wait for decrease
    }

    if (max_pipeline_num_ > hard_pipeline_limit) max_pipeline_num_ = hard_pipeline_limit;
}

void MappingServer::CreateTODOPipeline() {
    auto first_iter = pipeline_todo_.begin();
    int num_running = running_pipeline_num_.load();

    // create engine from default yaml
    std::shared_ptr<PipelineEngine> new_engine(new PipelineEngine(server_ip_));
    std::string config_file = "./config/" + first_iter->second.name_ + ".yaml";

    if (!new_engine->Init(config_file)) {
        LOG(ERROR) << "failed to init task " << first_iter->second.name_;
        return;
    }

    pipelines_.insert({first_iter->first, new_engine});

    if (first_iter->second.is_new_pipeline_) {
        boost::format fmt("Start pipeline %s with id %d.");
        answer_ = (fmt % first_iter->second.name_ % first_iter->second.id_).str();
        pipeline_name_to_id_.insert({first_iter->second.name_, first_iter->first});
        new_engine->StartPipeline();
    } else {
        boost::format fmt("Restart pipeline %s with id %d at step %d.");
        answer_ = (fmt % first_iter->second.name_ % first_iter->second.id_ % first_iter->second.step_).str();
        new_engine->SetTaskInfo(first_iter->second.task_info_);
        new_engine->StartPipeline(first_iter->second.step_, first_iter->second.report);
    }

    running_pipeline_num_.store(num_running + 1);
    pipeline_todo_.erase(first_iter);
}

void MappingServer::ServerLoop() {
    int server_sockfd;
    int client_sockfd;
    int len = 0;
    struct sockaddr_in my_addr;
    struct sockaddr_in remote_addr;
    socklen_t sin_size;
    const int BUFFER_SIZE = 10240;
    char buf[BUFFER_SIZE] = "";
    memset(&my_addr, 0, sizeof(my_addr));

    my_addr.sin_family = AF_INET;
    if (use_internal_ip_) my_addr.sin_addr.s_addr = inet_addr(internal_ip_.c_str());
    else
        my_addr.sin_addr.s_addr = inet_addr(server_ip_.c_str());
    my_addr.sin_port = htons(port_);

    if ((server_sockfd = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        LOG(ERROR) << "socket error";
        server_running_.store(false);
        return;
    }

    int on = 1;
    setsockopt(server_sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    // setsockopt(server_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *) &timeout, sizeof(struct timeval));

    if (bind(server_sockfd, (struct sockaddr *)&my_addr, sizeof(my_addr)) < 0) {
        LOG(ERROR) << "bind error";
        LOG(ERROR) << "Error code: " << errno;
        server_running_.store(false);
        return;
    }

    if (listen(server_sockfd, 5) < 0) {
        LOG(ERROR) << "listen error";
        server_running_.store(false);
        return;
    };

    sin_size = sizeof(struct sockaddr_in);
    struct timeval timeout = {3, 0};

    while (server_running_.load()) {
        timeout.tv_sec = 0;
        setsockopt(server_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(struct timeval));

        if ((client_sockfd = accept(server_sockfd, (struct sockaddr *)&remote_addr, &sin_size)) < 0) {
            LOG(ERROR) << "accept error";
            return;
        }

        // get its ip
        struct sockaddr_in addr;
        socklen_t addr_size = sizeof(struct sockaddr_in);
        int res = getpeername(client_sockfd, (struct sockaddr *)&addr, &addr_size);
        char *clientip_str = new char[20];
        strcpy(clientip_str, inet_ntoa(addr.sin_addr));
        std::string client_ip(clientip_str);

        LOG(INFO) << "Get connection from " << client_ip;
        if (blacklist_ip_.find(client_ip) != blacklist_ip_.end() && blacklist_ip_[client_ip] >= blacklist_cnt_) {
            LOG(INFO) << "Reject connection from " << client_ip << " because it is in the blacklist";
            close(client_sockfd);
            continue;
        }

        // set timeout to 3s
        timeout.tv_sec = 3;
        setsockopt(client_sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(struct timeval));
        setsockopt(client_sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(struct timeval));

        len = recv(client_sockfd, buf, BUFFER_SIZE, 0);

        if (len <= 0) {
            LOG(INFO) << "close client";
            close(client_sockfd);
            continue;
        }

        if (len > 0 && len < BUFFER_SIZE) {
            buf[len] = '\0';
        }

        std::string cmd(buf);
        LOG(INFO) << "Receive cmd " << cmd;

        bool ret = RunCommand(cmd);
        memset(buf, 0, BUFFER_SIZE);

        if (!ret) {
            LOG(INFO) << "Invalid operation, this ip will be put into blacklist";
            SetBlackList(client_ip);
            close(client_sockfd);
        } else {
            // send answer to client
            send(client_sockfd, answer_.c_str(), answer_.size(), 0);
            close(client_sockfd);
        }
    }

    // wait pipelines to end
    std::unique_lock<std::mutex> lock(pipeline_mutex_);
    LOG(WARNING) << "Server will be closed. Waiting for pipelines to stop.";
    for (auto &pipeline : pipelines_) {
        pipeline.second->ClosePipeline();
        LOG(WARNING) << "close pipeline " << pipeline.first;
    }

    LOG(WARNING) << "server closed";
    close(server_sockfd);
}

void MappingServer::SetBlackList(const std::string &ip) {
    if (blacklist_ip_.find(ip) == blacklist_ip_.end()) {
        blacklist_ip_.insert({ip, 1});
    } else {
        blacklist_ip_[ip]++;
    }
}

bool MappingServer::RunCommand(std::string &command) {
    if (!ParseCommand(command)) {
        return false;
    }

    if (base_command_ == "list") {
        PrintPipelineStatus();
    } else if (base_command_ == "show") {
        ShowPipelineReport();
    } else if (base_command_ == "start") {
        CreateNewPipeline();
    } else if (base_command_ == "close") {
        answer_ = "Server will be closed!";
        server_running_.store(false);
    } else if (base_command_ == "restart") {
        RestartPipeline();
    } else if (base_command_ == "merge") {
        BuildAndMergeMapsPipeline();
    }

    return true;
}

void MappingServer::SavePipelineStatus(const std::string &file_name) {
    std::ofstream fout(file_name);
    if (!fout) return;
    std::unique_lock<std::mutex> lock(pipeline_mutex_);

    std::map<unsigned long, std::string> id_name;
    for (auto &name_id : pipeline_name_to_id_) {
        id_name.insert({name_id.second, name_id.first});
    }

    for (auto &in : id_name) {
        PipelineResult result;
        std::string status, step;
        std::string info;
        fout << in.first << " " << in.second << " ";
        std::string report;

        bool save_task_info = false;
        TaskInfo task_info;

        if (pipelines_.find(in.first) != pipelines_.end()) {
            // 正在运行的
            result = pipelines_[in.first]->GetResult();
            if (result.finished) {
                if (result.succeed) {
                    status = "succeed";
                    step = "end";
                } else {
                    status = "failed";
                    step = result.failed_at;
                }
            } else {
                status = "working";
                step = result.current_working;
            }

            report = result.report;
            task_info = pipelines_[in.first]->GetTaskInfo();
            save_task_info = true;

        } else if (archived_result_.find(in.first) != archived_result_.end()) {
            // 已经归档的
            result = archived_result_.at(in.first);
            report = result.report;
            if (result.succeed) {
                status = "succeed";
                step = "end";
            } else {
                status = "failed";
                step = result.failed_at;
            }
        } else if (pipeline_todo_.find(in.first) != pipeline_todo_.end()) {
            // 还未执行的
            status = "TODO";
            step = "None";
        }

        fout << status << " " << step << std::endl;
        if (save_task_info) {
            task_info.Save(fout);
        }

        fout << report << std::endl;
    }
    fout.close();
}

bool MappingServer::LoadPipelineStatus(const std::string &file_name) {
    std::ifstream fin(file_name);
    if (!fin) {
        LOG(WARNING) << "Not happy!";
        return false;
    }

    LOG(INFO) << "Auto-loading ... ";
    std::map<unsigned long, std::string> id_to_name;

    while (!fin.eof() && fin.good()) {
        char buff[256] = {};
        fin.getline(buff, 256);
        int id = 0;
        std::string name, status, step;
        char name_buff[256] = {0}, status_buff[128] = {0}, step_buff[128] = {0};
        if (sscanf(buff, "%d %s %s %s", &id, name_buff, status_buff, step_buff) < 0) {
            continue;
        }

        name = std::string(name_buff);
        status = std::string(status_buff);
        step = std::string(step_buff);

        if (id_to_name.find(id) != id_to_name.end()) {
            LOG(WARNING) << "pipeline id " << id << " already exist, skip it";
            continue;
        }

        if (name.empty()) {
            continue;
        }

        id_to_name.insert({id, name});
        pipeline_name_to_id_.insert({name, id});

        LOG(INFO) << "Load pipeline " << id << " with name " << name;

        if (status == "succeed" || status == "failed") {
            // 已完成
            std::string report = LoadReport(fin);
            PipelineResult result;
            result.finished = true;
            result.succeed = (status == "succeed");
            result.report = report;
            if (!result.succeed) {
                result.failed_at = step;
            }

            archived_result_.insert({id, result});
        } else if (status == "working") {
            // 已开始，未完成
            PipelineInfo info(id, name, "");  // URL 已经填入pipeline的配置文件，这里留空
            info.step_ = ToIntStep(step);     // 从中间开始
            if (info.step_ == -1) {
                info.step_ = 1;
            }  // 可能存在文件存了一半的情况
            info.is_new_pipeline_ = false;

            pipeline::TaskInfo task_info;
            task_info.Load(fin);
            info.task_info_ = task_info;

            std::string report = LoadReport(fin);
            info.report = report;
            pipeline_todo_.insert({id, info});
            LOG(INFO) << "report: \n" << report;

        } else if (status == "TODO") {
            // 未开始
            PipelineInfo info(id, name, "");  // URL 已经填入pipeline的配置文件，这里留空
            pipeline_todo_.insert({id, info});
        }

        s_pipeline_id = id + 1;
    }
    return true;
}

std::string MappingServer::LoadReport(std::ifstream &fin) {
    std::string report;
    std::regex pattern(
        R"(([0-9]+) (.+) (working|succeed|failed|TODO) (end|DataFetching|Preprocessing|DRFrontend|LidarFrontend|OptimizationStage1|OptimizationStage2|LoopClosing|Validation|CheckIn|CheckOut|None)*)");

    while (!fin.eof()) {
        auto pos = fin.tellg();
        char buff[10000] = {};
        fin.getline(buff, 10000);
        std::string str(buff);

        if (std::regex_match(str, pattern)) {
            fin.seekg(pos);
            break;
        }

        str += "\n";
        report.append(str);
    }
    return report;
}

void MappingServer::RestartPipeline() {
    std::unique_lock<std::mutex> lock(pipeline_mutex_);
    std::string pipeline_name = params_[0];
    std::string step = params_[1];

    if (pipeline_name_to_id_.find(pipeline_name) == pipeline_name_to_id_.end()) {
        answer_ = "Pipeline " + pipeline_name + " does not exist.";
        return;
    }
    auto id = pipeline_name_to_id_.find(pipeline_name)->second;

    if (archived_result_.find(id) == archived_result_.end()) {
        answer_ = "Pipeline " + pipeline_name + " is still running, cannot restart a pipeline that is still active.";
        return;
    }

    /// Create a todo pipeline
    unsigned long pipeline_id = id;
    PipelineInfo info(pipeline_id, pipeline_name, "");
    info.is_new_pipeline_ = false;
    info.step_ = atoi(step.c_str());

    // remove from archived
    archived_result_.erase(id);
    pipeline_todo_.insert({pipeline_id, info});
    answer_ = "Restart pipeline " + pipeline_name + ", put it into todo list.";
}

void MappingServer::BuildAndMergeMapsPipeline() {
    std::unique_lock<std::mutex> lock(pipeline_mutex_);
    assert(params_.size() == 3);

    std::string pipeline_name = params_[0];
    std::string input_map = params_[0];
    std::string url = params_[1];
    std::string target_map = params_[2];

    if (pipeline_name_to_id_.find(pipeline_name) != pipeline_name_to_id_.end()) {
        answer_ = "Pipeline " + pipeline_name + " already exist, I'll skip this command.";
        return;
    }

    int num_running = running_pipeline_num_.load();

    if (pipeline_todo_.empty() && num_running < max_pipeline_num_) {
        // 创建新流水线，立即执行
        std::shared_ptr<PipelineEngine> new_engine(new PipelineEngine(server_ip_));
        io::YAML_IO default_yaml("./config/mapping.yaml");
        default_yaml.SetValue("map_name", pipeline_name);
        default_yaml.SetValue("data_fetching", "data_url", url);
        default_yaml.SetValue("if_merge_maps", true);
        std::string origin_map_db_path = "/home/idriver/results/" + target_map;
        default_yaml.SetValue("origin_map_db_path", origin_map_db_path);
        std::string new_config_file = "./config/" + pipeline_name + ".yaml";
        default_yaml.Save(new_config_file);

        new_engine->Init(new_config_file);
        unsigned long pipeline_id = s_pipeline_id++;
        pipelines_.insert({pipeline_id, new_engine});
        pipeline_name_to_id_.insert({pipeline_name, pipeline_id});

        boost::format fmt("Start pipeline %s with id %d.");
        answer_ = (fmt % pipeline_name % pipeline_id).str();
        new_engine->StartPipeline();
        running_pipeline_num_.store(num_running + 1);

    } else {
        // 进入todo
        io::YAML_IO default_yaml("./config/mapping.yaml");
        default_yaml.SetValue("map_name", pipeline_name);
        default_yaml.SetValue("data_fetching", "data_url", url);
        default_yaml.SetValue("if_merge_maps", true);
        std::string new_config_file = "./config/" + pipeline_name + ".yaml";
        default_yaml.Save(new_config_file);

        answer_ = "Sorry, number of running pipelines reaches max: " + std::to_string(num_running) + "/" +
                  std::to_string(max_pipeline_num_) +
                  +" or todo list not empty, I'll put this pipeline into todo list.";

        unsigned long pipeline_id = s_pipeline_id++;
        PipelineInfo info(pipeline_id, pipeline_name, url);
        pipeline_todo_.insert({pipeline_id, info});
        pipeline_name_to_id_.insert({pipeline_name, pipeline_id});
    }
}

bool MappingServer::ParseCommand(const std::string &cmd) {
    if (cmd.empty()) return false;

    // split by space
    std::vector<std::string> sv;
    std::istringstream iss(cmd);
    std::string temp;
    while (std::getline(iss, temp, ' ')) {
        sv.emplace_back(std::move(temp));
    }

    if (sv.empty()) {
        return false;
    }

    base_command_ = sv[0];
    params_.clear();
    if (sv.size() > 1) {
        params_ = {sv.begin() + 1, sv.end()};
    }

    // check command
    std::string command = base_command_;
    int argc = params_.size();
    if (command == "server" && argc == 0) {
        return true;
    } else if (command == "list" && argc == 0) {
        return true;
    } else if (command == "start" && argc == 2) {
        return true;
    } else if (command == "show" && argc == 1) {
        return true;
    } else if (command == "close" && argc == 0) {
        return true;
    } else if (command == "restart" && argc == 2) {
        return true;
    } else if (command == "version" && argc == 0) {
        return true;
    } else if (command == "merge" && argc == 3) {
        return true;
    } else {
        return false;
    }

    return false;
}

void MappingServer::PrintPipelineStatus() {
    std::unique_lock<std::mutex> lock(pipeline_mutex_);
    boost::format fmt("%d\t%s\t\t%s\t%s");
    answer_ = "Success: " + std::to_string(num_success_) + "Failed: " + std::to_string(num_failed_) +
              "/Running: " + std::to_string(running_pipeline_num_.load()) + " of " + std::to_string(max_pipeline_num_) +
              "/TODO: " + std::to_string(pipeline_todo_.size()) + "\n";
    answer_ += "id\tStatus       \t\tStep          \tName\n";
    std::map<unsigned long, std::string> id_name;
    for (auto &name_id : pipeline_name_to_id_) {
        id_name.insert({name_id.second, name_id.first});
    }

    for (auto &in : id_name) {
        PipelineResult result;
        std::string status, step;
        std::string info;

        if (pipelines_.find(in.first) != pipelines_.end()) {
            // 正在运行的
            result = pipelines_[in.first]->GetResult();
            if (result.finished) {
                if (result.succeed) {
                    status = "Succeed";
                    step = "End";
                } else {
                    status = "Failed";
                    step = result.failed_at;
                }
            } else {
                status = "Working";
                step = result.current_working;
            }

            info = (fmt % in.first % status % step % in.second).str();
        } else if (archived_result_.find(in.first) != archived_result_.end()) {
            // 已经归档的
            result = archived_result_.at(in.first);
            if (result.succeed) {
                status = "Succeed";
                step = "End";
            } else {
                status = "Failed";
                step = result.failed_at;
            }
            info = (fmt % in.first % status % step % in.second).str();
        } else if (pipeline_todo_.find(in.first) != pipeline_todo_.end()) {
            // 还未执行的
            info = (fmt % in.first % "TODO" % "None" % in.second).str();
        }
        answer_ = answer_ + info + "\n";
    }
}

void MappingServer::CreateNewPipeline() {
    std::unique_lock<std::mutex> lock(pipeline_mutex_);
    assert(params_.size() == 2);
    std::string pipeline_name = params_[0];
    std::string url = params_[1];

    if (pipeline_name_to_id_.find(pipeline_name) != pipeline_name_to_id_.end()) {
        answer_ = "Pipeline " + pipeline_name + " already exist, I'll skip this command.";
        return;
    }

    int num_running = running_pipeline_num_.load();

    if (pipeline_todo_.empty() && num_running < max_pipeline_num_) {
        // 创建新流水线，立即执行
        std::shared_ptr<PipelineEngine> new_engine(new PipelineEngine(server_ip_));

        io::YAML_IO default_yaml("./config/mapping.yaml");
        default_yaml.SetValue("map_name", pipeline_name);
        default_yaml.SetValue("data_fetching", "data_url", url);
        std::string new_config_file = "./config/" + pipeline_name + ".yaml";
        default_yaml.Save(new_config_file);

        if (!new_engine->Init(new_config_file)) {
            LOG(ERROR) << "failed to init task: " << pipeline_name;
            return;
        }

        unsigned long pipeline_id = s_pipeline_id++;
        pipelines_.insert({pipeline_id, new_engine});
        pipeline_name_to_id_.insert({pipeline_name, pipeline_id});

        boost::format fmt("Start pipeline %s with id %d.");
        answer_ = (fmt % pipeline_name % pipeline_id).str();
        new_engine->StartPipeline();
        running_pipeline_num_.store(num_running + 1);
    } else {
        // 进入todo
        io::YAML_IO default_yaml("./config/mapping.yaml");
        default_yaml.SetValue("map_name", pipeline_name);
        default_yaml.SetValue("data_fetching", "data_url", url);
        std::string new_config_file = "./config/" + pipeline_name + ".yaml";
        default_yaml.Save(new_config_file);

        answer_ = "Sorry, number of running pipelines reaches max: " + std::to_string(num_running) + "/" +
                  std::to_string(max_pipeline_num_) +
                  +" or todo list not empty, I'll put this pipeline into todo list.";

        unsigned long pipeline_id = s_pipeline_id++;
        PipelineInfo info(pipeline_id, pipeline_name, url);
        pipeline_todo_.insert({pipeline_id, info});
        pipeline_name_to_id_.insert({pipeline_name, pipeline_id});
    }
}

void MappingServer::ShowPipelineReport() {
    std::unique_lock<std::mutex> lock(pipeline_mutex_);
    assert(params_.size() == 1);
    std::string pipeline_name = params_[0];

    if (pipeline_name_to_id_.find(pipeline_name) == pipeline_name_to_id_.end()) {
        answer_ = "pipeline name " + pipeline_name + " does not exist!";
        return;
    }

    auto id = pipeline_name_to_id_[pipeline_name];
    if (pipelines_.find(id) != pipelines_.end()) {
        auto pipeline = pipelines_[id];
        auto result = pipeline->GetResult();
        answer_ = "pipeline " + pipeline_name + " report: \n" + result.report;
    } else if (archived_result_.find(id) != archived_result_.end()) {
        auto result = archived_result_[id];
        answer_ = "pipeline " + pipeline_name + " report: \n" + result.report;
    } else if (pipeline_todo_.find(id) != pipeline_todo_.end()) {
        answer_ = "pipeline " + pipeline_name + " is in TODO list.\n";
    }
}

int MappingServer::ToIntStep(const std::string &step) {
    if (step == "DataFetching") {
        return 1;
    } else if (step == "CheckIn") {
        return 2;
    } else if (step == "Preprocessing") {
        return 3;
    } else if (step == "DRFrontend") {
        return 4;
    } else if (step == "LidarFrontend") {
        return 5;
    } else if (step == "OptimizationStage1") {
        return 6;
    } else if (step == "LoopClosing") {
        return 7;
    } else if (step == "OptimizationStage2") {
        return 8;
    } else if (step == "Validation") {
        return 9;
    } else if (step == "CheckOut") {
        return 10;
    }

    return -1;
}

}  // namespace mapping::app