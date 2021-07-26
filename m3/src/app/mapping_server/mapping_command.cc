//
// Created by gaoxiang on 19-7-15.
//
#include <glog/logging.h>

#include "app/mapping_server/mapping_client.h"
#include "app/mapping_server/mapping_server.h"
#include "common/version.h"

using namespace mapping::app;

/// 建图服务的命令行界面
void Help() {
    LOG(INFO) << "Usage: " << std::endl
              << "mapping server #start mapping server " << std::endl
              << "mapping list #Show pipelines " << std::endl
              << "mapping start map_name data_dir #Start a pipeline using [map_name] and [data_dir] " << std::endl
              << "mapping merge input_map_name data.xxx target_map_name # merge [target_map_name] and [input_map_name]" << std::endl
              << "mapping show map_name #Show that status of pipeline [map_name]" << std::endl
              << "mapping restart map_name step #Restart a pipeline [map_name] from [step], step is an "
                 "int value from 1-9"
              << std::endl
              << "mapping close #Quit map server" << std::endl;
}

// global variables
std::string command;
std::vector<std::string> params;

/// 检查输入参数
bool CheckCommandLine(int argc, char **argv);

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    if (!CheckCommandLine(argc, argv)) {
        Help();
        return 1;
    }

    // Do command
    if (command == "server") {
        MappingServer server;
        server.Start();
        LOG(WARNING) << "Local server has been launched.";
        server.Spin();  // 阻塞
    } else if (command == "version") {
        LOG(INFO) << "Your are running mapping version " << mapping::common::mapping_major_version << "."
                  << mapping::common::mapping_minor_version << "." << mapping::common::mapping_revision << std::endl;
    } else {
        MappingClient client;
        for (auto &p : params) {
            command += " " + p;
        }
        client.DoCommand(command);  // 使用客户端执行命令
    }

    return 0;
}

bool CheckCommandLine(int argc, char **argv) {
    if (argc < 2) return false;
    command = argv[1];
    if (command == "server") {
        return true;
    } else if (command == "list") {
        return true;
    } else if (command == "start") {
        // 需要两个参数
        if (argc != 4) {
            LOG(ERROR) << "parameter number is not right: " << argc;
            return false;
        }
        params.emplace_back(argv[2]);
        params.emplace_back(argv[3]);
        return true;
    } else if (command == "show") {
        // 需要1个参数
        if (argc != 3) {
            LOG(ERROR) << "parameter number is not right: " << argc;
            return false;
        }
        params.emplace_back(argv[2]);
        return true;
    } else if (command == "close") {
        return true;
    } else if (command == "restart") {
        if (argc != 4) {
            LOG(ERROR) << "parameter number is not right: " << argc;
            return false;
        }
        if (atoi(argv[3]) == 0) {
            LOG(ERROR) << "step must be a integer from 1 to 5";
            return false;
        }
        params.emplace_back(argv[2]);
        params.emplace_back(argv[3]);
        return true;
    } else if (command == "version") {
        return true;
    } else if (command == "merge") {
        // 需要三个参数
        if (argc != 5) {
            LOG(ERROR) << "parameter number is not right: " << argc;
            LOG(INFO) << " correct command: 'bin/mapping merge input_map_name data.xxx target_map_name'";
            return false;
        }
        params.emplace_back(argv[2]);
        params.emplace_back(argv[3]);
        params.emplace_back(argv[4]);
        return true;
    } else {
        LOG(ERROR) << "Unknown command " << command;
        return false;
    }
}
