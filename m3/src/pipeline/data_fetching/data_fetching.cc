//
// Created by gaoxiang on 19-7-12.
//

#include "pipeline/data_fetching/data_fetching.h"
#include "io/file_io.h"

#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <cstdlib>

namespace mapping {
namespace pipeline {

DataFetching::DataFetching(const io::YAML_IO &yaml_file, RunMode run_mode)
    : yaml_file_(yaml_file), PipelineContext(run_mode) {
    data_url_ = yaml_file.GetValue<std::string>("data_fetching", "data_url");
    local_data_path_ = yaml_file.GetValue<std::string>("data_fetching", "local_data_path");
    user_name_ = yaml_file.GetValue<std::string>("data_fetching", "user_name");
    password_ = yaml_file.GetValue<std::string>("data_fetching", "password");
    map_name_ = yaml_file.GetValue<std::string>("map_name");
    context_name_ = "DataFetching";
}

DataFetching::~DataFetching() { LOG(INFO) << "data fetching deconstructed."; }

bool DataFetching::Init() {
    // create map data folder if not exist
    std::string map_folder;
    if (run_mode_ == RunMode::PIPELINE) {
        map_folder = "/home/idriver/data/" + map_name_;
    } else {
        map_folder = "/home/idriver/work/share/data/" + map_name_;
        // create db folder if not exist
        std::string db_folder = "/home/idriver/work/own/" + map_name_;
        if (!io::PathExists(db_folder)) {
            std::string cmd = "mkdir -p " + db_folder;
            system(cmd.c_str());
        }

        yaml_file_.SetValue<std::string>("local_db_path", db_folder + "/");
    }

    if (!io::PathExists(map_folder)) {
        LogAndReport("文件夹不存在，创建文件夹，位于：" + map_folder);
        std::string cmd = "mkdir -p " + map_folder;
        int ret = system(cmd.c_str());
        if (ret != 0) {
            SetStatus(ContextStatus::FAILED);
            report_ += "DataFetching: create map data folder failed.";
            data_fetching_failed_reason_ = "create map data folder failed.";
            data_fetching_passed_ = 2;
            return false;
        }
    }

    map_folder += "/";
    yaml_file_.SetValue<std::string>("data_fetching", "local_data_path", map_folder);
    local_data_path_ = map_folder;
    return true;
}

bool DataFetching::Start() {
    // 检查原始数据是否存在
    bool skip = false;

    // unrar or unzip
    bool is_rar = data_url_.find(".rar") != std::string::npos;
    bool is_zip = data_url_.find(".zip") != std::string::npos;
    bool is_dir = io::IsDirectory(data_url_);

    if (is_rar) {
        LogAndReport("数据类型为rar包");
    }
    if (is_zip) {
        LogAndReport("数据类型为zip包");
    }
    if (is_dir) {
        LogAndReport("数据类型文件夹");
    }

    if (!is_rar && !is_zip && !is_dir) {
        LOG(ERROR) << "Data fetching only supports rar or zip files!";
        LogAndReport("数据类型不是rar,zip或目录");
        report_ += "数据类型不是rar,zip或目录";
        data_fetching_failed_reason_ = "数据类型不是rar,zip或目录";
        data_fetching_passed_ = 2;
        SetStatus(ContextStatus::FAILED);
        return false;
    }

    std::string cmd;
    if (is_rar && io::PathExists(local_data_path_ + "/data.rar")) {
        LogAndReport("存在历史数据，删除历史数据");
        cmd = "cd " + local_data_path_ + " && rm -rf `ls ./ | grep -v data.rar`";
        system(cmd.c_str());
        skip = true;
    }

    if (is_zip && io::PathExists(local_data_path_ + "/data.zip")) {
        LogAndReport("存在历史数据，删除历史数据");
        cmd = "cd " + local_data_path_ + " && rm -rf `ls ./ | grep -v data.zip`";
        system(cmd.c_str());
        skip = true;
    }

    if (is_dir) {
        skip = true;
    }

    if (!skip) {
        cmd = std::string("wget -c ") + " \"" + data_url_ + "\" --no-check-certificate" +
              " --header=\"User-Agent: Mozilla/5.0 (Macintosh; Intel Mac OS X 10_10_0) AppleWebKit/600.1.17 (KHTML, "
              "like Gecko) Version/8.0 Safari/600.1.17\"";

        if (data_url_.find("https://192.168.2.240/svn") != std::string::npos) {
            // svn 密码
            cmd += " --user " + user_name_ + " --password " + password_;
        } else if (data_url_.find("ftp://192.168.100.93") != std::string::npos) {
            // ftp密码
            cmd += " --user idriver --password idriver";
        }

        if (is_rar) {
            cmd += " -O \"" + local_data_path_ + "/data.rar\"";
        } else if (is_zip) {
            cmd += " -O \"" + local_data_path_ + "/data.zip\"";
        }

        SetStatus(ContextStatus::WORKING);
        LOG(INFO) << "cmd: " << cmd;
        int ret = system(cmd.c_str());

        if (ret == 0) {
            LogAndReport("数据下载：完成");
        } else {
            LogAndReport("数据下载失败");
            data_fetching_failed_reason_ = "下载失败";
            data_fetching_passed_ = 2;
            SetStatus(ContextStatus::FAILED);
            return false;
        }
    } else {
        LogAndReport("发现文件夹下存在数据，跳过下载步骤");
    }

    int ret = 0;
    LOG(INFO) << "decompressing files ... ";
    if (is_rar) {
        cmd = "unrar x -y -inul " + local_data_path_ + "data.rar " + local_data_path_;
        ret = system(cmd.c_str());
        data_size_ = boost::filesystem::file_size(local_data_path_ + "data.rar");
    } else if (is_zip) {
        cmd = "unzip -qq -o " + local_data_path_ + "data.zip -d " + local_data_path_;
        ret = system(cmd.c_str());
        data_size_ = boost::filesystem::file_size(local_data_path_ + "data.zip");
    } else if (is_dir) {
        data_size_ = 0;
        for (const boost::filesystem::directory_entry &f : boost::filesystem::recursive_directory_iterator(data_url_)) {
            if (boost::filesystem::is_regular_file(f.path())) {
                data_size_ += boost::filesystem::file_size(f.path());
            }
        }
    }

    if (is_rar && ret != 0) {
        LogAndReport("解压失败，数据包可能损坏");
        data_fetching_failed_reason_ = "解压失败，数据包可能损坏";
        data_fetching_passed_ = 2;
        SetStatus(ContextStatus::FAILED);
        return false;
    }

    // set local data path
    std::string conf_path;
    if (io::FindFile(local_data_path_, "calibration.launch", conf_path) ||
        io::FindFile(local_data_path_, "velodyne.launch", conf_path) ||
        io::FindFile(local_data_path_, "localization_map_collection.yaml", conf_path)) {
        LogAndReport("本地数据位于：" + conf_path);
        yaml_file_.SetValue("data_fetching", "local_data_path", conf_path + "/");
    } else {
        LogAndReport("无法确定本地数据位置");
        data_fetching_failed_reason_ = "无法确定本地数据位置";
        data_fetching_passed_ = 2;
        SetStatus(ContextStatus::FAILED);
        return false;
    }

    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        yaml_file_.SetValue("task_info", "failed_reason", data_fetching_failed_reason_);
        yaml_file_.SetValue("task_info", "total_bag_size", data_size_);
    }

    yaml_file_.Save();
    SetStatus(ContextStatus::SUCCEED);
    return true;
}

bool DataFetching::Save() { return true; }

bool DataFetching::Load() {
    std::string conf_path;
    // check if calibration can be found in local data path
    if ((!io::FindFile(local_data_path_, "calibration.launch", conf_path)) &&
        (!io::FindFile(local_data_path_, "velodyne.launch", conf_path))) {
        LogAndReport("未找到launch文件");
        data_fetching_failed_reason_ = "未找到launch文件";
        data_fetching_passed_ = 2;
        return false;
    }

    report_ += "DataFetching: loaded.\n";
    return true;
}

bool DataFetching::FillTaskInfo(TaskInfo &info) {
    info.total_bag_size = data_size_;
    info.failed_reason = data_fetching_failed_reason_;
    info.checkin_passed = data_fetching_passed_;
    return true;
}

}  // namespace pipeline
}  // namespace mapping