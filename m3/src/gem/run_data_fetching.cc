//
// Created by gaoxiang on 2020/11/20.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(yaml_path, "./config/mapping", "config yaml");

#include "pipeline/data_fetching/data_fetching.h"

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    if (FLAGS_yaml_path.empty()) {
        LOG(ERROR) << "config yaml is empty";
        return -1;
    }

    mapping::io::YAML_IO yaml(FLAGS_yaml_path);
    if (yaml.IsOpened() == false) {
        LOG(ERROR) << "failed to open yaml";
        return -1;
    }

    mapping::pipeline::DataFetching data_fetching(yaml, mapping::pipeline::PipelineContext::RunMode::GEM_EXECUTABLE);
    if (data_fetching.Init() == false) {
        LOG(ERROR) << "failed to init data fetching";
        return -1;
    }

    if (data_fetching.Start() == false) {
        LOG(ERROR) << "failed at data fetching";
        return -1;
    }

    LOG(INFO) << "data fetching done.";
    return 0;
}
