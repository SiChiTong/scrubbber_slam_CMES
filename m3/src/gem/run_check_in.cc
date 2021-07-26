//
// Created by idriver on 2020/11/23.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(yaml_path, "./config/mapping", "config yaml");

#include "io/file_io.h"
#include "pipeline/check_in/check_in.h"

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    if (FLAGS_yaml_path.empty()) {
        LOG(ERROR) << "path or name or url is empty";
        return -1;
    }

    mapping::io::YAML_IO yaml(FLAGS_yaml_path);
    if (yaml.IsOpened() == false) {
        LOG(ERROR) << "failed to open yaml";
        return -1;
    }

    mapping::pipeline::CheckIn check_in(yaml, mapping::pipeline::PipelineContext::RunMode::GEM_EXECUTABLE);
    if (check_in.Init() == false) {
        LOG(ERROR) << "failed to init check in";
        return -1;
    }

    if (check_in.Start() == false) {
        LOG(ERROR) << "failed at check in";
        return -1;
    }

    LOG(INFO) << "check in done.";
    return 0;
}