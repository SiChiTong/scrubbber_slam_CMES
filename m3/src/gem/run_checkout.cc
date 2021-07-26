//
// Created by gaoxiang on 2020/11/20.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(yaml_path, "./config/mapping", "config yaml");

#include "pipeline/check_out/check_out.h"

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

    mapping::pipeline::CheckOut checkout(yaml, mapping::pipeline::PipelineContext::RunMode::GEM_EXECUTABLE);
    if (checkout.Init() == false) {
        LOG(ERROR) << "failed to init opti s2";
        return -1;
    }

    if (checkout.Start() == false) {
        LOG(ERROR) << "failed at opti s2";
        return -1;
    }

    LOG(INFO) << "opti s2 done.";
    return 0;
}
