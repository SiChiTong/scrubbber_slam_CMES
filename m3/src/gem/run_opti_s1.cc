//
// Created by gaoxiang on 2020/11/20.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(yaml_path, "./config/mapping", "config yaml");

#include "pipeline/optimization_s1/optimization_s1.h"

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

    mapping::pipeline::OptimizationStage1 opti_s1(yaml, mapping::pipeline::PipelineContext::RunMode::GEM_EXECUTABLE);
    if (opti_s1.Init() == false) {
        LOG(ERROR) << "failed to init opti s1";
        return -1;
    }

    if (opti_s1.Start() == false) {
        LOG(ERROR) << "failed at opti s1";
        return -1;
    }

    LOG(INFO) << "opti s1 done.";
    return 0;
}
