//
// Created by idriver on 2021/2/1.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(yaml_path, "./config/mapping", "config yaml");

#include "pipeline/merge_map/merge_map.h"

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

    mapping::pipeline::MergeMap merge_map(yaml, mapping::pipeline::PipelineContext::RunMode::GEM_EXECUTABLE);
    if (merge_map.Init() == false) {
        LOG(ERROR) << "failed to init merge map";
        return -1;
    }

    if (merge_map.Start() == false) {
        LOG(ERROR) << "failed at merge map";
        return -1;
    }

    LOG(INFO) << "merge map done.";
    return 0;
}
