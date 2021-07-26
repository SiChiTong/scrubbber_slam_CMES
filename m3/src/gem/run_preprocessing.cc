//
// Created by idriver on 2020/11/23.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

// DEFINE_string(yaml_path, "./config/mapping", "config yaml");
DEFINE_string(yaml_path, "/home/idriver/workspace/mapping/mapping3.0/config/北京亦庄.yaml", "config yaml");

#include "io/file_io.h"
#include "pipeline/preprocessing/preprocessing.h"

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

    mapping::pipeline::Preprocessing preproc(yaml, mapping::pipeline::PipelineContext::RunMode::GEM_EXECUTABLE);
    if (preproc.Init() == false) {
        LOG(ERROR) << "failed to init preprocessing";
        return -1;
    }

    if (preproc.Start() == false) {
        LOG(ERROR) << "failed at preprocessing";
        return -1;
    }

    LOG(INFO) << "preprocessing done.";
    return 0;
}