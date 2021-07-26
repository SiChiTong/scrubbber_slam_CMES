//
// Created by gaoxiang on 2020/11/20.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(yaml_path, "./config/mapping", "config yaml");

#include "pipeline/loop_closing/loop_closing.h"

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

    mapping::pipeline::LoopClosing loop_closing(yaml, mapping::pipeline::PipelineContext::RunMode::GEM_EXECUTABLE,
                                                true);
    LOG(INFO) << "initing";
    if (loop_closing.Init() == false) {
        LOG(ERROR) << "failed to init loop closing";
        return -1;
    }

    /// 检查回环
    LOG(INFO) << "detecting loop candidates";
    loop_closing.DetectLoopCandidates();

    /// 保存检查结果
    LOG(INFO) << "saving";
    loop_closing.SaveLoopCandidates();

    LOG(INFO) << "clear";
    loop_closing.Clear();

    LOG(INFO) << "loop closing done.";
    return 0;
}
