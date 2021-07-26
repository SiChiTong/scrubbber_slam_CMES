//
// Created by gaoxiang on 2020/11/20.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(yaml_path, "./config/mapping", "config yaml");
DEFINE_uint64(start_id, 0, "start id");
DEFINE_uint64(end_id, 0, "end id");

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

    if (FLAGS_end_id < FLAGS_start_id) {
        LOG(ERROR) << "end id is smaller than start: " << FLAGS_start_id << ", " << FLAGS_end_id;
    }

    mapping::io::YAML_IO yaml(FLAGS_yaml_path);
    if (yaml.IsOpened() == false) {
        LOG(ERROR) << "failed to open yaml";
        return -1;
    }

    mapping::pipeline::LoopClosing loop_closing(yaml, mapping::pipeline::PipelineContext::RunMode::GEM_EXECUTABLE,
                                                false);
    loop_closing.SetStartEndIdx(FLAGS_start_id, FLAGS_end_id);

    if (loop_closing.Init() == false) {
        LOG(ERROR) << "failed to init loop closing";
        return -1;
    }

    LOG(INFO) << "computing loop closing";
    loop_closing.ComputeRelativeMotionMT();

    LOG(INFO) << "saving loop candidates";
    loop_closing.SaveLoopCandidates(false);

    LOG(INFO) << "clear";
    loop_closing.Clear();

    LOG(INFO) << "loop closing done.";
    return 0;
}
