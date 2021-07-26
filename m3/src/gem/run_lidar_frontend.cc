//
// Created by gaoxiang on 2020/11/20.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(yaml_path, "./config/mapping", "config yaml");
DEFINE_uint64(start_keyframe, 0, "start keyframe");
DEFINE_uint64(end_keyframe, 0, "end keyframe");

#include "pipeline/lidar_frontend/lidar_frontend.h"

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    if (FLAGS_yaml_path.empty()) {
        LOG(ERROR) << "config is empty";
        return -1;
    }

    mapping::io::YAML_IO yaml(FLAGS_yaml_path);
    if (yaml.IsOpened() == false) {
        LOG(ERROR) << "failed to open yaml";
        return -1;
    }

    if (FLAGS_end_keyframe < FLAGS_start_keyframe) {
        LOG(INFO) << "end is smaller than start";
        return -1;
    }

    mapping::pipeline::LidarFrontend lidar_frontend(yaml, mapping::pipeline::PipelineContext::RunMode::GEM_EXECUTABLE,
                                                    FLAGS_start_keyframe, FLAGS_end_keyframe);
    if (lidar_frontend.Init() == false) {
        LOG(ERROR) << "failed to init lidar_frontend";
        return -1;
    }

    if (lidar_frontend.Start() == false) {
        LOG(ERROR) << "failed at lidar_frontend";
        return -1;
    }

    LOG(INFO) << "lidar_frontend from " << FLAGS_start_keyframe << " to " << FLAGS_end_keyframe << " done.";
    return 0;
}
