//
// Created by gaoxiang on 2020/10/29.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "io/db_io.h"
#include "io/file_io.h"

DEFINE_string(db_path, "", "path_db");
DEFINE_string(dump_data_path, "./data/", "dir to dump");

using namespace mapping::common;
using namespace mapping::io;

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    if (FLAGS_db_path.empty() || FLAGS_dump_data_path.empty()) {
        LOG(ERROR) << "usage: dump_db_data --db_path=[path to db] --dump_data_path=[path to data;";
        return -1;
    }

    DB_IO db_io(FLAGS_db_path);
    std::vector<std::shared_ptr<KeyFrame>> keyframes;
    std::vector<int> failed_id;
    if (db_io.ReadAllKF(keyframes, failed_id) == false) {
        LOG(ERROR) << "failed when reading db";
        return -1;
    }

    // dump kf pose and cloud
    std::map<IdType, std::vector<std::shared_ptr<KeyFrame>>> keyframes_map;
    for (auto& kf : keyframes) {
        if (keyframes_map.find(kf->trajectory_id_) == keyframes_map.end()) {
            keyframes_map.insert({kf->trajectory_id_, {kf}});
        } else {
            keyframes_map[kf->trajectory_id_].emplace_back(kf);
        }

        pcl::io::savePCDFileBinary(FLAGS_dump_data_path + "/" + std::to_string(kf->id_) + ".pcd", *kf->cloud_);
    }

    SaveKeyframeSinglePath(FLAGS_dump_data_path + "/poses.txt", keyframes_map, SaveKeyframePathType::OPTI_PATH_STAGE_2);
    LOG(INFO) << "done.";

    return 0;
}