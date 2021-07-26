//
// Created by gaoxiang on 2020/11/27.
//

/// 将多个DB文件合并成一个

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "io/db_io.h"
#include "io/file_io.h"
#include "io/yaml_io.h"

DEFINE_string(config_yaml, "./config/mapping.yaml", "path to config yaml");

using namespace mapping::common;
using namespace mapping::io;

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    if (FLAGS_config_yaml.empty()) {
        LOG(ERROR) << "usage: dump_db_data --db_path=[path to db] --dump_data_path=[path to data;";
        return -1;
    }

    YAML_IO yaml_io(FLAGS_config_yaml);
    if (yaml_io.IsOpened() == false) {
        LOG(ERROR) << "cannot open yaml:" << FLAGS_config_yaml;
        return -1;
    }

    std::string local_data_path = yaml_io.GetValue<std::string>("data_fetching", "local_data_path");
    std::string local_db_path = yaml_io.GetValue<std::string>("local_db_path");

    std::map<IdType, std::vector<KFPtr>> all_keyframes;
    LoadKeyframes(local_data_path + "keyframes.txt", all_keyframes);

    std::map<IdType, std::string> id_to_db;
    for (auto& kfp : all_keyframes) {
        if (kfp.second[0]->bag_type_ == mapping::common::KeyFrameBagType::MAPPING_BAGS) {
            id_to_db.insert({kfp.first, local_db_path + "map_" + std::to_string(kfp.first) + ".db"});
        }
    }

    if (id_to_db.empty()) {
        LOG(WARNING) << "cannot locate any db in " << local_db_path;
        return -1;
    }

    if (id_to_db.size() == 1) {
        // 只有一个，无需合并
        IdType id = id_to_db.begin()->first;
        system(std::string("mv " + local_db_path + "map_" + std::to_string(id) + ".db " + local_db_path + "map.db")
                   .c_str());
        return 0;
    }

    // load keyframe id and pose from keyframe.txt
    int min_id = id_to_db.begin()->first;
    LOG(INFO) << "merging to " << min_id;
    std::map<IdType, std::shared_ptr<DB_IO>> id_to_dbio;

    // open the dbs
    for (auto& idp : id_to_db) {
        id_to_dbio.insert({idp.first, std::make_shared<DB_IO>(idp.second)});
    }

    // 将所有的db都合并到最小的ID号，然后move
    DB_IO db_io(local_db_path + "map_" + std::to_string(min_id) + ".db");

    auto iter = id_to_db.begin();
    iter++;
    for (; iter != id_to_db.end(); iter++) {
        LOG(INFO) << "merging trajectory " << iter->first;
        auto db_io_other = id_to_dbio[iter->first];
        std::vector<std::shared_ptr<KeyFrame>> keyframes;
        std::vector<int> frames_read_fail_id;
        db_io_other->ReadAllKF(keyframes, frames_read_fail_id);
        db_io.WritePoseAndCloudToDB(keyframes);
    }

    system(std::string("mv " + id_to_db.begin()->second + " " + local_db_path + "map.db").c_str());
    std::for_each(id_to_db.begin(), id_to_db.end(), [](const auto& idp) {
        if (PathExists(idp.second)) {
            system(("rm -rf " + idp.second).c_str());
        }
    });

    LOG(INFO) << "done.";
    return 0;
}
