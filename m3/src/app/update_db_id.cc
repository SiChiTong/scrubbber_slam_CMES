//
// Created by gaoxiang on 2020/12/1.
//

/// 刷新DB中的pose id

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "io/db_io.h"
#include "io/file_io.h"

DEFINE_string(db_path, "./map.db", "path to db");
DEFINE_int32(id_inc, 0, "id incremental");

using namespace mapping::common;
using namespace mapping::io;

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    if (PathExists(FLAGS_db_path) == false) {
        LOG(ERROR) << "path " << FLAGS_db_path << " not exist.";
        return -1;
    }

    LOG(INFO) << "working on " << FLAGS_db_path << " by inc = " << FLAGS_id_inc << "...";
    DB_IO db_io(FLAGS_db_path);
    db_io.UpdateIdByInc(FLAGS_id_inc);

    LOG(INFO) << "done.";
    return 0;
}
