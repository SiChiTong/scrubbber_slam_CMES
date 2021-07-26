//
// Created by gaoxiang on 2020/10/29.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/mapping_point_types.h"
#include "io/db_io.h"
#include "io/file_io.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

DEFINE_string(input_pcd, "./data/map.pcd", "input pcd");
DEFINE_string(output_pcd, "./data/remap.pcd", "output pcd");
DEFINE_double(weight, 5.0, "intensity weight");

using namespace mapping::common;
using namespace mapping::io;

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    PointCloudType::Ptr input_cloud(new PointCloudType);
    pcl::io::loadPCDFile(FLAGS_input_pcd, *input_cloud);

    LOG(INFO) << "converting";
    PointCloudType::Ptr output_cloud(new PointCloudType);
    for (auto& pt : input_cloud->points) {
        auto pt_remap = pt;
        double intensity = pt.intensity * FLAGS_weight;
        pt_remap.intensity = intensity > 255 ? 255 : intensity;
        output_cloud->points.emplace_back(pt_remap);
    }

    output_cloud->is_dense = false;
    output_cloud->width = output_cloud->points.size();
    LOG(INFO) << "saving pcd;";
    pcl::io::savePCDFileBinary(FLAGS_output_pcd, *output_cloud);
    LOG(INFO) << "done.";

    return 0;
}