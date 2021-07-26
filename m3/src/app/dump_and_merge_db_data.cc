//
// Created by gaoxiang on 2020/10/29.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/mapping_point_types.h"
#include "io/db_io.h"
#include "io/file_io.h"

#include <pcl/filters/voxel_grid.h>

DEFINE_string(db_path, "", "path_db");
DEFINE_string(dump_data_path, "./data/", "dir to dump");
DEFINE_double(resolution, 0.02, "voxel resolution");
DEFINE_int32(min_ring, 3, "minimal ring");
DEFINE_int32(max_ring, 8, "maximal ring");

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
    LOG(INFO) << "loading keyframes";
    if (db_io.ReadAllKF(keyframes, failed_id) == false) {
        LOG(ERROR) << "failed when reading db";
        return -1;
    }

    // dump kf cloud and merge
    LOG(INFO) << "merging";
    PointCloudType::Ptr opt_map_points_filtered(new PointCloudType());
    PointCloudPtr global_cloud(new PointCloudType);

    pcl::VoxelGrid<PointType> voxel_grid_filter;
    float resolution = FLAGS_resolution;
    voxel_grid_filter.setLeafSize(resolution, resolution, resolution);

    int cnt = 0;
    for (auto& kf : keyframes) {
        SE3 pose = kf->optimized_pose_stage_2_;
        PointCloudPtr cloud_trans(new PointCloudType);
        pcl::transformPointCloud(*kf->cloud_, *cloud_trans, pose.matrix().cast<float>());

        PointCloudPtr cloud_trans_filtered(new PointCloudType);
        for (auto& pt : cloud_trans->points) {
            if (pt.ring >= FLAGS_min_ring && pt.ring < FLAGS_max_ring) {
                cloud_trans_filtered->points.push_back(pt);
            }
        }
        cloud_trans_filtered->width = cloud_trans_filtered->points.size();
        cloud_trans_filtered->is_dense = false;

        PointCloudPtr kf_cloud_voxeled(new PointCloudType);
        voxel_grid_filter.setInputCloud(cloud_trans_filtered);
        voxel_grid_filter.filter(*kf_cloud_voxeled);

        *global_cloud += *kf_cloud_voxeled;
        kf->UnloadCloud();

        LOG(INFO) << "merging " << cnt << " in " << keyframes.size();
        cnt++;
    }

    if (global_cloud->empty()) {
        return 1;
    }

    LOG(INFO) << "saving pcd";
    pcl::io::savePCDFile(FLAGS_dump_data_path + "/map.pcd", *global_cloud, true);
    LOG(INFO) << "point cloud saved at " << FLAGS_dump_data_path << ", pts: " << global_cloud->points.size();
    LOG(INFO) << "done.";

    return 0;
}