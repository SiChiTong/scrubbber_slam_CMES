//
// Created by pengguoqi on 2021/1/6.
//

/// 将map.db数据切分成瓦片时的点云数据用于定位仿真调试

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "io/db_io.h"
#include "io/file_io.h"
#include "common/keyframe.h"
#include "common/mapping_point_types.h"

#include <pcl/filters/voxel_grid.h>

DEFINE_string(db_path, "./map.db", "path to db");
DEFINE_string(out_path, "./", "bin output path");

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

    LOG(INFO) << "mapdb : " << FLAGS_db_path << ", out_path : " << FLAGS_out_path;
    DB_IO db_io(FLAGS_db_path);
    std::map<int, SE3> map_id_pose;
    if (!db_io.ReadAllUniqueIdAndPose(map_id_pose)) {
        LOG(ERROR) << "Failed to ReadAllUniqueIdAndPose.";
        std::map<int, SE3>().swap(map_id_pose);
        return -1;
    }
    // 计算点云地图大致边界
    double minx = DBL_MAX, miny = DBL_MAX, maxx = DBL_MIN, maxy = DBL_MIN;
    for (auto id : map_id_pose) {
        auto pos = id.second.translation();
        if (pos[0] > maxx) maxx = pos[0];
        if (pos[1] > maxy) maxy = pos[1];
        if (pos[0] < minx) minx = pos[0];
        if (pos[1] < miny) miny = pos[1];
    }
    
    minx -= 30.0;
    miny -= 30.0;
    maxx += 30.0;
    maxy += 30.0;

    LOG(INFO) << " range: min(" << minx << ", " << miny << ") "
              << "max(" << maxx << ", " << maxx << ")";

    int cols = (int)((maxx - minx) / 50.0) + 1;
    int rows = (int)((maxy - miny) / 50.0) + 1;
    LOG(INFO) << "cols = " << cols << ", rows = " << rows;
    OriginPointInformation origin;
    if (!db_io.ReadOriginPointInformationByDB(origin)) return -1;

    std::string cmd = "mkdir " + FLAGS_out_path + "point_map/";
    LOG(INFO) << "origin : " << origin.map_origin_x << ", " << origin.map_origin_y;
    system(cmd.c_str());
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            double x = minx + (col + 0.5) * 50.0;
            double y = miny + (row + 0.5) * 50.0;
            std::vector<std::shared_ptr<KeyFrame>> keyframes;
            keyframes.clear();
            if (!db_io.ReadDiscretePoseAndCloudByRange(x, y, 100, keyframes)) {
                LOG(ERROR) << " read pose and cloud failed! ";
                std::vector<std::shared_ptr<KeyFrame>>().swap(keyframes);
                return -1;
            }
            if (keyframes.size() < 1) {
                LOG(WARNING) << " distance is too far.";
                continue;
            }
            PointCloudXYZI::Ptr out_points(new PointCloudXYZI());
            for (auto &kf : keyframes) {
                PointCloudType::Ptr points(new PointCloudType());
                if (kf->cloud_->points.empty()) {
                    LOG(ERROR) << " Point cloud is empty! ";
                    std::vector<std::shared_ptr<KeyFrame>>().swap(keyframes);
                    return -2;
                }
                
                Eigen::Matrix<float, 4, 4> pose_eigen;
                auto tem = kf->optimized_pose_stage_2_.rotationMatrix();
                for (unsigned int i = 0; i < 3; ++i) {
                    for (unsigned int j = 0; j < 3; ++j) {
                        pose_eigen(i, j) = tem(i, j);
                    }
                }
                pose_eigen.col(3) << kf->optimized_pose_stage_2_.translation()[0], 
                                     kf->optimized_pose_stage_2_.translation()[1], 
                                     kf->optimized_pose_stage_2_.translation()[2], 1;
                // pose_eigen(0, 3) += origin.map_origin_x;
                // pose_eigen(1, 3) += origin.map_origin_y;
                points = kf->cloud_;
                pcl::transformPointCloud(*points, *points, pose_eigen);
                for (auto &p : *points) {
                    // if (p.x >= (x - 0.5 * 50.0 + origin.map_origin_x) && 
                    //     p.x < (x + 0.5 * 50.0 + origin.map_origin_x) &&
                    //     p.y >= (y - 0.5 * 50.0 + origin.map_origin_y) && 
                    //     p.y < (y + 0.5 * 50.0 + origin.map_origin_y)) {
                    if (p.x >= (x - 0.5 * 50.0 + 0) && 
                        p.x < (x + 0.5 * 50.0 + 0) &&
                        p.y >= (y - 0.5 * 50.0 + 0) && 
                        p.y < (y + 0.5 * 50.0 + 0)) {
                        PointXYZI pp;
                        pp.x = p.x;
                        pp.y = p.y;
                        pp.z = p.z;
                        pp.intensity = p.intensity;
                        out_points->points.emplace_back(pp);
                    }
                }
                out_points->header.stamp = kf->cloud_->header.stamp;
            }
            int pcd_x = static_cast<int>((minx + col * 50.0 + origin.map_origin_x) / 50);
            int pcd_y = static_cast<int>((miny + row * 50.0 + origin.map_origin_y) / 50);
            out_points->height = 1;
            out_points->width = out_points->points.size();
            if (out_points->points.size() < 10) continue;
            pcl::VoxelGrid<PointXYZI> voxel_grid_filter;
            voxel_grid_filter.setLeafSize(0.1, 0.1, 0.1);
            PointCloudXYZI::Ptr pointcloud(new PointCloudXYZI());
            voxel_grid_filter.setInputCloud(out_points);
            voxel_grid_filter.filter(*pointcloud);
            if (pointcloud->points.size() < 10) continue;
            std::string pcd_name = FLAGS_out_path + "point_map/" +
                                   std::to_string(pcd_y) + "_" + 
                                   std::to_string(pcd_x) + "_50.bin" ;
            pcl::io::savePCDFileBinaryCompressed(pcd_name, *pointcloud);
        }
    }

    LOG(INFO) << "done.";
    return 0;
}
