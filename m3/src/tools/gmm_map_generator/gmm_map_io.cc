//
// Created by pengguoqi on 19-8-24.
//

#include "tools/gmm_map_generator/gmm_map_io.h"
#include <glog/logging.h>
#include "io/db_io.h"

#include <pcl/common/transforms.h>

namespace mapping::tools {

void GmmBaseIO::SetResolution(float resolution) { resolution_ = resolution; }

int GmmBaseIO::LoadRawMap(std::string filename, common::PointCloudXYZI::Ptr &cloud) {
    double minz = -100.0;
    double maxz = 100.0;

    common::PointCloudXYZI::Ptr all(new common::PointCloudXYZI());
    if (!GetPcd(filename, all, minz, maxz)) {
        LOG(ERROR) << "failed to get pcd : " << filename;
        return -1;
    }
    cloud = all;
    all = nullptr;
    return 0;
}

int GmmBaseIO::LoadRawMap(std::string filename) {
    std::shared_ptr<io::DB_IO> db_io = std::make_shared<io::DB_IO>(filename);
    if (!db_io->ReadAllUniqueIdAndPose(map_id_pose_)) {
        LOG(ERROR) << "Failed to ReadAllUniqueIdAndPose.";
        std::map<int, SE3>().swap(map_id_pose_);
        db_io = nullptr;
        return -1;
    }
    map_range_ = ComputeRange();
    return 0;
}

bool GmmBaseIO::GetPcd(std::string filename, common::PointCloudXYZI::Ptr &all, double minz, double maxz) {
    std::shared_ptr<io::DB_IO> db_io = std::make_shared<io::DB_IO>(filename);
    std::vector<int> frames_read_fail_id;
    std::vector<std::shared_ptr<common::KeyFrame>> keyframes;
    if (!db_io->ReadAllKF(keyframes, frames_read_fail_id)) {
        LOG(ERROR) << "Failed to ReadPoseAndCloudByDB.";
        std::vector<int>().swap(frames_read_fail_id);
        std::vector<std::shared_ptr<common::KeyFrame>>().swap(keyframes);
        db_io = nullptr;
        return false;
    }
    for (auto &kf : keyframes) {
        common::PointCloudXYZI::Ptr out_points(new common::PointCloudXYZI());
        CutZ(kf->cloud_, out_points, minz, maxz);
        TransformPcd(out_points, kf->optimized_pose_stage_2_);
        *all += *out_points;
    }
    //    pcl::io::savePCDFileASCII("/home/idriver/test.pcd", *all);
    std::vector<int>().swap(frames_read_fail_id);
    std::vector<std::shared_ptr<common::KeyFrame>>().swap(keyframes);

    db_io = nullptr;
    return true;
}

void GmmBaseIO::CutZ(common::PointCloudType::Ptr pcd, common::PointCloudXYZI::Ptr &out, double min, double max) {
    if (pcd->points.size() < 1) {
        return;
    }
    for (auto &point : pcd->points) {
        common::PointXYZI out_point;
        out_point.x = point.x;
        out_point.y = point.y;
        out_point.intensity = point.intensity;
        double z = point.z;
        if (z > min && z < max) {
            out_point.z = point.z;
            out->points.push_back(out_point);
        }
    }
}

int GmmBaseIO::GetZ(const float r, int &rows, int &cols, std::vector<short int> &gs, const int &type,
                    const std::string &binfile) {
    common::PointCloudXYZ::Ptr pxyz(new common::PointCloudXYZ());
    pcl::KdTreeFLANN<common::PointXYZ> kdtree;
    std::vector<float> groud;
    size_t size = map_id_pose_.size();
    if (size < 1) return -1;
    pxyz->reserve(size);
    groud.reserve(size);
    rows = (map_range_.maxy - map_range_.miny) / r + 1;
    cols = (map_range_.maxx - map_range_.minx) / r + 1;
    for (auto &pose : map_id_pose_) {
        common::PointXYZ tem;

        tem.x = pose.second.translation()[0] - map_range_.minx;
        tem.y = pose.second.translation()[1] - map_range_.miny;
        tem.z = pose.second.translation()[2];

        pxyz->push_back(tem);
        groud.push_back(pose.second.translation()[2]);
    }
    if (1 == type) {
        LOG(INFO) << "map size: " << pxyz->size();
        if (pcl::io::savePCDFileBinary(binfile, *pxyz) < 0) return -1;
        else return 0;
    } else {
        kdtree.setInputCloud(pxyz);
        gs.reserve(rows * cols);
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                float z;
                std::vector<int> index(1);
                std::vector<float> dis(1);
                common::PointXYZ p;
                p.x = j * r;
                p.y = i * r;
                p.z = 0;
                if (kdtree.nearestKSearch(p, 1, index, dis) > 0) {
                    z = groud[index[0]];
                    gs[i * cols + j] = To16(z * 100);
                } else {
                    gs[i * cols + j] = std::numeric_limits<short int>::max();
                }
            }
        }
        LOG(INFO) << " rows : " << rows << " cols : " << cols;
        return 0;
    }
}

int GmmBaseIO::HandlingPCD(const std::string &filename) {
    io::DBRW db;
    if (!db.OpenDB(filename.c_str())) {
        LOG(ERROR) << " open map database failed ";
        return -1;
    }
    // common::PointCloudXYZI::Ptr all(new common::PointCloudXYZI());
    for (auto id : map_id_pose_) {
        common::PointCloudXYZI::Ptr out_points(new common::PointCloudXYZI());
        /*int ret_length = 0;
        void* p_data = NULL;
        DBPose pose;
        int ver = 0;
        if (db.ReadRawLaserData(id.first, pose, p_data, ret_length, ver)) {
            common::PointXYZI* pcl_point = (common::PointXYZI*)p_data;
            int point_count = ret_length / sizeof(common::PointXYZI);
            out_points->reserve(point_count);
            for (int i = 0; i < point_count; i++) {
                out_points->push_back(*pcl_point++);
            }
            TransformPcd2(out_points, pose);
        }*/
        std::shared_ptr<io::DB_IO> db_io = std::make_shared<io::DB_IO>(filename);

        std::shared_ptr<common::KeyFrame> kf = std::make_shared<common::KeyFrame>();
        if (db_io->ReadSingleKF(id.first, kf)) {
            TransformPointCloud(kf->cloud_, out_points);
            TransformPcd2(out_points, kf->optimized_pose_stage_2_);
        } else {
            LOG(ERROR) << " read smiple pointscloud error!";
            continue;
        }

        // *all += *out_points;

        MapPointsf points;
        points.reserve(out_points->size());
        for (auto &p : *out_points) {
            MapPointf pp;
            pp.x = p.x;
            pp.y = p.y;
            pp.z = p.z;
            pp.r = p.intensity;
            points.emplace_back(pp);
        }
        auto ps = VoxelFilter(points);
        AddPointsToHashCells(ps);
    }
    // pcl::io::savePCDFileASCII("/home/idriver-pgq/qinghua.pcd", *all);

    db.CloseDB();
    return 0;
}

int GmmBaseIO::HandlingPCD(const std::string &filename, const TilesPointF &center, const float &size) {
    std::shared_ptr<io::DB_IO> db_io = std::make_shared<io::DB_IO>(filename);
    std::vector<std::shared_ptr<common::KeyFrame>> keyframes;
    keyframes.clear();

    // if (!db_io->ReadDiscretePoseAndCloudByRange(center.x, center.y, size + map_range_.r, keyframes)) {
    //     LOG(ERROR) << " read pose and cloud failed! ";
    //     std::vector<std::shared_ptr<common::KeyFrame>>().swap(keyframes);
    //     return -1;
    // }

    if (!db_io->ReadDiscretePoseByRange(center.x, center.y, size + map_range_.r, keyframes)) {
        LOG(ERROR) << " read pose failed! ";
        std::vector<std::shared_ptr<common::KeyFrame>>().swap(keyframes);
        return -1;
    }

    if (keyframes.size() < 1) {
        LOG(WARNING) << " distance is too far.";
        return 1;
    }

    LOG(INFO) << "keyframes size: " << keyframes.size() << " tiles_minx: " << center.x - 0.5 * size - map_range_.minx
              << " tiles_maxx: " << center.x + 0.5 * size - map_range_.minx
              << " tiles_miny: " << center.y - 0.5 * size - map_range_.miny
              << " tiles_maxy: " << center.y + 0.5 * size - map_range_.miny;
    for (auto &kf : keyframes) {
        common::PointCloudXYZI::Ptr out_points(new common::PointCloudXYZI());
        std::shared_ptr<io::DB_IO> db_io = std::make_shared<io::DB_IO>(filename);
        std::shared_ptr<common::KeyFrame> keyframe = std::make_shared<common::KeyFrame>();
        if (db_io->ReadSingleKF(kf->id_, keyframe)) {
            kf->cloud_ = keyframe->cloud_;
        } else {
            LOG(ERROR) << " read smiple pointscloud error!";
            continue;
        }

        if (kf->cloud_->points.empty()) {
            LOG(ERROR) << " Point cloud is empty! ";
            std::vector<std::shared_ptr<common::KeyFrame>>().swap(keyframes);
            return -2;
        }

        TransformPointCloud(kf->cloud_, out_points);
        TransformPcd2(out_points, kf->optimized_pose_stage_2_);
        MapPointsf points;
        points.clear();
        for (auto &p : *out_points) {
            if (p.x >= (center.x - 0.5 * size - map_range_.minx) && p.x < (center.x + 0.5 * size - map_range_.minx) &&
                p.y >= (center.y - 0.5 * size - map_range_.miny) && p.y < (center.y + 0.5 * size - map_range_.miny)) {
                MapPointf pp;
                pp.x = p.x;
                pp.y = p.y;
                pp.z = p.z;
                pp.r = p.intensity;
                points.emplace_back(pp);
            }
        }
        MapPointsf ps = VoxelFilter(points);
        AddPointsToHashCells(ps);
    }
    if (cells_.size() < 1) {
        LOG(WARNING) << " distance is too far(cells).";
        return 1;
    }
    return 0;
}

void GmmBaseIO::AddPointsToHashCells(MapPointsf &ps) {
    for (auto& p : ps) {
      unsigned long long id = 0;
      if (p.x < 0 || p.y < 0) continue;
      ((unsigned int *)&id)[0] = (unsigned int)(p.y / resolution_);
      ((unsigned int *)&id)[1] = (unsigned int)(p.x / resolution_);
      cells_[id].emplace_back(p.z, p.r);
    }
}

MapRange GmmBaseIO::ComputeRange() {
    MapRange range;
    for (auto id : map_id_pose_) {
        auto pos = id.second.translation();
        if (pos[0] > range.maxx) range.maxx = pos[0];
        if (pos[1] > range.maxy) range.maxy = pos[1];
        if (pos[0] < range.minx) range.minx = pos[0];
        if (pos[1] < range.miny) range.miny = pos[1];
        if (pos[2] > range.maxz) range.maxz = pos[2];
        if (pos[2] < range.minz) range.minz = pos[2];
    }
    range.minx -= range.r;
    range.miny -= range.r;
    range.maxx += range.r;
    range.maxy += range.r;
    LOG(INFO) << " range: min(" << range.minx << ", " << range.miny << ") "
              << "max(" << range.maxx << ", " << range.maxx << ")";
    return range;
}

void GmmBaseIO::TransformPointCloud(common::PointCloudType::Ptr src, common::PointCloudXYZI::Ptr &out) {
    out->clear();
    for (const auto &point : src->points) {
        common::PointXYZI new_point;
        new_point.x = point.x;
        new_point.y = point.y;
        new_point.z = point.z;
        new_point.intensity = point.intensity;
        out->points.push_back(new_point);
    }
    out->header.stamp = src->header.stamp;
    out->height = 1;
    out->width = out->points.size();
}

void GmmBaseIO::TransformPcd(common::PointCloudXYZI::Ptr &src, SE3 &pose) {
    Eigen::Matrix<float, 4, 4> pose_eigen;

    auto tem = pose.rotationMatrix();
    for (unsigned int i = 0; i < 3; ++i)
        for (unsigned int j = 0; j < 3; ++j) pose_eigen(i, j) = tem(i, j);
    //    pose_eigen.col(3) << pose.pos.x, pose.pos.y, pose.pos.z, 1;
    pose_eigen.col(3) << pose.translation()[0], pose.translation()[1], pose.translation()[2], 1;

    pcl::transformPointCloud(*src, *src, pose_eigen);
}

void GmmBaseIO::TransformPcd2(common::PointCloudXYZI::Ptr &src, SE3 &pose) {
    Eigen::Matrix<float, 4, 4> pose_eigen;
    //    Eigen::Quaternion<float> output_quaternion(pose.quat.w, pose.quat.x,
    //                                               pose.quat.y, pose.quat.z);
    //    auto tem = output_quaternion.toRotationMatrix();
    auto tem = pose.rotationMatrix();
    for (unsigned int i = 0; i < 3; ++i)
        for (unsigned int j = 0; j < 3; ++j) pose_eigen(i, j) = tem(i, j);
    //    pose_eigen.col(3) << pose.pos.x, pose.pos.y, pose.pos.z, 1;
    pose_eigen.col(3) << pose.translation()[0], pose.translation()[1], pose.translation()[2], 1;
    pose_eigen(0, 3) -= map_range_.minx;
    pose_eigen(1, 3) -= map_range_.miny;
    pcl::transformPointCloud(*src, *src, pose_eigen);
}

MapPointsf GmmBaseIO::VoxelFilter(const MapPointsf &points) {
    MapPointsf results;
    std::unordered_set<std::bitset<3 * 32>> voxel_set;
    int size = points.size();
    for (int i = 0; i < size; ++i) {
        auto &point = points[i];
        auto it_inserted = voxel_set.insert(IndexToKey(GetCellIndex(point)));
        if (it_inserted.second) {
            results.push_back(point);
        }
    }
    return results;
}

std::bitset<3 * 32> GmmBaseIO::IndexToKey(const Eigen::Array3i &index) {
    std::bitset<3 * 32> k_0(static_cast<uint32_t>(index[0]));
    std::bitset<3 * 32> k_1(static_cast<uint32_t>(index[1]));
    std::bitset<3 * 32> k_2(static_cast<uint32_t>(index[2]));
    return (k_0 << 2 * 32) | (k_1 << 1 * 32) | k_2;
}

Eigen::Array3i GmmBaseIO::GetCellIndex(const MapPointf &point) const {
    return Eigen::Array3i(point.x / resolution_, point.y / resolution_, point.z / resolution_);
}

int GmmBaseIO::BinFile2EigenMatrix(std::string filename, MapT &map) {
    size_t size = map.rows() * map.cols() * sizeof(MapCell<short int>);
    std::ifstream is;
    is.open(filename.c_str(), std::ios::binary);
    if (!is.is_open()) {
        LOG(ERROR) << "failed to open : " << filename;
        return -1;
    }
    is.seekg(0, is.end);
    if (size != is.tellg()) {
        LOG(ERROR) << filename << " file size " << is.tellg() << "!=" << size;
        is.close();
        return -1;
    }
    is.seekg(0, is.beg);
    is.read((char *)map.data(), size);
    if (is.bad()) {
        is.close();
        LOG(ERROR) << filename << " is bad.";
        return -1;
    }
    is.close();
    return 0;
}

int GmmBaseIO::ReadBin(std::string filename, int rc, MapTa &mapa, MapTr &mapr) {
    if (mapa.rows() != mapr.rows()) return -1;
    if (mapa.cols() != mapr.cols()) return -1;
    MapT bin(1, rc);
    int r = BinFile2EigenMatrix(filename, bin);
    if (r < 0) return r;
    for (size_t i = 0; i < mapa.rows(); ++i) {
        for (size_t j = 0; j < mapa.cols(); ++j) {
            mapa(i, j).w1 = 0;
            mapa(i, j).w2 = 0;
            mapr(i, j).w1 = 0;
            mapr(i, j).w2 = 0;
            mapa(i, j).s1 = 1000;
            mapa(i, j).s2 = 1000;
            mapr(i, j).s1 = 200;
            mapr(i, j).s2 = 200;
        }
    }
    for (size_t k = 0; k < rc; ++k) {
        int i = bin(0, k).row;
        int j = bin(0, k).col;
        mapa(i, j) = bin(0, k).cella;
        mapr(i, j) = bin(0, k).cellr;
        if (mapa(i, j).s1 <= 5) mapa(i, j).s1 = 5;
        if (mapa(i, j).s2 <= 5) mapa(i, j).s2 = 5;
        if (mapr(i, j).s1 <= 1) mapr(i, j).s1 = 1;
        if (mapr(i, j).s2 <= 1) mapr(i, j).s2 = 1;
    }
    return 0;
}

int GmmBaseIO::ReadTilesBin(std::string filename, int rc, MapTa &mapa, MapTr &mapr) {
    if (mapa.rows() != mapr.rows()) return -1;
    if (mapa.cols() != mapr.cols()) return -1;
    MapT bin(1, rc);
    int r = BinFile2EigenMatrix(filename, bin);
    if (r < 0) return r;
    for (size_t i = 0; i < mapa.rows(); ++i) {
        for (size_t j = 0; j < mapa.cols(); ++j) {
            mapa(i, j).w1 = 0;
            mapa(i, j).w2 = 0;
            mapr(i, j).w1 = 0;
            mapr(i, j).w2 = 0;
            mapa(i, j).s1 = 1000;
            mapa(i, j).s2 = 1000;
            mapr(i, j).s1 = 200;
            mapr(i, j).s2 = 200;
        }
    }
    for (size_t k = 0; k < rc; ++k) {
        int i = bin(0, k).row % mapa.rows();
        int j = bin(0, k).col % mapa.cols();
        mapa(i, j) = bin(0, k).cella;
        mapr(i, j) = bin(0, k).cellr;
        if (mapa(i, j).s1 <= 5) mapa(i, j).s1 = 5;
        if (mapa(i, j).s2 <= 5) mapa(i, j).s2 = 5;
        if (mapr(i, j).s1 <= 1) mapr(i, j).s1 = 1;
        if (mapr(i, j).s2 <= 1) mapr(i, j).s2 = 1;
    }
    return 0;
}

int GmmBaseIO::FindFileInfolder(const char *dirname, const std::string &extendname,
                                std::vector<std::string> &filenames) {
    int number = 0;
    if (NULL == dirname) {
        LOG(ERROR) << " dirname file null ! ";
        return 0;
    }

    struct stat s;
    lstat(dirname, &s);

    struct dirent *filename;
    DIR *dir;
    dir = opendir(dirname);
    if (NULL == dir) {
        LOG(ERROR) << "Can not open dir " << dirname;
        return 0;
    }

    while ((filename = readdir(dir)) != NULL) {
        if (strcmp(filename->d_name, ".") == 0 || strcmp(filename->d_name, "..") == 0) continue;

        std::string name(filename->d_name);
        std::string suffixStr = name.substr(name.find_last_of('.') + 1);

        if (suffixStr.compare(extendname) == 0) {
            filenames.push_back(name);
            ++number;
        }
    }
    return number;
}

void GmmBaseIO::Visualize(const std::string &filename, const double &resolution, const MapTa &mapa, const MapTr &mapr,
                          bool is_tiles, bool is_init) {
    int rows = (is_init ? TILES_GMM_INIT_SIZE : TILES_GMM_SIZE);
    int cols = (is_init ? TILES_GMM_INIT_SIZE : TILES_GMM_SIZE);
    if (!is_tiles) {
        rows = (map_range_.maxy - map_range_.miny) / resolution + 1;
        cols = (map_range_.maxx - map_range_.minx) / resolution + 1;
    }
    FILE *F = fopen((filename + "a.pgm").c_str(), "w");
    if (!F) {
        LOG(ERROR) << "could not open 'a.pgm' for writing!";
        return;
    }
    fprintf(F, "P6\n#\n");
    fprintf(F, "%d %d\n255\n", cols, rows);

    for (size_t i = mapa.rows() - 1; i >= 0; --i) {
        for (size_t j = 0; j < mapa.cols(); ++j) {
            if (mapa(i, j).w1 == 0 && mapa(i, j).w2 == 0 && mapa(i, j).s1 == 1000 && mapa(i, j).s2 == 1000) {
                fputc(0, F);
                fputc(0, F);
                fputc(0, F);
            } else {
                short int u = (short int)(mapa(i, j).u1 * mapa(i, j).w1 + mapa(i, j).u2 * mapa(i, j).w2);
                fputc(u, F);
                fputc(mapa(i, j).s1, F);
                fputc(mapa(i, j).s2, F);
            }
            // LOG(INFO) << (int)mapa(i, j).s1  << " " << (int)mapa(i, j).s2 << " "
            //           << (int)mapa(i, j).u1  << " " << (int)mapa(i, j).u2 << " a";
        }
    }
    fclose(F);

    FILE *Fr = fopen((filename + "r.pgm").c_str(), "w");
    if (!Fr) {
        LOG(ERROR) << "could not open 'r.pgm' for writing!";
        return;
    }
    fprintf(Fr, "P6\n#\n");
    fprintf(Fr, "%d %d\n255\n", cols, rows);
    for (size_t i = mapr.rows() - 1; i >= 0; --i) {
        for (size_t j = 0; j < mapr.cols(); ++j) {
            if (mapr(i, j).w1 == 0 && mapr(i, j).w2 == 0 && mapr(i, j).s1 == 200 && mapr(i, j).s2 == 200) {
                fputc(0, Fr);
                fputc(0, Fr);
                fputc(0, Fr);
            } else {
                short int u = (short int)(mapr(i, j).u1 * mapr(i, j).w1 + mapr(i, j).u2 * mapr(i, j).w2);
                fputc(u, Fr);
                fputc(mapr(i, j).s1, Fr);
                fputc(mapr(i, j).s2, Fr);
            }
            // LOG(INFO) << (int)mapr(i, j).s1  << " " << (int)mapr(i, j).s2 << " "
            //           << (int)mapr(i, j).u1  << " " << (int)mapr(i, j).u2 << " a";
        }
    }
    fclose(Fr);
}

}  // namespace mapping::tools
