//
// Created by pengguoqi on 19-7-15.
//

#include "db_io.h"
#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>

namespace mapping::io {

using namespace mapping::common;

/// 内部类型定义
struct PointXYZ_DB {
    float x;
    float y;
    float z;
    float height;

    float range;
    uint8_t intensity;
    uint8_t intensity_vi;
    uint8_t ring;
    uint8_t type;  /// classification of point cloud
};

/// pcl点云类型转换为db存储点云类型
void PCLPointsToDBPoints(const PointCloudPtr &frame_points, PointXYZ_DB *db_points) {
    int points_size = frame_points->points.size();
    for (int i = 0; i < points_size; i++) {
        db_points[i].x = frame_points->points[i].x;
        db_points[i].y = frame_points->points[i].y;
        db_points[i].z = frame_points->points[i].z;
        db_points[i].intensity = frame_points->points[i].intensity;
        db_points[i].height = frame_points->points[i].height;
        db_points[i].range = frame_points->points[i].range;
        db_points[i].intensity_vi = frame_points->points[i].intensity_vi;
        db_points[i].ring = frame_points->points[i].ring;
        db_points[i].type = frame_points->points[i].type;
    }
}

/// db存储点云类型转换为pcl点云类型
void DBPointsToPCLPoints(const PointXYZ_DB *db_points, const int char_length, PointCloudPtr &frame_points) {
    int points_size = char_length / sizeof(PointXYZ_DB);
    for (int i = 0; i < points_size; i++) {
        PointType pcl_point;
        pcl_point.x = db_points[i].x;
        pcl_point.y = db_points[i].y;
        pcl_point.z = db_points[i].z;
        pcl_point.intensity = db_points[i].intensity;
        pcl_point.height = db_points[i].height;
        pcl_point.range = db_points[i].range;
        pcl_point.intensity_vi = db_points[i].intensity_vi;
        pcl_point.ring = db_points[i].ring;
        pcl_point.type = db_points[i].type;
        frame_points->push_back(pcl_point);
    }
}

// db存储点云类型转换成PointXYZI类型
void DBPointsToPCLPointsXYZI(const PointXYZ_DB *db_points, const int char_length,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr &frame_points) {
    int points_size = char_length / sizeof(PointXYZ_DB);
    for (int i = 0; i < points_size; i++) {
        PointXYZI pcl_point;
        pcl_point.x = db_points[i].x;
        pcl_point.y = db_points[i].y;
        pcl_point.z = db_points[i].z;
        pcl_point.intensity = db_points[i].intensity;
        frame_points->push_back(pcl_point);
    }
}

DB_IO::~DB_IO() {
    if (db_rw_.IsOpen()) {
        CloseMapDataBase();
    }
}

bool DB_IO::OpenMapDataBase() {
    bool result = true;
    try {
        result = db_rw_.OpenDB(db_path_.c_str());
    } catch (...) {
        LOG(ERROR) << " open db failed!!! ";
        result = false;
    }
    return result;
}

bool DB_IO::CloseMapDataBase() {
    db_rw_.CloseDB();
    return true;
}

bool DB_IO::WriteOriginPointInformationToDB(const OriginPointInformation &origin) {
    db_rw_.BeginTransaction();
    if (!db_rw_.WriteMoveOriginPoint(origin.map_origin_x, origin.map_origin_y, origin.map_origin_z,
                                     origin.map_origin_zone, origin.is_southern)) {
        LOG(ERROR) << " write move origin point failed ";
        db_rw_.RollbackTransaction();
        return false;
    }
    db_rw_.CommitTransaction();
    return true;
}

bool DB_IO::ReadOriginPointInformationByDB(OriginPointInformation &origin) {
    if (!db_rw_.ReadMoveOriginPoint(origin.map_origin_x, origin.map_origin_y, origin.map_origin_z,
                                    origin.map_origin_zone, origin.is_southern)) {
        LOG(ERROR) << " read move origin point failed ";
        db_rw_.RollbackTransaction();
        return false;
    }
    return true;
}

int DB_IO::ReadTrackLineByDB() {
    std::vector<UniqueType> mLineIdx;
    mLineIdx.clear();
    if (!db_rw_.QueryAllTrackLine(mLineIdx)) LOG(ERROR) << " query all track line failed) ";
    return mLineIdx.size();
}

bool DB_IO::WriteTrackToDB(const std::vector<TrackPoseD> &track) {
    int index = ReadTrackLineByDB();
    index = index < 1 ? 0 : index;
    db_rw_.BeginTransaction();
    int length = track.size() * sizeof(TrackPoseD);
    if (!db_rw_.WriteTrack(index, (void *)&(track[0]), length)) {
        LOG(ERROR) << " write track to DB database failed (write track failed) ";
        return false;
    }
    db_rw_.CommitTransaction();
    return true;
}

bool DB_IO::ReadTrackByDB(const int index, int len, std::vector<TrackPoseD> &track) {
    int length = len * sizeof(TrackPoseD);
    char *t_data = NULL;
    if (!db_rw_.ReadTrack(index, t_data, length)) {
        LOG(ERROR) << " read track database failed (read track failed) ";
    }

    TrackPoseD *track_pose = (TrackPoseD *)t_data;
    for (int i = 0; i < len; ++i) {
        track.push_back(*track_pose++);
    }
    return true;
}

bool DB_IO::WriteImageToDB(const std::vector<ImageData> &images) {
    db_rw_.BeginTransaction();
    for (const ImageData &im : images) {
        db_rw_.WriteImage(im.id, im.data, im.length);
    }

    db_rw_.CommitTransaction();
    return true;
}

bool DB_IO::ReadAllImageIDByDB(std::vector<UniqueType> &image_id) {
    image_id.clear();
    if (!db_rw_.QueryAllTrackImage(image_id)) {
        LOG(ERROR) << " read all image id failed ";
        return false;
    }
    return true;
}

bool DB_IO::WriteFunctionPointsToDB(const std::vector<FunctionPoint> &function_points) {
    db_rw_.BeginTransaction();
    for (auto &fp : function_points) {
        if (!db_rw_.WriteFunctionPoint(fp.id, fp.type, fp.number, fp.pose, fp.heading)) {
            LOG(ERROR) << "write function point error!!!";
            return false;
        }
    }
    db_rw_.CommitTransaction();
    return true;
}

bool DB_IO::WriteScanContextToDB(const std::vector<ScanContextData> &data) {
    db_rw_.BeginTransaction();
    for (size_t i = 0; i < data.size(); ++i) {
        if (!db_rw_.WriteScanContextBlock(data[i].x, data[i].y, data[i].data, data[i].size)) {
            LOG(ERROR) << "write scancontext error!!!";
            db_rw_.RollbackTransaction();
            return false;
        }
    }
    db_rw_.CommitTransaction();
    return true;
}

bool DB_IO::WritePoseAndCloudToDB(std::shared_ptr<common::KeyFrame> &keyframe) {
    return WritePoseAndCloudToDB({keyframe});
}

bool DB_IO::WritePoseAndCloudToDB(const std::vector<std::shared_ptr<common::KeyFrame>> &keyframes) {
    db_rw_.BeginTransaction();
    bool has_fail = false;
    for (auto &kf : keyframes) {
        if (kf->cloud_->empty()) {
            continue;
        }

        int size_count = kf->cloud_->points.size();
        int length = 0;
        PointXYZ_DB *tmp = new PointXYZ_DB[size_count];
        PCLPointsToDBPoints(kf->cloud_, tmp);
        length = size_count * sizeof(PointXYZ_DB);

        if (!db_rw_.WriteRawLaserData(kf->id_, kf->optimized_pose_stage_2_, &tmp[0], length, 1)) {
            has_fail = true;
        }
        delete[] tmp;
    }

    if (has_fail) {
        LOG(ERROR) << " some key frames can not write to database ";
        db_rw_.RollbackTransaction();
        return false;
    }

    db_rw_.CommitTransaction();

    return true;
}

bool DB_IO::UpdateKeyFramePoses(const std::map<IdType, common::KFPtr> &keyframes) {
    std::vector<int> frames_write_fail_id;
    db_rw_.BeginTransaction();
    for (const auto &kf : keyframes) {
        if (!db_rw_.WritePose(kf.first, kf.second->optimized_pose_stage_2_)) {
            frames_write_fail_id.push_back(kf.first);
        }
    }

    if (frames_write_fail_id.size() > 0) {
        LOG(ERROR) << " some key frames can not write to database";
        db_rw_.RollbackTransaction();
        return false;
    }

    db_rw_.CommitTransaction();
    return true;
}

bool DB_IO::ReadAllUniqueIdAndPose(std::map<int, SE3> &map_id_pose) {
    if (!db_rw_.QueryAllRawLaserPose(map_id_pose)) {
        LOG(ERROR) << " read all mapid and pose failed ";
        return false;
    }
    return true;
}

bool DB_IO::ReadAllKF(std::vector<std::shared_ptr<common::KeyFrame>> &keyframes,
                      std::vector<int> &frames_read_fail_id) {
    std::map<int, SE3> map_id_pose;
    frames_read_fail_id.clear();
    map_id_pose.clear();
    if (!ReadAllUniqueIdAndPose(map_id_pose)) {
        LOG(ERROR) << " read all mapid and pose failed (ReadPoseAndCloudByDB) ";
        return false;
    }

    for (auto &id_pose : map_id_pose) {
        pcl::PointCloud<io::PointType>::Ptr out_points(new pcl::PointCloud<io::PointType>());
        int ret_length = 0;
        void *p_data = NULL;
        SE3 pose;
        int db_version = 0;
        if (db_rw_.ReadRawLaserData(id_pose.first, pose, p_data, ret_length, db_version)) {
            PointXYZ_DB *db_points = (PointXYZ_DB *)p_data;
            DBPointsToPCLPoints(db_points, ret_length, out_points);
            delete[]((char *)p_data);

            auto kf = std::make_shared<common::KeyFrame>();
            kf->id_ = id_pose.first;
            kf->optimized_pose_stage_2_ = pose;
            kf->cloud_ = out_points;
            keyframes.push_back(kf);
        } else {
            frames_read_fail_id.push_back(id_pose.first);
        }
    }

    if (frames_read_fail_id.size() > 0) {
        LOG(ERROR) << " some key frames read failed ";
    }
    return true;
}

bool DB_IO::ReadSingleKF(int id, std::shared_ptr<KeyFrame> kf, bool rewrite_pose) {
    if (kf == nullptr) {
        return false;
    }

    PointCloudPtr out_points(new PointCloudType());
    int ret_length = 0;
    void *p_data = NULL;
    SE3 pose;
    int db_version = 0;

    if (db_rw_.ReadRawLaserData(id, pose, p_data, ret_length, db_version)) {
        PointXYZ_DB *db_points = (PointXYZ_DB *)p_data;
        DBPointsToPCLPoints(db_points, ret_length, out_points);
        delete[]((char *)p_data);
        kf->id_ = id;
        kf->cloud_ = out_points;

        /// convert timestamp
        double kf_time = kf->timestamp_;
        ros::Time kf_ros_time(kf_time);
        std::uint64_t kf_pcl_time;
        pcl_conversions::toPCL(kf_ros_time, kf_pcl_time);
        kf->cloud_->header.stamp = kf_pcl_time;

        if (rewrite_pose) {
            kf->optimized_pose_stage_2_ = pose;
        }
    } else {
        LOG(ERROR) << "keyframes load failed";
        return false;
    }

    return true;
}

bool DB_IO::DeleteRawLaserRecord(bool delete_type, UniqueType unique_id) {
    if (delete_type) {
        if (!db_rw_.ClearRawLaserData()) {
            LOG(ERROR) << "  clear raw laser data failed (delete raw laser data) ";
            return false;
        }
    } else {
        if (!db_rw_.DeleteRawLaserData(unique_id)) {
            LOG(WARNING) << "  delete the unique_id raw laser data failed (delete raw laser data) ";
        }
    }
    return true;
}

bool DB_IO::DeleteDB(const std::vector<std::string> &db_files) {
    if (db_files.size() < 1) {
        LOG(WARNING) << " No db needs to be deleted ";
        return true;
    }
    for (size_t i = 1; i < db_files.size(); ++i) {
        std::remove((db_files[i]).c_str());
    }
    return true;
}

bool DB_IO::UpdatePose() {
    std::map<int, SE3> map_id_pose;
    if (!ReadAllUniqueIdAndPose(map_id_pose)) {
        LOG(ERROR) << " read all mapid and pose failed (ReadPoseAndCloudByDB) ";
        return false;
    }

    PointCloudPtr pose_points_ptr(new PointCloudType);

    for (auto item = map_id_pose.begin(); item != map_id_pose.end(); item++) {
        common::PointXYZIHIRBS temp_point;
        // only used for 2D situation
        temp_point.x = item->second.translation().x();
        temp_point.y = item->second.translation().y();
        temp_point.z = 0;
        temp_point.intensity = item->first;
        pose_points_ptr->push_back(temp_point);
    }

    if (pose_points_ptr->points.size() < 1) {
        LOG(ERROR) << "no pose is found in DB";
        return false;
    }
    kdtree_pose_->setInputCloud(pose_points_ptr);
    LOG(INFO) << "kdtree of poses is updated!";
    return true;
}

bool DB_IO::ReadDiscretePoseAndCloudByID(const std::vector<int> &indices,
                                         std::vector<std::shared_ptr<common::KeyFrame>> &keyframes) {
    std::vector<int> frames_read_fail_id;
    keyframes.clear();
    for (auto &j : indices) {
        pcl::PointCloud<io::PointType>::Ptr out_points(new pcl::PointCloud<io::PointType>());
        int ret_length = 0;
        void *p_data = NULL;
        SE3 pose;
        int db_version = 0;
        if (db_rw_.ReadRawLaserData(j, pose, p_data, ret_length, db_version)) {
            PointXYZ_DB *db_points = (PointXYZ_DB *)p_data;
            DBPointsToPCLPoints(db_points, ret_length, out_points);
            delete[]((char *)p_data);

            auto kf = std::make_shared<common::KeyFrame>();
            kf->id_ = j;
            kf->cloud_ = out_points;
            kf->optimized_pose_stage_2_ = pose;
            keyframes.push_back(kf);
        } else {
            frames_read_fail_id.push_back(j);
        }
    }
    if (frames_read_fail_id.size() > 0) {
        LOG(ERROR) << " some key frames read failed: " << frames_read_fail_id.size();
    }
    return true;
}

bool DB_IO::ReadDiscretePoseAndCloudByRange(double x, double y, double range,
                                            std::vector<std::shared_ptr<common::KeyFrame>> &keyframes) {
    std::vector<int> frames_read_fail_id;
    keyframes.clear();
    // kdtres radius search
    if (!kdtree_init_flag_) {
        if (!UpdatePose()) {
            return false;
        } else {
            LOG(INFO) << "kdtree initialization";
        }
    }

    common::PointXYZIHIRBS current_pose_point;
    current_pose_point.x = x;
    current_pose_point.y = y;
    std::vector<int> indices;
    std::vector<float> distances;
    kdtree_pose_->radiusSearch(current_pose_point, range, indices, distances, 0);

    for (auto &j : indices) {
        pcl::PointCloud<io::PointType>::Ptr out_points(new pcl::PointCloud<io::PointType>());
        int ret_length = 0;
        void *p_data = NULL;
        SE3 pose;
        int db_version = 0;
        if (db_rw_.ReadRawLaserData(j, pose, p_data, ret_length, db_version)) {
            PointXYZ_DB *db_points = (PointXYZ_DB *)p_data;
            DBPointsToPCLPoints(db_points, ret_length, out_points);
            delete[]((char *)p_data);

            auto kf = std::make_shared<common::KeyFrame>();
            kf->id_ = j;
            kf->cloud_ = out_points;
            kf->optimized_pose_stage_2_ = pose;
            keyframes.push_back(kf);
        } else {
            frames_read_fail_id.push_back(j);
        }
    }
    if (frames_read_fail_id.size() > 0) {
        LOG(ERROR) << " some key frames read failed ";
    }
    return true;
}

bool DB_IO::ReadDiscretePoseByRange(double x, double y, double range,
                                    std::vector<std::shared_ptr<common::KeyFrame>> &keyframes) {
    std::vector<int> frames_read_fail_id;
    keyframes.clear();
    // kdtres radius search
    if (!kdtree_init_flag_) {
        if (!UpdatePose()) {
            return false;
        } else {
            LOG(INFO) << "kdtree initialization";
        }
    }

    common::PointXYZIHIRBS current_pose_point;
    current_pose_point.x = x;
    current_pose_point.y = y;
    std::vector<int> indices;
    std::vector<float> distances;
    kdtree_pose_->radiusSearch(current_pose_point, range, indices, distances, 0);

    for (auto &j : indices) {
        int ret_length = 0;
        void *p_data = NULL;
        SE3 pose;
        int db_version = 0;
        if (db_rw_.ReadRawLaserData(j, pose, p_data, ret_length, db_version)) {
            delete[]((char *)p_data);
            auto kf = std::make_shared<common::KeyFrame>();
            kf->id_ = j;
            kf->optimized_pose_stage_2_ = pose;
            keyframes.push_back(kf);
        } else {
            frames_read_fail_id.push_back(j);
        }
    }
    if (frames_read_fail_id.size() > 0) {
        LOG(ERROR) << " some key frames read failed ";
    }
    return true;
}

bool DB_IO::UpdatePoseByID(const std::map<int, SE3> &map_pose_id) {
    std::vector<int> frames_write_fail_id;
    db_rw_.BeginTransaction();
    for (auto item = map_pose_id.begin(); item != map_pose_id.end(); item++) {
        if (!db_rw_.WritePose(item->first, item->second)) {
            frames_write_fail_id.push_back(item->first);
        }
    }

    if (frames_write_fail_id.size() > 0) {
        LOG(ERROR) << " some key frames can not write to database ";
        db_rw_.RollbackTransaction();
        return false;
    }
    db_rw_.CommitTransaction();
    if (!UpdatePose()) {
        return false;
    }
    return true;
}

bool DB_IO::UpdateIdByInc(int inc) {
    if (inc == 0) {
        return true;
    }

    std::map<int, SE3> poses;
    db_rw_.QueryAllRawLaserPose(poses);
    db_rw_.BeginTransaction();

    if (inc < 0) {
        for (auto &p : poses) {
            db_rw_.UpdatePoseID(p.first, p.first + inc);
        }
    } else {
        // 反向
        for (auto iter = poses.rbegin(); iter != poses.rend(); ++iter) {
            db_rw_.UpdatePoseID(iter->first, iter->first + inc);
        }
    }
    db_rw_.CommitTransaction();
    return true;
}

}  // namespace mapping::io
