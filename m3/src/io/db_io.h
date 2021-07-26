//
// Created by pengguoqi on 19-7-15.
//
#ifndef DB_IO_H__
#define DB_IO_H__

#define PCL_NO_PRECOMPILE

#include "common/function_point.h"
#include "common/keyframe.h"
#include "common/mapping_point_types.h"
#include "common/num_type.h"
#include "common/origin_point_info.h"
#include "common/track_pose.h"
#include "io/db_rw.h"

#include <pcl/kdtree/kdtree_flann.h>

namespace mapping::io {

/// db读写类
/**
 * 该类在构造时指定DB文件路径，并默认会将其打开，然后一直保持打开状态以节省时间
 * 析构时关闭该文件
 */
class DB_IO {
   public:
    explicit DB_IO(std::string db_path) : db_path_(std::move(db_path)) {
        kdtree_pose_.reset(new pcl::KdTreeFLANN<common::PointXYZIHIRBS>());
        OpenMapDataBase();
    }

    ~DB_IO();

    // disable copy
    DB_IO(const DB_IO &) = delete;
    bool operator=(const DB_IO &) = delete;

    struct ImageData {
        UniqueType id;
        void *data;
        int length;
    };

    struct ScanContextData {
        int x;
        int y;
        void *data;
        int size;

        ScanContextData(int _x, int _y, void *_data, int _size) : x(_x), y(_y), data(_data), size(_size) {}
    };

    /// 写入地图原点数据
    bool WriteOriginPointInformationToDB(const common::OriginPointInformation &origin);

    /// 写贴边轨迹
    bool WriteTrackToDB(const std::vector<common::TrackPoseD> &track);

    /// 写入图像数据
    bool WriteImageToDB(const std::vector<ImageData> &images);

    /// 写入功能点信息
    bool WriteFunctionPointsToDB(const std::vector<common::FunctionPoint> &function_points);

    /// 写入初始化信息文件
    bool WriteScanContextToDB(const std::vector<ScanContextData> &data);

    /// 以点云和位姿的形式输入，将点云数据写入db中
    bool WritePoseAndCloudToDB(std::shared_ptr<common::KeyFrame> &keyframe);

    bool WritePoseAndCloudToDB(const std::vector<std::shared_ptr<common::KeyFrame>> &keyframes);

    // 只更新位姿，不更新点云
    // NOTE 默认使用optimized_pose_2
    bool UpdateKeyFramePoses(const std::map<IdType, common::KFPtr> &keyframes);

    /// 读取地图原点数据
    bool ReadOriginPointInformationByDB(common::OriginPointInformation &origin);

    /// 根据index读取贴边轨迹
    bool ReadTrackByDB(const int index, int len, std::vector<common::TrackPoseD> &track);

    /// 获取所有图像id
    bool ReadAllImageIDByDB(std::vector<UniqueType> &image_id);

    /**
     * 根据id获取某关键帧的optimized pose和点云
     * @param id keyframe id
     * @param kf keyframe to be loaded
     * @param rewrite_opt_pose 是否用DB里的pose覆盖keyframe中的pose?
     * @return
     * @note keyframe 的时间戳不存储于DB中
     */
    bool ReadSingleKF(int id, std::shared_ptr<common::KeyFrame> kf, bool rewrite_opt_pose = true);

    /// 读取点云数据和位姿数据，会载入整个DB中的点云
    bool ReadAllKF(std::vector<std::shared_ptr<common::KeyFrame>> &keyframes, std::vector<int> &frames_read_fail_id);

    /// 获取db中全部的位姿和id
    bool ReadAllUniqueIdAndPose(std::map<int, SE3> &map_id_pose);

    /// 删除db文件
    bool DeleteDB(const std::vector<std::string> &db_files);

    /// 删除点云数据
    /// delete_type为true，删除dn中所有点云数据；为false：删除第unique_id帧点云数据
    bool DeleteRawLaserRecord(bool delete_type, UniqueType unique_id);

    //第一次优化结束后开始调用
    bool UpdatePose();

    // 根据ID读取点云和位姿
    bool ReadDiscretePoseAndCloudByID(const std::vector<int> &indices,
                                      std::vector<std::shared_ptr<common::KeyFrame>> &keyframes);

    // 根据位姿和范围读取点云
    bool ReadDiscretePoseAndCloudByRange(double x, double y, double range,
                                         std::vector<std::shared_ptr<common::KeyFrame>> &keyframes);

    // 根据位姿和范围读取点云
    bool ReadDiscretePoseByRange(double x, double y, double range,
                                 std::vector<std::shared_ptr<common::KeyFrame>> &keyframes);

    // 根据ID更新位姿
    bool UpdatePoseByID(const std::map<int, SE3> &map_pose_id);

    /// 给所有point cloud id一个增量
    bool UpdateIdByInc(int inc);

   private:
    /// 打开db文件
    bool OpenMapDataBase();

    /// 关闭db文件
    bool CloseMapDataBase();

    /// 读取db中所有的贴边轨迹标号
    int ReadTrackLineByDB();

   protected:
    std::string db_path_;
    DBRW db_rw_;

    pcl::KdTreeFLANN<common::PointType>::Ptr kdtree_pose_;
    bool kdtree_init_flag_ = false;
};
}  // namespace mapping::io

#endif  // DB_READ_WRITE_H__
