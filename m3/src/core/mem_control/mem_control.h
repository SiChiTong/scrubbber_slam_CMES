//
// Created by gaoxiang on 2020/9/23.
//

#ifndef MAPPING_MEM_CONTROL_H
#define MAPPING_MEM_CONTROL_H

#include "common/keyframe.h"
#include "common/num_type.h"

#include <mutex>

namespace mapping::io {
class DB_IO;
}

namespace mapping::core {

struct KeyFrameCloudRecord {
    KeyFrameCloudRecord() = default;
    KeyFrameCloudRecord(common::KFPtr kf, long long lt) : keyframe(kf), kf_id(kf->id_), loaded_time(lt) {}

    IdType kf_id = 0;
    common::KFPtr keyframe = nullptr;
    long long loaded_time = 0;
};

/// 控制关键帧加载数量以防内存爆炸
/// 可以为关键帧加载点云设置一个门限，每次申请载入点云时会释放超出的关键帧
/// 内部逻辑优先卸载历史点云
class MemoryControl {
   public:
    /**
     * 限定DB路径和关键帧数
     * @param local_data_path
     * @param kf_limit
     */
    explicit MemoryControl(const std::string& local_db_path, int kf_limit = 100, bool running_as_gem = false);
    ~MemoryControl();

    /// 申请载入一个关键帧的点云
    common::PointCloudType::Ptr RequestCloud(common::KFPtr kf);

    MemoryControl(const MemoryControl&) = delete;
    bool operator=(const MemoryControl&) = delete;

   private:
    /// 清理内存
    void Clean();

    bool running_as_gem_ = false;
    int num_loaded_kf_limit_ = 100;  // 最多允许加载的关键帧数量
    long long load_time_count_ = 0;  // 加载时间计数器

    std::shared_ptr<io::DB_IO> db_io_mapping_ = nullptr;     // 建图DB接口
    std::shared_ptr<io::DB_IO> db_io_validation_ = nullptr;  // 验证DB接口

    std::string local_db_path_;

    std::map<IdType, std::shared_ptr<io::DB_IO>> traj_to_db_;  // 从轨迹到DB的映射

    std::mutex mem_mutex_;                     // 多线程锁
    std::vector<KeyFrameCloudRecord> record_;  // 加载记录
    bool verbose_ = false;                     // 打印调试信息
};

}  // namespace mapping::core

#endif  // MAPPING_MEM_CONTROL_H
