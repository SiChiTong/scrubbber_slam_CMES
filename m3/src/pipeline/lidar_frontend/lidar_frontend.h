//
// Created by wangqi on 19-7-19.
//

#ifndef MAPPING_LIDAR_ODOM_H
#define MAPPING_LIDAR_ODOM_H

#include "common/keyframe.h"
#include "common/mapping_point_types.h"
#include "io/yaml_io.h"
#include "pipeline/pipeline_context.h"

namespace mapping::pipeline {

/// TODO 用memory_control 预载一些雷达点云，避免边读边算
class LidarFrontend : public PipelineContext {
   public:
    LidarFrontend(const io::YAML_IO &yaml_file, RunMode run_mode = RunMode::PIPELINE, IdType start_kf = 0,
                  IdType end_kf = 0);
    ~LidarFrontend() override;

    /// Context 接口
    /// 初始化，成功返回 true
    virtual bool Init() override;

    /// 开始处理，完成后返回true，无论结果是否成功
    /// 要求完成后更新context status状态，以便 engine 确定它的完成情况
    /// NOTE: 模块内容应阻塞此Start函数，Start函数返回表明已执行完毕
    virtual bool Start() override;

    void SetDebugParam(bool save_pcd = true) { debug_save_pcd_ = save_pcd; }

   private:
    bool ComputeLidarTrajectory(size_t traj_id);

    bool ReadDataPreparingData();

    /// 建图框架下存储结果
    bool SaveResultsForMapping();

    /// GEM框架下存储
    bool SaveResultsForGEM();

   private:
    const io::YAML_IO &yaml_file_;

    std::string local_data_path_;

    /// 在GEM模式下，由于DB无法通过NFS共享，于是必须拷贝一个本地备份
    std::string local_db_path_;  // 本地DB位置

    int num_lidar_lines_ = 16;  // lidar 线数

    IdType start_kf_ = 0;
    IdType end_kf_ = 0;

    std::string matching_type_;
    std::map<IdType, std::vector<common::KFPtr>> keyframes_;  // trajectory id to keyframes

    bool debug_save_pcd_ = true;
};

}  // namespace mapping::pipeline
#endif  // MAPPING_LIDAR_ODOM_H
