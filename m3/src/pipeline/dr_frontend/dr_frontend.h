//
// Created by gaoxiang on 2020/8/14.
//

#ifndef MAPPING_DR_FRONTEND_H
#define MAPPING_DR_FRONTEND_H

#include <regex>
#include "common/car_type.h"
#include "common/gps_status_def.h"
#include "common/keyframe.h"
#include "common/message_def.h"
#include "common/num_type.h"
#include "common/origin_files.h"
#include "common/timed_pose.h"
#include "common/trajectory.h"
#include "io/db_io.h"
#include "io/yaml_io.h"
#include "pipeline/pipeline_context.h"

/// forward declare
namespace mapping {
namespace core {
class DeadReckoning;
class GpsTransform;
}  // namespace core

namespace tools {
class PerceptionInterface;
class SuTengPcConvertor;
class SuTengIRADPcConvertor;
class HeSaiPcConvertor;
}  // namespace tools

}  // namespace mapping

namespace mapping::pipeline {

/**
 * DR 前端
 * DR前端负责收集各个包中的轨迹，如果有自动分包，处理分包逻辑
 * 然后，把各包数据归整成若干条trajectory，再分成keyframes
 * keyframes的各种pose测量被填入，点云放入DB中
 *
 * TODO 考虑z包轨迹和点云是否存储在DB中（不然仿真还得这么来一遍）
 * z包点云和pose存在valid.db中，另外pose也会写入map.db里作为贴边轨迹
 */
class DRFrontend : public PipelineContext {
   public:
    /// 给定配置文件
    DRFrontend(const io::YAML_IO &yaml_file, RunMode run_mode = RunMode::PIPELINE);
    ~DRFrontend() override;

    /// Context 接口
    /// 初始化，成功返回 true
    virtual bool Init() override;

    /// 开始处理，完成后返回true，无论结果是否成功
    /// 要求完成后更新context status状态，以便 engine 确定它的完成情况
    /// NOTE: 模块内容应阻塞此Start函数，Start函数返回表明已执行完毕
    virtual bool Start() override;

    /// 缓存中间结果
    virtual bool Save() override;

    virtual bool Load() override;

    /// 填写任务信息
    virtual bool FillTaskInfo(TaskInfo &info) override;

    void SetDebugParams(bool save_pcd);

   private:
    /// 计算DR轨迹
    bool ComputeDRTrajectory();

    /// 根据包名分析包间关系
    bool ParseBagsByName();

    /// 读取包内消息并按时间排列
    void CollectMessage(std::shared_ptr<common::Trajectory> t);

    /// 处理一个IMU数据并记录在dr_path_中
    bool ProcessIMU(ImuMsgPtr imu_msg);

    /// 处理一个Odom数据
    void ProcessOdom(OdomMsgPtr odom_msg);

    /// 根据名称查找对应的trajectory
    std::shared_ptr<common::Trajectory> FindStartTrajectory(const std::string &bag_name);

    /// 建立关键帧并存储到DB中
    /// 关键帧是基于dr pose建立的，完成之后寻找对应的gps_pose和激光点云
    void ExtractKeyFrames(std::shared_ptr<common::Trajectory> trajectory);

    /// 寻找同步的GPS pose
    bool FindSynGpsPose(double pose_time, const std::map<double, common::GpsPoseStatus> &gps_path,
                        common::GpsPoseStatus &best_match);

    /// 读取激光数据，分配给关键帧并写入DB
    /// NOTE 为不同雷达计算点云数据（由于驱动和内部逻辑都不一样，不使用模板类，但逻辑大体相似）
    void AssignPointCloudDataForVelodyne(io::DB_IO &db_io_mapping, io::DB_IO &db_io_validation);
    void AssignPointCloudDataForHesai(io::DB_IO &db_io_mapping, io::DB_IO &db_io_validation);
    void AssignPointCloudDataForSuteng(io::DB_IO &db_io_mapping, io::DB_IO &db_io_validation);
    void AssignPointCloudDataForSutengIRAD(io::DB_IO &db_io_mapping, io::DB_IO &db_io_validation);

    /// 寻找并处理激光数据
    bool FindSynLidarDataForVelodyne(std::shared_ptr<common::KeyFrame> kf,
                                     const std::deque<PacketsMsgPtr> &packets_buffer);
    bool FindSynLidarDataForSuteng(std::shared_ptr<common::KeyFrame> kf,
                                   const std::deque<SuTengScanMsgPtr> &scan_buffer,
                                   const std::deque<SuTengPacketsMsgPtr> &packets_buffer);
    bool FindSynLidarDataForSuteng(std::shared_ptr<common::KeyFrame> kf,
                                   const std::deque<SuTengIRADPacketsMsgPtr> &scan_buffer);
    bool FindSynLidarDataForHesai(std::shared_ptr<common::KeyFrame> kf, const std::deque<HeSaiScanMsgPtr> &scan_buffer);

    /// 存储结果数据
    void SaveResults();

    /// 重置DR
    void ResetDR();

    ///解析速腾类型RS80/RS32等，不解析RSBP
    int CheckSutengLidarType(const std::string suteng_type);

    ///根据关键帧激光时间重置dr和gps位姿
    void ResetKeyFrameTimeAndPose(const double &scan_time, std::shared_ptr<common::KeyFrame> kf);

    /// 从某种数据buffer里查询时间最近的，返回迭代器
    template <typename T>
    auto FindClosest(double query_time, const std::deque<T> &queue, const std::function<double(T data)> &time_getter)
        -> decltype(queue.cbegin()) {
        auto iter = queue.cbegin();
        auto best_iter = iter;
        for (; iter != queue.end(); ++iter) {
            auto iter_next = iter;
            iter_next++;

            if (iter_next == queue.end()) {
                break;
            }

            double t1 = time_getter(*iter);
            double t2 = time_getter(*iter_next);
            if (t1 < 0) {
                break;
            }

            if (t1 < query_time && t2 >= query_time) {
                best_iter = (query_time - t1) < (t2 - query_time) ? iter : iter_next;
                break;
            }
        }

        if (iter == queue.end() || iter == queue.end() - 1) {
            best_iter = queue.end() - 1;
        }
        return best_iter;
    }

    std::string report_;
    io::YAML_IO yaml_file_;

    common::CarType car_type_;        // 车辆类型
    std::string lidar_manufacturer_;  // 雷达厂家

    std::string local_data_path_;
    std::string local_db_path_;
    common::OriginFiles origin_files_;

    std::shared_ptr<core::DeadReckoning> dr_;

    /// DR轨迹
    std::map<IdType, std::shared_ptr<common::Trajectory>> trajectory_;
    IdType traj_id_ = 0;
    IdType keyframe_id_ = 0;
    std::vector<common::KFPtr> keyframes_cache_mapping_;     // 缓存的关键帧（建图）
    std::vector<common::KFPtr> keyframes_cache_validation_;  // 缓存的关键帧（验证）
    size_t keyframes_cache_max_size_ = 100;                  // 最大缓存关键帧数量

    std::map<double, SE3> dr_path_;  // 由DR形成的轨迹，与IMU同步gps_pa

    // GPS 转换关系
    std::shared_ptr<core::GpsTransform> gps_trans_ = nullptr;  // GPS原点转换
    std::map<double, common::GpsPoseStatus> gps_path_;         // 由gps形成的轨迹，与IMU同步
    std::map<IdType, std::map<double, common::GpsPoseStatus>> all_gps_path_;

    // 感知接口
    std::shared_ptr<tools::PerceptionInterface> perception_interface_ = nullptr;
    std::map<std::shared_ptr<common::KeyFrame>, std::vector<common::TimedPose>> kf_to_dr_poses_;

    std::shared_ptr<mapping::tools::SuTengPcConvertor> suteng_lidar_convertor_;
    std::shared_ptr<mapping::tools::SuTengIRADPcConvertor> suteng_lidar_irad_convertor_;
    std::shared_ptr<mapping::tools::HeSaiPcConvertor> hesai_lidar_convertor_;

    /// 参数
    double dist_th_ = 0.5;        // 关键帧距离阈值
    double angle_dist_th_ = 5.0;  // 关键帧角度阈值
    double time_diff_th_ = 30.0;  // 关键帧时间阈值

    /// 报告信息
    TaskInfo task_info_;

    /// 调试
    bool debug_save_pcd_ = false;
    
    // 是否为自研激光驱动
    bool is_irad_ = true;

    double dr_angle_ini_set_ = 90;
    double ant_angle_ = 0.0;
};

}  // namespace mapping::pipeline

#endif  // MAPPING_DR_FRONTEND_H
