//
// Created by gaoxiang on 2020/8/11.
//

#ifndef MAPPING_TASK_INFO_H
#define MAPPING_TASK_INFO_H

#include <ctime>
#include <iostream>

#include "common/num_type.h"
#include "common/std_headers.h"

namespace mapping::pipeline {

/// 任务信息
struct TaskInfo {
    TaskInfo() {
        time_t now = time(nullptr);
        start_time = *localtime(&now);
        end_time = *localtime(&now);
    }

    /// 保存文件
    void Save(std::ofstream &out);

    /// 读取文件
    void Load(std::ifstream &in);

    tm start_time{}, end_time{};           // 收到任务时间，起始时间，结束时间
    std::string map_name;                  // 地图名，构建pipeline时填入
    double time_usage = 0;                 // 总用时，结束pipeline时填入
    int valid_bags = 0;                    // 有效的包数，preprocessing时填入
    int valid_trajs = 0;                   // 有效的轨迹数，preprocessing时填入
    unsigned long int total_bag_size = 0;  // 数据文件大小，data fetching时填入
    double length = 0;                     // 长度，data preparing时填入
    double area = 0;                       // 面积，data preparing时填入
    bool success = false;                  // 是否成功，完成时填入
    int total_keyframes = 0;               // 总关键帧数量，data preparing填入
    bool checkin_passed = false;           // 准入是否成功，checkin填入
    bool checkout_passed = false;          // 准出是否成功，checkout时填入
    std::string scene_type = "室外";       // 场景类型，data preparing时填入
    std::string failed_reason;             // 失败原因，各步骤可能导致失败时都可以填入
    std::vector<V2d> loop_points;          // 产生回环的点，由pose optimization填入

    std::string map_db_url;  // DB下载链接
    std::string pcd_url;     // PCD下载链接
    std::string gmm_url;     // GMM下载链接
    std::string video_url;   // 视频链接

    // 云控交互部分
    bool enable_cloud_config = false;  // 是否使用云控侧交互信息
    std::string access_token;          // 建图帐号token
    std::string account_name;          // 建图人员姓名
    int task_id = 0;                   // config中任务编号
    std::string area_id;               // config中场景编号
    std::string project_name;          // config中场地名称
    std::string version;               // config中地图版本
};
}  // namespace mapping::pipeline

#endif  // MAPPING_TASK_INFO_H
