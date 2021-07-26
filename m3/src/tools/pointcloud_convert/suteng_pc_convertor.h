
//
// Created by pengguoqi on 20-05-29.
//

#ifndef SUTENG_PC_CONVERTOR_H
#define SUTENG_PC_CONVERTOR_H

#include "common/message_def.h"
#include "common/timed_pose.h"
#include "common/num_type.h"

namespace mapping::tools {

class Compensator;

//速腾输出的packets转换成pointcloud格式
class SuTengPcConvertor {
   public:
    /**
     * 传入速腾的yaml路径(默认为local path下的suteng.yaml)
     * @param config_path
     */
    explicit SuTengPcConvertor(const std::string &config_path);
    virtual ~SuTengPcConvertor();

    int Init();

    int Convert(const SuTengScanMsg &scan, const SuTengPacketsMsg &packet, common::PointCloudType::Ptr &output);

    inline std::string GetDriverVersion() const { return driver_version_; }
    inline std::string GetLidarConfigFileName() const { return config_path_; }

    void SetDrPose(std::vector<common::TimedPose> &dr_pose_buffer);

    std::string GetSutengLidarType();

   private:
    void ConvertCloudType(RSCloudType::Ptr input_cloud, common::PointCloudType::Ptr &output_cloud);

    /// 对速腾雷达数据进行运动补偿
    void MotionCompensationWithTimeStamp(const RSCloudType ::Ptr &temp_cloud, RSCloudType::Ptr &temp_cloud_motion, double scan_header_time);

   private:
    std::string driver_version_;
    std::string config_path_;

    std::shared_ptr<robosense::lidar::Manager> manager_ptr_;
    std::shared_ptr<Compensator> compensator_;
};

}  // namespace mapping::tools

#endif  // SUTENG_PC_CONVERTOR_H
