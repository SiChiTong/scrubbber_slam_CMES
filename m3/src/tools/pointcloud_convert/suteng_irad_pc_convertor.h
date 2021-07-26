
//
// Created by pengguoqi on 21-01-28.
//

#ifndef SUTENG_IRAD_PC_CONVERTOR_H
#define SUTENG_IRAD_PC_CONVERTOR_H

#include "common/message_def.h"
#include "common/vehicle_calib_param.h"
#include "common/timed_pose.h"
#include "common/num_type.h"

namespace mapping::tools {

class Compensator;

//速腾输出的packets转换成pointcloud格式
class SuTengIRADPcConvertor {
   public:
    /**
     * 传入速腾的yaml路径(默认为local path下的suteng.yaml)
     * @param config_path
     */
    explicit SuTengIRADPcConvertor();
    virtual ~SuTengIRADPcConvertor();

    int Init(const common::VehicleCalibrationParam &config);

    int Convert(const SuTengIRADPacketsMsg &packet, common::PointCloudType::Ptr &output);

    inline std::string GetDriverVersion() const { return driver_version_; }

    void SetDrPose(std::vector<common::TimedPose> &dr_pose_buffer);

    std::string GetSutengLidarType();

   private:
    void ConvertCloudType(IRADCloudType::Ptr input_cloud, common::PointCloudType::Ptr &output_cloud);

    /// 对速腾雷达数据进行运动补偿
    void MotionCompensationWithTimeStamp(const IRADCloudType ::Ptr &temp_cloud, IRADCloudType::Ptr &temp_cloud_motion, double packet_header_time);

   private:
    std::string driver_version_;
    std::shared_ptr<avos::driver::RsLidarApi> manager_ptr_;
    std::shared_ptr<Compensator> compensator_;
};

}  // namespace mapping::tools

#endif  // SUTENG_PC_CONVERTOR_H
