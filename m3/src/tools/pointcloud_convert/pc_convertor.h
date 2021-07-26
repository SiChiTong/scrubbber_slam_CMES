//
// Created by idriver on 19-7-19.
//

#ifndef MAPPING_PC_CONVERTOR_H
#define MAPPING_PC_CONVERTOR_H

#include "tools/pointcloud_convert/compensator.h"
#include "tools/pointcloud_convert/packets_parser.h"

namespace mapping::tools {

// velodyne输出的packets转换成pointcloud格式
class PcConvertor {
   public:
    PcConvertor(VelodyneConfig &config);

    ~PcConvertor() {}

    void SetDrPose(std::vector<common::TimedPose> &DR_pose_buffer);

    void ProcessScan(const PacketsMsgPtr &packets_msg, common::PointCloudXYZIT::Ptr &out_cloud);
    void ProcessScan(const PacketsMsgPtr &packets_msg, common::PointCloudType::Ptr &out_cloud);

    void SetMotionCompensatedEnableFlag(bool flag);

    VelodyneConfig GetConfig() const;

   private:
    void ConvertCloudType(const common::PointCloudXYZIT::Ptr &input_cloud, common::PointCloudType::Ptr &output_cloud);

   private:
    VelodyneConfig velodyne_config_;
    std::shared_ptr<PacketsParser> packets_parser_;

    common::PointCloudXYZIT::Ptr converted_cloud_;
    common::PointCloudXYZIT::Ptr compensated_cloud_;

    bool is_motion_compensation_enabled_ = true;
    std::shared_ptr<Compensator> compensator_;
};

}  // namespace mapping::tools

#endif  // MAPPING_PC_CONVERTOR_H
