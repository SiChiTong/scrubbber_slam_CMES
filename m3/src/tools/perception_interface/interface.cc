#include "tools/perception_interface/interface.h"
#include "tools/perception_interface/pcl_compensator.h"

#include "thirdparty/velodyne/include/lidar_transform.h"
#include "thirdparty/velodyne/include/packet_data.h"
#include "thirdparty/velodyne/include/packet_process.h"
#include "thirdparty/velodyne/include/point_types.h"
#include "thirdparty/velodyne/include/velodyne_constant.h"
#include "thirdparty/velodyne/include/velodyne_lidar_model.h"

namespace mapping::tools {

using namespace driver::velodyne;

void VelodyneConfig::LoadFromYAML(const io::YAML_IO& yaml) {
    xoffset = yaml.GetValue<double>("velodyne_calib_param", "xoffset");
    yoffset = yaml.GetValue<double>("velodyne_calib_param", "yoffset");
    zoffset = yaml.GetValue<double>("velodyne_calib_param", "zoffset");
    roll = yaml.GetValue<double>("velodyne_calib_param", "roll");
    pitch = yaml.GetValue<double>("velodyne_calib_param", "pitch");
    yaw = yaml.GetValue<double>("velodyne_calib_param", "yaw");
    type = yaml.GetValue<int>("velodyne_calib_param", "type");
    pre_rot_axis_0 = yaml.GetValue<int>("velodyne_calib_param", "pre_rot_axis_0");
    pre_rot_axis_1 = yaml.GetValue<int>("velodyne_calib_param", "pre_rot_axis_1");
    pre_rot_axis_2 = yaml.GetValue<int>("velodyne_calib_param", "pre_rot_axis_2");
    pre_rot_degree_0 = yaml.GetValue<double>("velodyne_calib_param", "pre_rot_degree_0");
    pre_rot_degree_1 = yaml.GetValue<double>("velodyne_calib_param", "pre_rot_degree_1");
    pre_rot_degree_2 = yaml.GetValue<double>("velodyne_calib_param", "pre_rot_degree_2");
    is_irad = yaml.GetValue<bool>("velodyne_calib_param", "is_irad");
}

PerceptionInterface::PerceptionInterface(const io::YAML_IO& yaml)
    : lidar_mode_(new VelodyneVLP16()),
      lidar_type_32_(new VelodyneVLP32()),
      lidar_type_64_(new VelodyneVLPHDL64()),
      last_azimuth_(36000),
      start_angle_(18000),
      last_azimuth_32_(36000),
      start_angle_32_(18000),
      compensator_(new PCLPointsCompensator) {
    velodyne_config_.LoadFromYAML(yaml);
    cloud_.reset(new PointCloudXYZRRIART());
    cloud_32_.reset(new PointCloudXYZRRIAR());
    cloud_64_.reset(new PointCloudXYZRRIAR());
    inner_time_ = &driver::velodyne::INNER_TIME_16;
    inner_time_32_ = &driver::velodyne::INNER_TIME_32C;
    inner_time_64_ = &driver::velodyne::INNER_TIME_64E_S3;
    is_64e_s2_ = false;

    compensator_->SetParam(velodyne_config_.max_range, velodyne_config_.min_range);

    start_angle_ = static_cast<uint16_t>(static_cast<int>(fabs(90) * 100) % (kRotationMaxUnits - 200));

    start_angle_32_ = static_cast<uint16_t>(static_cast<int>(fabs(180) * 100) % (kRotationMaxUnits - 200));

    lidar_transform_.reset(new LidarTransform(velodyne_config_.roll, velodyne_config_.pitch, velodyne_config_.yaw,
                                              velodyne_config_.xoffset, velodyne_config_.yoffset,
                                              velodyne_config_.zoffset));

    lidar_transform_32_.reset(new LidarTransform(velodyne_config_.roll, velodyne_config_.pitch, velodyne_config_.yaw,
                                                 velodyne_config_.xoffset, velodyne_config_.yoffset,
                                                 velodyne_config_.zoffset));

    lidar_transform_64_.reset(new LidarTransform(velodyne_config_.roll, velodyne_config_.pitch, velodyne_config_.yaw,
                                                 velodyne_config_.xoffset, velodyne_config_.yoffset,
                                                 velodyne_config_.zoffset));
}

PerceptionInterface::PerceptionInterface(const VelodyneConfig& velodyne_config)
    : lidar_mode_(new VelodyneVLP16()),
      lidar_type_32_(new VelodyneVLP32()),
      lidar_type_64_(new VelodyneVLPHDL64()),
      last_azimuth_(36000),
      start_angle_(18000),
      last_azimuth_32_(36000),
      start_angle_32_(18000) {
    velodyne_config_ = velodyne_config;
    cloud_.reset(new PointCloudXYZRRIART());
    cloud_32_.reset(new PointCloudXYZRRIAR());
    cloud_64_.reset(new PointCloudXYZRRIAR());
    inner_time_ = &driver::velodyne::INNER_TIME_16;
    inner_time_32_ = &driver::velodyne::INNER_TIME_32C;
    inner_time_64_ = &driver::velodyne::INNER_TIME_64E_S3;
    is_64e_s2_ = false;

    compensator_->SetParam(velodyne_config_.max_range, velodyne_config_.min_range);

    start_angle_ = static_cast<uint16_t>(static_cast<int>(fabs(90) * 100) % (kRotationMaxUnits - 200));

    start_angle_32_ = static_cast<uint16_t>(static_cast<int>(fabs(180) * 100) % (kRotationMaxUnits - 200));

    lidar_transform_.reset(new LidarTransform(velodyne_config_.roll, velodyne_config_.pitch, velodyne_config_.yaw,
                                              velodyne_config_.xoffset, velodyne_config_.yoffset,
                                              velodyne_config_.zoffset));

    lidar_transform_32_.reset(new LidarTransform(velodyne_config_.roll, velodyne_config_.pitch, velodyne_config_.yaw,
                                                 velodyne_config_.xoffset, velodyne_config_.yoffset,
                                                 velodyne_config_.zoffset));

    lidar_transform_64_.reset(new LidarTransform(velodyne_config_.roll, velodyne_config_.pitch, velodyne_config_.yaw,
                                                 velodyne_config_.xoffset, velodyne_config_.yoffset,
                                                 velodyne_config_.zoffset));
}

void PerceptionInterface::SetDrPose(std::vector<common::TimedPose>& DR_pose_buffer) {
    compensator_->SetDrPose(DR_pose_buffer);
}

void PerceptionInterface::PointConvertAndHeightProcess(std::vector<PacketsMsgPtr>& three_packets,
                                                       common::PointCloudXYZIHIRBS::Ptr& full_cloud) {
    PointCloudXYZRRIART::Ptr out_cloud(new PointCloudXYZRRIART);
    ProcessScan(three_packets, out_cloud);
    PointCloudXYZRRIARTH::Ptr cloud_xyzrriarth(new PointCloudXYZRRIARTH());
    perception_lib::PointHeightProcess(*out_cloud, *cloud_xyzrriarth);

    compensator_->Perform(cloud_xyzrriarth, full_cloud);
}

void PerceptionInterface::ProcessScan(std::vector<PacketsMsgPtr>& three_packets, PointCloudXYZRRIART::Ptr& out_cloud) {
    TimeHDLDataPacket timed_packet;
    cloud_->clear();

    std::vector<PacketsMsgPtr> two_packets;

    if (fabs(three_packets[1]->header.stamp.toSec() - three_packets[0]->header.stamp.toSec()) < 0.001) {
        two_packets.push_back(three_packets[1]);
        two_packets.push_back(three_packets[2]);
    } else if (fabs(three_packets[2]->header.stamp.toSec() - three_packets[1]->header.stamp.toSec()) < 0.001) {
        two_packets.push_back(three_packets[1]);
        two_packets.push_back(three_packets[2]);
    } else {
        TimeHDLDataPacket first_packet;
        memcpy(reinterpret_cast<void*>(&first_packet.packet), &(three_packets[0]->packets[0].data[0]),
               sizeof(HDLDataPacket));
        HDLDataPacket* data_packet = &first_packet.packet;
        HDLFiringData* firing_data = &(data_packet->firing_data[0]);
        auto current_azimuth = firing_data->azimuth;
        if (current_azimuth < (start_angle_ + 18000)) {
            two_packets.push_back(three_packets[1]);
            two_packets.push_back(three_packets[2]);
        } else {
            two_packets.push_back(three_packets[1]);
            two_packets.push_back(three_packets[2]);
        }
    }
    for (auto one_packets : two_packets) {
        for (const velodyne_msgs::VelodynePacket packet : one_packets->packets) {
            ros::Time packet_time = packet.stamp;
            memcpy(reinterpret_cast<void*>(&timed_packet.packet), &packet.data[0], sizeof(HDLDataPacket));
            timed_packet.nsec = packet_time.toNSec();
            if (ProcessPacket(&timed_packet.packet, timed_packet.nsec)) {
                out_cloud = cloud_;
                return;
            }
        }
    }
    if (out_cloud->points.size() == 0) {
        LOG(ERROR) << "Can't compute valid cloud.";
    }
}

bool PerceptionInterface::ProcessPacket(const HDLDataPacket* data_packet, uint64_t nsec) {
    int azimuth_diff = 40;
    const uint64_t block_offset_nsec = static_cast<uint64_t>(110.592 * 1000);

    for (size_t firing_block = 0; firing_block < kFiringPerPacket; ++firing_block) {
        const HDLFiringData* firing_data = &(data_packet->firing_data[firing_block]);

        auto current_azimuth = firing_data->azimuth;
        if (AtEndScanningBoundary(current_azimuth)) {
            ros::Time cloud_last_point_stamp;
            const uint64_t cloud_nsec = nsec - firing_block * block_offset_nsec;
            cloud_last_point_stamp.fromNSec(cloud_nsec);
            cloud_->header.stamp = pcl_conversions::toPCL(cloud_last_point_stamp);
            cloud_->header.frame_id = "map";
            cloud_->height = 16;
            cloud_->width = cloud_->points.size() / cloud_->height;
            if (cloud_->points.size() < 28000) {
                cloud_->clear();
            }
            if (cloud_->points.size() > 28000) return true;
        }

        last_azimuth_ = current_azimuth;

        std::vector<PointXYZRRIART>* firing_points = new std::vector<PointXYZRRIART>();
        firing_points->resize(kLaserPerFiring);
        ros::Time temp_time;
        temp_time.fromNSec(nsec);
        ProcessFiring(*firing_points, firing_data, firing_block, azimuth_diff, temp_time.toSec());
        cloud_->points.insert(cloud_->points.end(), firing_points->begin(), firing_points->end());

        delete firing_points;
    }

    return false;
}

void PerceptionInterface::ProcessFiring(std::vector<PointXYZRRIART>& firing_points, const HDLFiringData* firing_data,
                                        int block, int azimuth_diff, double pkt_time) {
    uint16_t azimuth = firing_data->azimuth;
    uint16_t azimuth_corrected = 0;

    const std::array<float, kLaserPerFiring>& azimuth_corrected_table = lidar_mode_->get_azimuth_corrected_table();
    const std::array<int, kLaserPerFiring>& firing_sequence = lidar_mode_->get_firing_laser_sequence();
    const float& distance_resolution = lidar_mode_->get_distance_resolution();
    const std::array<float, kLaserPerFiring>& vert_angle_table_cos = lidar_mode_->get_vert_angle_table_cos();
    const std::array<float, kLaserPerFiring>& vert_angle_table_sin = lidar_mode_->get_vert_angle_table_sin();
    const std::array<float, kRotationMaxUnits>& rot_table_cos = lidar_mode_->get_rot_table_cos();
    const std::array<float, kRotationMaxUnits>& rot_table_sin = lidar_mode_->get_rot_table_sin();

    // loop calculate the every firing points
    for (size_t dsr = 0; dsr < kLaserPerFiring; ++dsr) {
        azimuth_corrected = azimuth + static_cast<uint16_t>(azimuth_diff * azimuth_corrected_table[dsr]);
        azimuth_corrected = static_cast<uint16_t>(round(static_cast<double>(azimuth_corrected))) %
                            kRotationMaxUnits;  // kRotationMaxUnits=36000
        // azimuth_corrected = (int)round(fmod(azimuth_corrected_f, 36000.0));

        PointXYZRRIART& point = firing_points[firing_sequence[dsr]];

        point.time = pkt_time + (*inner_time_)[block][dsr] * 1e-6;
        // point ring
        point.ring = static_cast<uint8_t>(firing_sequence[dsr] % 16);
        // point angle
        point.angle = azimuth_corrected;

        const HDLLaserReturn& laser_returns = firing_data->laser_returns[dsr];
        // point intensity
        point.intensity = laser_returns.intensity;
        // reset point x,y,z,range,radius
        point.x = point.y = point.z = point.range = point.radius = kLaserHitFree;

        float distance = kLaserHitFree;
        float xy_distance = kLaserHitFree;

        if (laser_returns.distance != 0) {
            // calculate xyz coordinate
            distance = laser_returns.distance * distance_resolution;

            // Compute the distance in the xy plane (w/o accounting for rotation)
            xy_distance = distance * vert_angle_table_cos[dsr];

            // Use standard ROS coordinate system (right-hand rule)
            point.x = xy_distance * rot_table_cos[azimuth_corrected];
            point.y = -xy_distance * rot_table_sin[azimuth_corrected];
            point.z = distance * vert_angle_table_sin[dsr];
            point.radius = xy_distance;
            point.range = distance;
        }

        lidar_transform_->TransformCloud<PointXYZRRIART>(&point);
    }
}

void PerceptionInterface::AdjustPointClouds(PointCloudXYZRRIAR& input_cloud,
                                            common::PointCloudXYZIHIRBS::Ptr& output_cloud) {
    common::PointXYZIHIRBS new_point;
    for (const auto& point : input_cloud.points) {
        if (isPointValid(point) && isPointInRange(point)) {
            new_point.x = point.x;
            new_point.y = point.y;
            new_point.z = point.z;
            new_point.intensity = point.intensity;
            new_point.intensity_vi = point.intensity;
            new_point.height = 0;
            new_point.range = 0;
            new_point.intensity_vi = point.intensity;
            new_point.ring = point.ring;
            new_point.type = 0;

            output_cloud->push_back(new_point);
        }
    }
    output_cloud->header = input_cloud.header;
    output_cloud->height = 1;
    output_cloud->width = output_cloud->points.size();
}

void PerceptionInterface::ProcessScan32(PacketsMsgPtr& packets_ptr, common::PointCloudXYZIHIRBS::Ptr& out_cloud) {
    TimeHDLDataPacket timed_packet;
    cloud_32_->points.clear();

    ////test gh
    double scan_time = packets_ptr->header.stamp.toSec();
    min_point_time_ = -1;
    max_point_time_ = -1;
    int count = 0;

    for (const velodyne_msgs::VelodynePacket packet : packets_ptr->packets) {
        count++;
        ros::Time packet_time = packet.stamp;
        memcpy(reinterpret_cast<void*>(&timed_packet.packet), &packet.data[0], sizeof(HDLDataPacket));
        timed_packet.nsec = packet_time.toNSec();
        ProcessPacket32(&timed_packet.packet, timed_packet.nsec);
    }
    cloud_32_->header.stamp = pcl_conversions::toPCL(packets_ptr->header.stamp);
    cloud_32_->height = lidar_type_32_->get_ring_count();
    cloud_32_->width = cloud_32_->points.size() / cloud_32_->height;

    // common::PointCloudXYZIHIRBS::Ptr points_converted(new common::PointCloudXYZIHIRBS());
    // AdjustPointClouds(*cloud_32_, points_converted);

    //motion compensator 
    ///test gh
    PointCloudXYZRRIAR::Ptr points_converted_compen(new PointCloudXYZRRIAR());
    compensator_->Perform32Compensation(cloud_32_, points_converted_compen, min_point_time_, max_point_time_);
    
    LOG(INFO) << std::setprecision(16) << scan_time << " "<<min_point_time_<<" "<<max_point_time_<<" "<<max_point_time_ - min_point_time_;
  
    common::PointCloudXYZIHIRBS::Ptr points_converted(new common::PointCloudXYZIHIRBS());
    AdjustPointClouds(*points_converted_compen, points_converted);

    *out_cloud = *points_converted;

    if (out_cloud->points.size() == 0) LOG(ERROR) << "Can't compute valid cloud.";
}

bool PerceptionInterface::ProcessPacket32(const HDLDataPacket* data_packet, uint64_t nsec) {
    float azimuth_diff = lidar_type_32_->get_firing_azimuth_diff();
    for (size_t firing_block = 0; firing_block < kFiringPerPacket; ++firing_block) {
        const HDLFiringData* firing_data = &(data_packet->firing_data[firing_block]);

        auto current_azimuth = firing_data->azimuth;

        last_azimuth_32_ = current_azimuth;

        ros::Time packet_time;
        packet_time.fromNSec(nsec);

        FiringCloudPtr firing_points = new FiringCloud();
        firing_points->resize(kLaserPerFiring);
        ProcessFiring32(*firing_points, firing_data, firing_block, azimuth_diff, packet_time.toSec());
        cloud_32_->points.insert(cloud_32_->points.end(), firing_points->begin(), firing_points->end());

        delete firing_points;
    }

    return false;
}

void PerceptionInterface::ProcessFiring32(std::vector<PointXYZRRIAR>& firing_points, const HDLFiringData* firing_data,
                                          int block, float azimuth_diff, double pkt_time) {
    uint16_t azimuth = firing_data->azimuth;
    uint16_t azimuth_corrected = 0;

    const std::array<float, kLaserPerFiring>& azimuth_corrected_table = lidar_type_32_->get_azimuth_corrected_table();
    const std::array<int, kLaserPerFiring>& azimuth_offsets = lidar_type_32_->get_azimuth_offset_table();
    const std::array<int, kLaserPerFiring>& firing_sequence = lidar_type_32_->get_firing_laser_sequence();
    const float& distance_resolution = lidar_type_32_->get_distance_resolution();
    const std::array<float, kLaserPerFiring>& vert_angle_table_cos = lidar_type_32_->get_vert_angle_table_cos();
    const std::array<float, kLaserPerFiring>& vert_angle_table_sin = lidar_type_32_->get_vert_angle_table_sin();
    const std::array<float, kRotationMaxUnits>& rot_table_cos = lidar_type_32_->get_rot_table_cos();
    const std::array<float, kRotationMaxUnits>& rot_table_sin = lidar_type_32_->get_rot_table_sin();
    const int& ring_count = lidar_type_32_->get_ring_count();

    // loop calculate the every firing points
    for (size_t dsr = 0; dsr < kLaserPerFiring; ++dsr) {
        azimuth_corrected = static_cast<uint16_t>((static_cast<int>(azimuth) +
                                                   static_cast<int>(azimuth_diff * azimuth_corrected_table[dsr]) +
                                                   azimuth_offsets[dsr] + kRotationMaxUnits) %
                                                  kRotationMaxUnits);
        // azimuth_corrected =
        //     static_cast<uint16_t>(round(static_cast<double>(azimuth_corrected)))
        //     % kRotationMaxUnits;  // kRotationMaxUnits=36000
        // azimuth_corrected = (int)round(fmod(azimuth_corrected_f, 36000.0));

        driver::velodyne::PointXYZRRIAR& point = firing_points[firing_sequence[dsr]];
        // point ring
        point.ring = static_cast<uint8_t>(firing_sequence[dsr] % ring_count);
        // point angle
        point.angle = azimuth_corrected;

        const HDLLaserReturn& laser_returns = firing_data->laser_returns[dsr];
        // point intensity
        point.intensity = laser_returns.intensity;
        // reset point x,y,z,range,radius
        point.x = point.y = point.z = point.range = point.radius = kLaserHitFree;

        float distance = kLaserHitFree;
        float xy_distance = kLaserHitFree;

        if (laser_returns.distance != 0) {
            // calculate xyz coordinate
            distance = laser_returns.distance * distance_resolution;

            // Compute the distance in the xy plane (w/o accounting for rotation)
            xy_distance = distance * vert_angle_table_cos[dsr];

            // Use standard ROS coordinate system (right-hand rule)
            point.x = xy_distance * rot_table_cos[azimuth_corrected];
            point.y = -xy_distance * rot_table_sin[azimuth_corrected];
            point.z = distance * vert_angle_table_sin[dsr];
            point.radius = xy_distance;
            point.range = distance;
            point.timestamp = pkt_time + (*inner_time_32_)[block][dsr] * 1e-6;

            ///test gh
            if(min_point_time_ < 0){
                min_point_time_ = point.timestamp;
            } else if(point.timestamp < min_point_time_){
                min_point_time_ = point.timestamp;
            }

            if(max_point_time_ < 0){
                max_point_time_ = point.timestamp;
            } else if(point.timestamp > max_point_time_){
                max_point_time_ = point.timestamp;
            }
        }

        lidar_transform_32_->TransformCloud<PointXYZRRIAR>(&point);
    }
}

void PerceptionInterface::ProcessScan64(PacketsMsgPtr& packets_ptr, common::PointCloudXYZIHIRBS::Ptr& out_cloud) {
    const std::array<uint64_t, 4>& gps_base_usec = lidar_type_64_->get_gps_base_usec();
    double scan_time = packets_ptr->header.stamp.toSec();
    cloud_64_->points.clear();
    min_point_time_ = -1;
    max_point_time_ = -1;
    for (const velodyne_msgs::VelodynePacket packet : packets_ptr->packets) {
        const HDL64RawPacket* pkt = (const HDL64RawPacket*)&packet.data[0];
        const double pkt_time = ros::Time(packet.stamp).toSec(); 
        if (gps_base_usec[0] == 0) {
            lidar_type_64_->set_base_time_from_packets(packet);
        }
        ProcessPacket64(pkt, pkt_time);
    }
    cloud_64_->header.stamp = pcl_conversions::toPCL(packets_ptr->header.stamp);
    cloud_64_->height = lidar_type_64_->get_ring_count();
    cloud_64_->width = cloud_64_->points.size() / cloud_64_->height;

    // 和32线相同
    PointCloudXYZRRIAR::Ptr points_converted_compen(new PointCloudXYZRRIAR());
    compensator_->Perform32Compensation(cloud_64_, points_converted_compen, min_point_time_, max_point_time_);
    LOG(INFO) << std::setprecision(16) << scan_time << " " << min_point_time_ 
                                       << " " << max_point_time_ << " " 
                                       << max_point_time_ - min_point_time_;

    common::PointCloudXYZIHIRBS::Ptr points_converted(new common::PointCloudXYZIHIRBS());
    AdjustPointClouds(*points_converted_compen, points_converted);
    *out_cloud = *points_converted;

    if (out_cloud->points.size() == 0) LOG(ERROR) << "Can't compute valid cloud.";
}

bool PerceptionInterface::ProcessPacket64(const HDL64RawPacket* data_packet, const double& pkt_time) {
    double basetime = data_packet->gps_timestamp;  // usec
    const std::array<float, kLaserPerFiring64>& dist_correction = lidar_type_64_->get_64_dist_correction();
    const std::array<int, kLaserPerFiring64>& max_intensity = lidar_type_64_->get_64_max_intensity();
    const std::array<int, kLaserPerFiring64>& min_intensity = lidar_type_64_->get_64_min_intensity();
    const std::array<float, kLaserPerFiring64>& focal_slope = lidar_type_64_->get_64_focal_slope();
    const std::array<float, kLaserPerFiring64>& focal_offset = lidar_type_64_->get_64_focal_offset();

    for (int i = 0; i < kFiringPerPacket; ++i) {  // 12
        if (!is_64e_s2_ && ((i & 3) >> 1) > 0) {
             continue;
        }

        int bank_origin = (data_packet->blocks[i].laser_block_id == BLOCK_32_TO_63) ? 32 : 0;
        for (int j = 0, k = 0; j < kLaserPerFiring; ++j, k += kPerLaserByteSize) {  // 32, 3
            // One point
            uint8_t laser_number = j + bank_origin;  // hardware laser number
            union driver::velodyne::RawDistance raw_distance;
            raw_distance.bytes[0] = data_packet->blocks[i].data[k];
            raw_distance.bytes[1] = data_packet->blocks[i].data[k + 1];
            // compute time
            double timestamp = 0.0;
            if (!enable_basetime_) {
                timestamp = pkt_time + (*inner_time_64_)[i][j] * 1e-6;
            } else {
                // const std::array<double, 4>& previous_packet_stamp_array = lidar_type_64_->get_previous_packet_stamp();
                // const std::array<uint64_t, 4>& gps_base_usec_array = lidar_type_64_->get_gps_base_usec();
                // double t = basetime - (*inner_time_64_)[i][j];
                // if (is_64e_s2_) {
                //     int index = i & 1;  // % 2
                //     double& previous_packet_stamp = previous_packet_stamp_array[index];
                //     uint64_t& gps_base_usec = gps_base_usec_array[index];
                //     timestamp = lidar_type_64_->get_gps_stamp(t, previous_packet_stamp, gps_base_usec);
                // } else {                 // 64E_S3
                //     int index = i & 3;  // % 4
                //     double& previous_packet_stamp = previous_packet_stamp_array[index];
                //     uint64_t& gps_base_usec = gps_base_usec_array[index];
                //     timestamp = lidar_type_64_->get_gps_stamp(t, previous_packet_stamp, gps_base_usec);
                // }
            }
            if (j == kLaserPerFiring - 1) {
                cloud_64_->header.stamp = static_cast<uint64_t>(timestamp * 1000000);
            }
            float distance = raw_distance.raw_distance * kLaserDistanceResolutionVLP64 + dist_correction[laser_number];
            if (raw_distance.raw_distance == 0 ||                              \
                !isPointInRange(data_packet->blocks[i].rotation, distance)) {
                continue;
            }

            driver::velodyne::PointXYZRRIAR point;
            point.timestamp = timestamp;
            // Position Calculation, append this point to the cloud
            ComputeCoords(raw_distance, laser_number, data_packet->blocks[i].rotation, point);

            if (min_point_time_ < 0) {
                min_point_time_ = point.timestamp;
            } else if (point.timestamp < min_point_time_) {
                min_point_time_ = point.timestamp;
            }

            if (max_point_time_ < 0) {
                max_point_time_ = point.timestamp;
            } else if (point.timestamp > max_point_time_) {
                max_point_time_ = point.timestamp;
            }

            int intensity = data_packet->blocks[i].data[k + 2];
            {
                float tmp = 1 - static_cast<float>(raw_distance.raw_distance) / 65535;
                intensity += focal_slope[laser_number] *
                            (fabs(focal_offset[laser_number] - 256 * tmp * tmp));

                if (intensity < min_intensity[laser_number]) {
                    intensity = min_intensity[laser_number];
                }

                if (intensity > max_intensity[laser_number]) {
                    intensity = max_intensity[laser_number];
                }
                point.intensity = intensity;
            }
            if (point.x < velodyne_config_.car_front && point.x > velodyne_config_.car_back &&     \
                point.y < velodyne_config_.car_left && point.y > velodyne_config_.car_right &&     \
                point.z < velodyne_config_.car_top && point.z > velodyne_config_.car_bottom) {
                continue;
            }
            // append this point to the cloud
            cloud_64_->points.emplace_back(point);
        }
    }
}

void PerceptionInterface::ComputeCoords(const union driver::velodyne::RawDistance &raw_distance,
                                        const int &laser_number,
                                        const uint16_t &rotation, PointXYZRRIAR &point,
                                        const float distance_resolution) {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      const std::array<float, kRotationMaxUnits>& rot_table_cos = lidar_type_64_->get_rot_table_cos();
      const std::array<float, kRotationMaxUnits>& rot_table_sin = lidar_type_64_->get_rot_table_sin();
      const std::array<float, kLaserPerFiring64>& dist_correction = lidar_type_64_->get_64_dist_correction();
      const std::array<float, kLaserPerFiring64>& rot_correction = lidar_type_64_->get_64_rot_correction();
      const std::array<float, kLaserPerFiring64>& cos_rot_correction = lidar_type_64_->get_64_cos_rot_correction();
      const std::array<float, kLaserPerFiring64>& sin_rot_correction = lidar_type_64_->get_64_sin_rot_correction();
      const std::array<float, kLaserPerFiring64>& cos_vert_correction = lidar_type_64_->get_64_cos_vert_correction();
      const std::array<float, kLaserPerFiring64>& sin_vert_correction = lidar_type_64_->get_64_sin_vert_correction();
      const std::array<float, kLaserPerFiring64>& horiz_offset_correction = lidar_type_64_->get_64_horiz_offset_correction();
      const std::array<float, kLaserPerFiring64>& vert_offset_correction = lidar_type_64_->get_64_vert_offset_correction();
      const std::array<float, kLaserPerFiring64>& dist_correction_x = lidar_type_64_->get_64_dist_correction_x();
      const std::array<float, kLaserPerFiring64>& dist_correction_y = lidar_type_64_->get_64_dist_correction_y();
      const std::array<int, kLaserPerFiring64>& laser_ring = lidar_type_64_->get_64_laser_ring();
      double distance1 = raw_distance.raw_distance * distance_resolution;
      double distance = distance1 + dist_correction[laser_number];

      double cos_rot_angle =
        rot_table_cos[rotation] * cos_rot_correction[laser_number] +   \
        rot_table_sin[rotation] * sin_rot_correction[laser_number];
      double sin_rot_angle =
        rot_table_sin[rotation] * cos_rot_correction[laser_number] -   \
        rot_table_cos[rotation] * sin_rot_correction[laser_number];

      double xy_distance = distance * cos_vert_correction[laser_number];
      double xx = fabs(xy_distance * sin_rot_angle -     \
                       horiz_offset_correction[laser_number] * cos_rot_angle);
      double yy = fabs(xy_distance * cos_rot_angle +     \
                       horiz_offset_correction[laser_number] * sin_rot_angle);

      double distance_corr_x = 0;
      double distance_corr_y = 0;

      if (need_two_pt_correction_ && distance1 <= 2500) {
        distance_corr_x =
          (dist_correction[laser_number] - dist_correction_x[laser_number]) *   \
          (xx - 2.4) / 22.64 + dist_correction_x[laser_number];  // 22.64 = 25.04 - 2.4
        distance_corr_y =
          (dist_correction[laser_number] - dist_correction_y[laser_number]) *   \
          (yy - 1.93) / 23.11 + dist_correction_y[laser_number];  // 23.11 = 25.04 - 1.93
      } else {
        distance_corr_x = distance_corr_y = dist_correction[laser_number];
      }

      double distance_x = distance1 + distance_corr_x;
      xy_distance = distance_x * cos_vert_correction[laser_number];
      x = xy_distance * sin_rot_angle -    \
          horiz_offset_correction[laser_number] * cos_rot_angle;

      double distance_y = distance1 + distance_corr_y;
      xy_distance = distance_y * cos_vert_correction[laser_number];
      y = xy_distance * cos_rot_angle +    \
          horiz_offset_correction[laser_number] * sin_rot_angle;
      z = distance * sin_vert_correction[laser_number] +    \
          vert_offset_correction[laser_number];

      point.x = static_cast<float>(y);
      point.y = static_cast<float>(-x);
      point.z = static_cast<float>(z);
      point.ring = static_cast<uint8_t>(laser_ring[laser_number]);

      float angle = rotation * 1.0 / 100 - rot_correction[laser_number] * 180.0 / M_PI;
      angle = angle >= 0.0 ? angle : angle + 360.0;
      angle = angle <= 360.0 ? angle : angle - 360.0;

      point.angle = static_cast<int>(100 * angle);

      lidar_transform_64_->TransformCloud<PointXYZRRIAR>(&point);
}

}  // namespace mapping::tools