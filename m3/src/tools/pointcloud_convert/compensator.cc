//
// Created by wangqi on 19-7-25.
//

#include "tools/pointcloud_convert/compensator.h"
#include <glog/logging.h>

namespace mapping::tools {

using namespace mapping::common;

Compensator::Compensator()
    : x_offset_(-1), y_offset_(-1), z_offset_(-1), timestamp_offset_(-1), timestamp_data_size_(0) {
    is_motion_compensation_valid_ = false;
}

void Compensator::SetDrPose(std::vector<TimedPose>& DR_pose_buffer) {
    for (int i = 0; i < DR_pose_buffer.size(); i++) {
        TimedPose poseDR = DR_pose_buffer[i];
        pose_buffer_.push(poseDR);
    }
}

void Compensator::Perform(sensor_msgs::PointCloud2ConstPtr origin_cloud,
                          sensor_msgs::PointCloud2::Ptr compensated_cloud) {
    if (!CheckMessage(origin_cloud)) {
        return;
    }

    Aff3f min_time_pose;
    Aff3f max_time_pose;

    double min_timestamp = 0.0;
    double max_timestamp = 0.0;
    ComputeTimestampInterval(origin_cloud, min_timestamp, max_timestamp);

    is_motion_compensation_valid_ =
        FindCorrespondPoseFrom(pose_buffer_, min_timestamp, max_timestamp, min_time_pose, max_time_pose);
    
    if (is_motion_compensation_valid_) {
        *compensated_cloud = *origin_cloud;

        MotionCompensation<float>(compensated_cloud, min_timestamp, max_timestamp, min_time_pose, max_time_pose);
    } else {
        *compensated_cloud = *origin_cloud;
        // LOG(WARNING)<<"[Compensator]: motion compensation is unvalid.";
    }
}

bool Compensator::CheckMessage(sensor_msgs::PointCloud2ConstPtr msg) {
    if (msg->width == 0 || msg->height == 0) {
        return false;
    }

    int x_data_type = 0;
    int y_data_type = 0;
    int z_data_type = 0;

    for (size_t i = 0; i < msg->fields.size(); ++i) {
        const sensor_msgs::PointField& f = msg->fields[i];

        if (f.name == "x") {
            x_offset_ = f.offset;
            x_data_type = f.datatype;
            if ((x_data_type != 7 && x_data_type != 8) || f.count != 1 || x_offset_ == -1) {
                return false;
            }
        } else if (f.name == "y") {
            y_offset_ = f.offset;
            y_data_type = f.datatype;
            if (f.count != 1 || y_offset_ == -1) {
                return false;
            }
        } else if (f.name == "z") {
            z_offset_ = f.offset;
            z_data_type = f.datatype;
            if (f.count != 1 || z_offset_ == -1) {
                return false;
            }
        } else if (f.name == "timestamp") {
            timestamp_offset_ = f.offset;
            timestamp_data_size_ = f.count * GetFieldSize(f.datatype);
            if (timestamp_offset_ == -1 || timestamp_data_size_ == -1) {
                return false;
            }
        }
    }

    if (x_offset_ == -1 || y_offset_ == -1 || z_offset_ == -1 || timestamp_offset_ == -1 ||
        timestamp_data_size_ == -1) {
        return false;
    }
    if (!(x_data_type == y_data_type && y_data_type == z_data_type)) {
        return false;
    }
    return true;
}

void Compensator::ComputeTimestampInterval(sensor_msgs::PointCloud2ConstPtr origin_cloud, double& min_timestamp,
                                           double& max_timestamp) {
    max_timestamp = 0.0;
    min_timestamp = std::numeric_limits<double>::max();
    int total = origin_cloud->width * origin_cloud->height;

    for (int i = 0; i < total; ++i) {
        double timestamp = 0.0;
        memcpy(&timestamp, &origin_cloud->data[i * origin_cloud->point_step + timestamp_offset_], timestamp_data_size_);

        if (timestamp < min_timestamp) {
            min_timestamp = timestamp;
        }
        if (timestamp > max_timestamp) {
            max_timestamp = timestamp;
        }
    }
}

bool Compensator::FindCorrespondPoseFrom(CircularBuffer<TimedPose>& pose_buffer, double min_timestamp,
                                         double max_timestamp, Aff3f& min_time_pose, Aff3f& max_time_pose) {
    size_t buffer_size = pose_buffer.size();

    if (buffer_size == 0) {
        // LOG(WARNING) << "No locpos !!!!";
        return false;
    }

    LOG(INFO) << std::setprecision(16) << "min_timestamp: " << pose_buffer.size() << " " << min_timestamp << " "
              << max_timestamp << " " << pose_buffer[0].time << " " << pose_buffer[buffer_size - 1].time;

    if (max_timestamp - min_timestamp > 0.15 || max_timestamp - min_timestamp < 0.085) {
        LOG(WARNING) << "The scan time between two pointcloud is wrong.";
        return false;
    }

    double tmin_min = 10.0;
    double tmax_min = 10.0;
    std::map<double, int> min_time_diff_idx, max_time_diff_idx;

    for (size_t i = 0; i < buffer_size; i++) {
        double DR_pose_timestamp = pose_buffer[i].time;
        double temp1 = DR_pose_timestamp - min_timestamp;
        if (temp1 < 0) temp1 = -temp1;
        double temp2 = DR_pose_timestamp - max_timestamp;
        if (temp2 < 0) temp2 = -temp2;
        min_time_diff_idx.insert(std::pair<double, int>(temp1, i));
        max_time_diff_idx.insert(std::pair<double, int>(temp2, i));
        if (temp1 < tmin_min) tmin_min = temp1;
        if (temp2 < tmax_min) tmax_min = temp2;
    }

    int min_time_pose_idx = min_time_diff_idx.find(tmin_min)->second;
    int max_time_pose_idx = max_time_diff_idx.find(tmax_min)->second;

    LOG(INFO) <<"Compensator: "<<min_time_pose_idx<<" "<<max_time_pose_idx<<" "<<buffer_size;

    if (min_time_pose_idx < buffer_size && min_time_pose_idx >= 0 && max_time_pose_idx < buffer_size &&
        max_time_pose_idx >= 0) {
        min_time_pose = Aff3f(pose_buffer[min_time_pose_idx].pose.matrix().cast<float>());
        max_time_pose = Aff3f(pose_buffer[max_time_pose_idx].pose.matrix().cast<float>());

        return true;
    } else {
        return false;
    }
}

uint Compensator::GetFieldSize(const int data_type) {
    switch (data_type) {
        case sensor_msgs::PointField::INT8:
        case sensor_msgs::PointField::UINT8:
            return 1;

        case sensor_msgs::PointField::INT16:
        case sensor_msgs::PointField::UINT16:
            return 2;

        case sensor_msgs::PointField::INT32:
        case sensor_msgs::PointField::UINT32:
        case sensor_msgs::PointField::FLOAT32:
            return 4;

        case sensor_msgs::PointField::FLOAT64:
            return 8;

        default:
            return 0;
    }
}

template <typename Scalar>
void Compensator::MotionCompensation(sensor_msgs::PointCloud2::Ptr& msg, const double min_timestamp,
                                     const double max_timestamp, const Aff3f& min_time_pose,
                                     const Aff3f& max_time_pose) {
    using std::abs;
    using std::acos;
    using std::sin;

    Eigen::Vector3f translation = min_time_pose.translation() - max_time_pose.translation();

    Eigen::Quaternionf q_max(max_time_pose.linear());
    Eigen::Quaternionf q_min(min_time_pose.linear());
    Eigen::Quaternionf q1(q_max.conjugate() * q_min);
    Eigen::Quaternionf q0(Eigen::Quaternionf::Identity());
    q1.normalize();
    translation = q_max.conjugate() * translation;

    int total = msg->width * msg->height;

    double d = q0.dot(q1);
    double abs_d = abs(d);
    double f = 1.0 / (max_timestamp - min_timestamp);

    const double theta = acos(abs_d);
    const double sin_theta = sin(theta);
    const double c1_sign = (d > 0) ? 1 : -1;

    for (int i = 0; i < total; ++i) {
        size_t offset = i * msg->point_step;
        Scalar* x_scalar = reinterpret_cast<Scalar*>(&msg->data[offset + x_offset_]);
        if (std::isnan(*x_scalar)) {
            continue;
        }
        Scalar* y_scalar = reinterpret_cast<Scalar*>(&msg->data[offset + y_offset_]);
        Scalar* z_scalar = reinterpret_cast<Scalar*>(&msg->data[offset + z_offset_]);
        Eigen::Vector3f p(*x_scalar, *y_scalar, *z_scalar);

        double tp = 0.0;
        memcpy(&tp, &msg->data[i * msg->point_step + timestamp_offset_], timestamp_data_size_);
        double t = (max_timestamp - tp) * f;

        Eigen::Translation3f ti(t * translation);

        if (abs_d < 1.0 - 1.0e-8) {
            double c0 = sin((1 - t) * theta) / sin_theta;
            double c1 = sin(t * theta) / sin_theta * c1_sign;
            Eigen::Quaternionf qi(c0 * q0.coeffs() + c1 * q1.coeffs());
            Aff3f trans = ti * qi;
            p = trans * p;
        } else {
            p = ti * p;
        }
        *x_scalar = p.x();
        *y_scalar = p.y();
        *z_scalar = p.z();
    }

    return;
}

void Compensator::PerformSuTeng80Compensation(RSCloudType ::Ptr origin_cloud, RSCloudType ::Ptr& compensated_cloud,
                                              const double& min_timestamp, const double& max_timestamp,
                                              double scan_time) {
    Aff3f min_time_pose;
    Aff3f max_time_pose;

    double scan_min_time = min_timestamp;
    double scan_max_time = max_timestamp;
    if (std::fabs(scan_time - scan_max_time) > 0.1) {
        double delta_time = scan_time - scan_max_time;
        scan_min_time = scan_min_time + delta_time;
        scan_max_time = scan_max_time + delta_time;
        LOG(INFO) << "Scan delta time: " << delta_time;
    }

    is_motion_compensation_valid_ =
        FindCorrespondPoseFrom(pose_buffer_, scan_min_time, scan_max_time, min_time_pose, max_time_pose);

    if (is_motion_compensation_valid_) {
        MotionCompensationSuTeng80(origin_cloud, compensated_cloud, min_timestamp, max_timestamp, min_time_pose,
                                   max_time_pose);
    } else {
        *compensated_cloud = *origin_cloud;
        LOG(WARNING) << "SuTeng compensation: motion compensation is unvalid.";
    }
}

void Compensator::MotionCompensationSuTeng80(RSCloudType::Ptr& msg, RSCloudType::Ptr& out_points,
                                             const double min_timestamp, const double max_timestamp,
                                             const Aff3f& min_time_pose, const Aff3f& max_time_pose) {
    using std::abs;
    using std::acos;
    using std::sin;

    Eigen::Vector3f translation = min_time_pose.translation() - max_time_pose.translation();

    LOG(INFO) <<"Motion translation: "<<translation[0]<<" "<<translation[1]<<" "<<max_timestamp - min_timestamp;

    Eigen::Quaternionf q_max(max_time_pose.linear());
    Eigen::Quaternionf q_min(min_time_pose.linear());
    Eigen::Quaternionf q1(q_max.conjugate() * q_min);
    Eigen::Quaternionf q0(Eigen::Quaternionf::Identity());
    q1.normalize();
    translation = q_max.conjugate() * translation;

    double d = q0.dot(q1);
    double abs_d = abs(d);
    double f = 1.0 / (max_timestamp - min_timestamp);

    const double theta = acos(abs_d);
    const double sin_theta = sin(theta);
    const double c1_sign = (d > 0) ? 1 : -1;

    for (auto& pt : msg->points) {
        if (isPointValidSuTeng<RSPoint>(pt) && isPointInRangeSuTeng<RSPoint>(pt)) {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            double t = (max_timestamp - pt.timestamp) * f;
            Eigen::Translation3f ti(t * translation);

            if (abs_d < 1.0 - 1.0e-8) {
                double c0 = sin((1 - t) * theta) / sin_theta;
                double c1 = sin(t * theta) / sin_theta * c1_sign;
                Eigen::Quaternionf qi(c0 * q0.coeffs() + c1 * q1.coeffs());
                Aff3f trans = ti * qi;
                p = trans * p;
            } else {
                p = ti * p;
            }
            RSPoint temp_point;
            temp_point = pt;
            temp_point.x = p.x();
            temp_point.y = p.y();
            temp_point.z = p.z();
            out_points->push_back(temp_point);
        } 
    }
    out_points->header = msg->header;
    out_points->height = 1;
    out_points->width = out_points->points.size();
}

void Compensator::PerformSuTeng80Compensation(IRADCloudType ::Ptr origin_cloud, IRADCloudType ::Ptr& compensated_cloud,
                                              const double& min_timestamp, const double& max_timestamp,
                                              double scan_time) {
    Aff3f min_time_pose;
    Aff3f max_time_pose;

    double scan_min_time = min_timestamp;
    double scan_max_time = max_timestamp;
    if (std::fabs(scan_time - scan_max_time) > 0.1) {
        double delta_time = scan_time - scan_max_time;
        scan_min_time = scan_min_time + delta_time;
        scan_max_time = scan_max_time + delta_time;
        LOG(INFO) << "Scan delta time: " << delta_time;
    }

    is_motion_compensation_valid_ =
        FindCorrespondPoseFrom(pose_buffer_, scan_min_time, scan_max_time, min_time_pose, max_time_pose);

    if (is_motion_compensation_valid_) {
        MotionCompensationSuTeng80(origin_cloud, compensated_cloud, min_timestamp, max_timestamp, min_time_pose,
                                   max_time_pose);
    } else {
        *compensated_cloud = *origin_cloud;
        LOG(WARNING) << "SuTeng compensation: motion compensation is unvalid.";
    }
}

void Compensator::MotionCompensationSuTeng80(IRADCloudType::Ptr& msg, IRADCloudType::Ptr& out_points,
                                             const double min_timestamp, const double max_timestamp,
                                             const Aff3f& min_time_pose, const Aff3f& max_time_pose) {
    using std::abs;
    using std::acos;
    using std::sin;

    Eigen::Vector3f translation = min_time_pose.translation() - max_time_pose.translation();

    LOG(INFO) <<"Motion translation: "<<translation[0]<<" "<<translation[1]<<" "<<max_timestamp - min_timestamp;

    Eigen::Quaternionf q_max(max_time_pose.linear());
    Eigen::Quaternionf q_min(min_time_pose.linear());
    Eigen::Quaternionf q1(q_max.conjugate() * q_min);
    Eigen::Quaternionf q0(Eigen::Quaternionf::Identity());
    q1.normalize();
    translation = q_max.conjugate() * translation;

    double d = q0.dot(q1);
    double abs_d = abs(d);
    double f = 1.0 / (max_timestamp - min_timestamp);

    const double theta = acos(abs_d);
    const double sin_theta = sin(theta);
    const double c1_sign = (d > 0) ? 1 : -1;

    for (auto& pt : msg->points) {
        if (isPointValidSuTeng<IRADPoint>(pt) && isPointInRangeSuTeng<IRADPoint>(pt)) {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            double t = (max_timestamp - pt.timestamp) * f;
            Eigen::Translation3f ti(t * translation);

            if (abs_d < 1.0 - 1.0e-8) {
                double c0 = sin((1 - t) * theta) / sin_theta;
                double c1 = sin(t * theta) / sin_theta * c1_sign;
                Eigen::Quaternionf qi(c0 * q0.coeffs() + c1 * q1.coeffs());
                Aff3f trans = ti * qi;
                p = trans * p;
            } else {
                p = ti * p;
            }
            IRADPoint temp_point;
            temp_point = pt;
            temp_point.x = p.x();
            temp_point.y = p.y();
            temp_point.z = p.z();
            out_points->push_back(temp_point);
        } 
    }
    out_points->header = msg->header;
    out_points->height = 1;
    out_points->width = out_points->points.size();
}


}  // namespace mapping::tools
