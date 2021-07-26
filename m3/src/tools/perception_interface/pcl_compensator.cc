//
// Created by elbert huang on 19-8-22.
//

#include "tools/perception_interface/pcl_compensator.h"
#include <glog/logging.h>
#include <map>

namespace mapping {
namespace tools {

using namespace mapping::common;

void PCLPointsCompensator::SetDrPose(const std::vector<TimedPose>& DR_pose_buffer) {
    for (size_t i = 0; i < DR_pose_buffer.size(); i++) {
        TimedPose poseDR = DR_pose_buffer[i];
        pose_buffer_.push(poseDR);
    }
}

void PCLPointsCompensator::Perform(PointCloudXYZRRIARTH::Ptr origin_cloud,
                                   common::PointCloudXYZIHIRBS::Ptr compensated_cloud) {
    SE3 min_time_pose;
    SE3 max_time_pose;

    double min_timestamp = 0.0;
    double max_timestamp = 0.0;
    ComputeTimestampInterval(origin_cloud, min_timestamp, max_timestamp);

    is_motion_compensation_valid_ =
        FindCorrespondPoseFrom(pose_buffer_, min_timestamp, max_timestamp, min_time_pose, max_time_pose);

    if (is_motion_compensation_valid_) {
        MotionCompensation(origin_cloud, compensated_cloud, min_timestamp, max_timestamp, min_time_pose, max_time_pose);
    } else {
        count_test++;
        ConvertPointCloud(origin_cloud, compensated_cloud);
    }
}

void PCLPointsCompensator::ConvertPointCloud(PointCloudXYZRRIARTH::Ptr in_cloud,
                                             common::PointCloudXYZIHIRBS::Ptr out_cloud) {
    for (auto& i : in_cloud->points) {
        if (isPointValid<PointXYZRRIARTH>(i) && isPointInRange<PointXYZRRIARTH>(i)) {
            common::PointXYZIHIRBS temp_point;
            temp_point.x = i.x;
            temp_point.y = i.y;
            temp_point.z = i.z;
            temp_point.intensity = i.intensity;
            temp_point.intensity_vi = i.intensity;
            temp_point.height = i.height;
            temp_point.range = i.range;
            temp_point.ring = i.ring;
            temp_point.type = 0;
            out_cloud->push_back(temp_point);
        }
    }
    out_cloud->header = in_cloud->header;
    out_cloud->height = 1;
    out_cloud->width = out_cloud->points.size();
}

void PCLPointsCompensator::ComputeTimestampInterval(PointCloudXYZRRIARTH::Ptr origin_cloud, double& min_timestamp,
                                                    double& max_timestamp) {
    max_timestamp = 0.0;
    min_timestamp = std::numeric_limits<double>::max();

    for (auto& i : origin_cloud->points) {
        if (i.time < min_timestamp) {
            min_timestamp = i.time;
        }
        if (i.time > max_timestamp) {
            max_timestamp = i.time;
        }
    }
}

bool PCLPointsCompensator::FindCorrespondPoseFrom(CircularBuffer<TimedPose>& pose_buffer, double min_timestamp,
                                                  double max_timestamp, SE3& min_time_pose, SE3& max_time_pose) {
    size_t buffer_size = pose_buffer.size();

    if (buffer_size == 0) {
        // LOG(WARNING) << "No locpos !!!!";
        return false;
    }

    if (max_timestamp - min_timestamp > 0.15 || max_timestamp - min_timestamp < 0.085) {
        LOG(WARNING) << "The scan time between two pointcloud is wrong.";
        LOG(WARNING) << " max_timestamp = " << max_timestamp << ", delta time = " << max_timestamp - min_timestamp;
        return false;
    }

    double tmin_min = 10.0;
    double tmax_min = 10.0;
    std::map<double, int> min_time_diff_idx, max_time_diff_idx;

    for (size_t i = 0; i < buffer_size; i++) {
        double dr_pose_timestamp = pose_buffer[i].time;
        double temp1 = dr_pose_timestamp - min_timestamp;
        if (temp1 < 0) temp1 = -temp1;
        double temp2 = dr_pose_timestamp - max_timestamp;
        if (temp2 < 0) temp2 = -temp2;
        min_time_diff_idx.insert({temp1, i});
        max_time_diff_idx.insert({temp2, i});
        if (temp1 < tmin_min) tmin_min = temp1;
        if (temp2 < tmax_min) tmax_min = temp2;
    }

    int min_time_pose_idx = min_time_diff_idx.find(tmin_min)->second;
    int max_time_pose_idx = max_time_diff_idx.find(tmax_min)->second;

    // LOG(INFO) << "Comp index: "<<min_time_pose_idx<<" "<<max_time_pose_idx;

    if (min_time_pose_idx < int(buffer_size) && min_time_pose_idx >= 0 && max_time_pose_idx < int(buffer_size) &&
        max_time_pose_idx >= 0) {
        min_time_pose = SE3(pose_buffer[min_time_pose_idx].pose);
        max_time_pose = SE3(pose_buffer[max_time_pose_idx].pose);
        return true;
    } else {
        return false;
    }
}

void PCLPointsCompensator::MotionCompensation(PointCloudXYZRRIARTH::Ptr& msg,
                                              common::PointCloudXYZIHIRBS::Ptr out_points, const double min_timestamp,
                                              const double max_timestamp, const SE3& min_time_pose,
                                              const SE3& max_time_pose) {
    using std::abs;
    using std::acos;
    using std::sin;

    V3d translation = min_time_pose.translation() - max_time_pose.translation();

    Quatd q_max = max_time_pose.unit_quaternion();
    Quatd q_min = min_time_pose.unit_quaternion();
    Quatd q1(q_max.conjugate() * q_min);
    Quatd q0(Quatd ::Identity());
    q1.normalize();
    translation = q_max.conjugate() * translation;

    double d = q0.dot(q1);
    double abs_d = abs(d);
    double f = 1.0 / (max_timestamp - min_timestamp);

    const double theta = acos(abs_d);
    const double sin_theta = sin(theta);
    const double c1_sign = (d > 0) ? 1 : -1;

    for (auto& i : msg->points) {
        if (isPointValid<PointXYZRRIARTH>(i) && isPointInRange<PointXYZRRIARTH>(i)) {
            V3d p(i.x, i.y, i.z);
            double t = (max_timestamp - i.time) * f;
            V3d ti = t * translation;

            if (abs_d < 1.0 - 1.0e-8) {
                double c0 = sin((1 - t) * theta) / sin_theta;
                double c1 = sin(t * theta) / sin_theta * c1_sign;
                Quatd qi(c0 * q0.coeffs() + c1 * q1.coeffs());
                SE3 trans(qi, ti);
                p = trans * p;
            } else {
                p = ti + p;
            }
            common::PointXYZIHIRBS temp_point;
            temp_point.x = p.x();
            temp_point.y = p.y();
            temp_point.z = p.z();
            temp_point.intensity = i.intensity;
            temp_point.intensity_vi = i.intensity;
            temp_point.height = i.height;
            temp_point.range = i.range;
            temp_point.ring = i.ring;
            temp_point.type = 0;
            out_points->push_back(temp_point);
        }
    }
    out_points->header = msg->header;
    out_points->height = 1;
    out_points->width = out_points->points.size();
}

void PCLPointsCompensator::Perform32Compensation(PointCloudXYZRRIAR::Ptr origin_cloud,
                                                 PointCloudXYZRRIAR::Ptr compensated_cloud, const double& min_timestamp,
                                                 const double& max_timestamp) {
    SE3 min_time_pose;
    SE3 max_time_pose;
    is_motion_compensation_valid_ =
        FindCorrespondPoseFrom(pose_buffer_, min_timestamp, max_timestamp, min_time_pose, max_time_pose);

    if (is_motion_compensation_valid_) {
        MotionCompensation32(origin_cloud, compensated_cloud, min_timestamp, max_timestamp, min_time_pose,
                             max_time_pose);
    } else {
        *compensated_cloud = *origin_cloud;
        LOG(WARNING) << "[PCLPointsCompensator]: motion compensation is unvalid.";
    }
}

void PCLPointsCompensator::MotionCompensation32(PointCloudXYZRRIAR::Ptr& msg, PointCloudXYZRRIAR::Ptr& out_points,
                                                const double min_timestamp, const double max_timestamp,
                                                const SE3& min_time_pose, const SE3& max_time_pose) {
    using std::abs;
    using std::acos;
    using std::sin;

    V3d translation = min_time_pose.translation() - max_time_pose.translation();

    Quatd q_max = max_time_pose.unit_quaternion();
    Quatd q_min = min_time_pose.unit_quaternion();
    Quatd q1(q_max.conjugate() * q_min);
    Quatd q0(Quatd ::Identity());
    q1.normalize();
    translation = q_max.conjugate() * translation;

    double d = q0.dot(q1);
    double abs_d = abs(d);
    double f = 1.0 / (max_timestamp - min_timestamp);

    const double theta = acos(abs_d);
    const double sin_theta = sin(theta);
    const double c1_sign = (d > 0) ? 1 : -1;

    for (auto& i : msg->points) {
        if (isPointValid<driver::velodyne::PointXYZRRIAR>(i) && isPointInRange<driver::velodyne::PointXYZRRIAR>(i)) {
            V3d p(i.x, i.y, i.z);
            double t = (max_timestamp - i.timestamp) * f;
            V3d ti = t * translation;

            if (abs_d < 1.0 - 1.0e-8) {
                double c0 = sin((1 - t) * theta) / sin_theta;
                double c1 = sin(t * theta) / sin_theta * c1_sign;
                Quatd qi(c0 * q0.coeffs() + c1 * q1.coeffs());
                SE3 trans(qi, ti);
                p = trans * p;
            } else {
                p = ti + p;
            }
            driver::velodyne::PointXYZRRIAR temp_point;
            temp_point.x = p.x();
            temp_point.y = p.y();
            temp_point.z = p.z();
            temp_point.intensity = i.intensity;
            temp_point.range = i.range;
            temp_point.ring = i.ring;
            temp_point.timestamp = i.timestamp;
            temp_point.radius = i.radius;
            temp_point.angle = i.angle;
            out_points->push_back(temp_point);
        }
    }
    out_points->header = msg->header;
    out_points->height = 1;
    out_points->width = out_points->points.size();
    return;
}

}  // namespace tools
}  // namespace mapping
