//
// Created by gaoxiang on 2020/10/28.
//

#include "lidar16/ml_submap.h"

#include <common/global_config.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

class TransformBroadcaster;
namespace scrubber_slam {
namespace lidar16 {

MLSubmap::MLSubmap(float resolution, float width, float height)
    : resolution_(resolution), metric_width_(width), metric_height_(height) {
    image_width_ = width / resolution;  ///图像长宽
    image_height_ = height / resolution;
    center_image_ = Vec2f(image_width_ / 2, image_height_ / 2);

    static int s_ml_submap_id = 0;
    id_ = s_ml_submap_id++;

    cloud_ = boost::make_shared<PointCloudType>();

    occupancy_mapping_ = std::make_shared<OccupancyMapping>(resolution_, metric_width_, metric_height_);
    auto yaml = GlobalConfig::Get();
    submap_pcd_path_ = yaml->GetValue<std::string>("map_saver_param", "submap_pcd_path");
    fix_height_ = yaml->GetValue<bool>("system", "fix_height");
}

bool MLSubmap::Init() { return true; }
void MLSubmap::ResetLocalPose(SE3 new_local_pose) {
    ///拍平
    if (fix_height_) {
        auto fix_height = [](SE3& pose) {
            ///////四元数转欧拉角 开始
            auto q = pose.unit_quaternion();
            double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
            double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
            double roll, pitch, yaw;
            roll = atan2(sinr_cosp, cosr_cosp);

            double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
            if (fabs(sinp) >= 1) pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
            else
                pitch = asin(sinp);

            double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
            double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
            yaw = atan2(siny_cosp, cosy_cosp);
            ///////四元数转欧拉角 结束

            Mat33 n = (Eigen::AngleAxisd(0., Vec3::UnitX()) * Eigen::AngleAxisd(0., Vec3::UnitY()) *
                       Eigen::AngleAxisd(yaw, Vec3::UnitZ()))
                          .matrix();
            double x0 = pose.translation()[0];
            double y0 = pose.translation()[1];
            pose = SE3(n, Vec3(x0, y0, 1.32));
        };
        fix_height(new_local_pose);
    }

    local_pose_ = new_local_pose;
    SetCenter(new_local_pose.translation().head<2>().cast<float>());
}

bool MLSubmap::InsertKeyFrame(const std::shared_ptr<MLFrame>& frame) {
    ///添加关键帧
    {
        std::unique_lock<std::mutex> kcsm(keyframes_cur_submap_mutex_);
        keyframes_cur_submap_.emplace_back(frame);
    }

    ///将当前关键帧的点云加入到当前submap中的点云中
    CloudPtr cur_cloud_trans(new PointCloudType);  // 当前关键帧的点云转到当前submap局部坐标系中
    pcl::transformPointCloud(*frame->cloud_ptr_, *cur_cloud_trans, frame->Tsl_.matrix());

    {
        std::unique_lock<std::mutex> scm(submap_cloud_mutex_);
        *cloud_ += *cur_cloud_trans;  // 本submap对应的cloud
        cur_cloud_trans->clear();
        cur_cloud_trans = nullptr;
        if (keyframes_cur_submap_.size() > 100) {
            LOG(INFO) << "keyframes_cur_submap_.size(): " << keyframes_cur_submap_.size() << " > 100";
            pcl::VoxelGrid<pcl::PointXYZI> sor;
            sor.setInputCloud(cloud_);
            //        sor.setLeafSize(1.0f, 1.0f, 1.0f);
            sor.setLeafSize(0.2f, 0.2f, 0.2f);

            CloudPtr cloud_save(new PointCloudType);  // 当前关键帧的点云转到当前submap局部坐标系中

            sor.filter(*cloud_save);

            pcl::io::savePCDFileBinaryCompressed(submap_pcd_path_ + "submap_" + std::to_string(id_) + ".pcd",
                                                 *cloud_save);
            cloud_->clear();
            cloud_ = nullptr;
            cloud_save->clear();
            cloud_save = nullptr;
        }
    }

    ///将当前关键帧加入到栅格地图中，用于更新栅格地图
    occupancy_mapping_->PushNewKF(frame);
}

void MLSubmap::PubInfo() {
    ///发布当前submap的局部坐标，以及当前局部坐标下的点云
    if (!init_publisher_) {
        ///可视化点云用
        ros::NodeHandle nh;
        tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>();
        //        submap_points_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("/submap_cloud_points_" +
        //        std::to_string(id_), 1);
        init_publisher_ = true;
    }
    /// 1 发布坐标
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "submap_" + std::to_string(id_);
    SE3 local_pose = GetLocalPose();
    transformStamped.transform.translation.x = local_pose.translation()[0];
    transformStamped.transform.translation.y = local_pose.translation()[1];
    transformStamped.transform.translation.z = local_pose.translation()[2];
    transformStamped.transform.rotation.x = local_pose.unit_quaternion().x();
    transformStamped.transform.rotation.y = local_pose.unit_quaternion().y();
    transformStamped.transform.rotation.z = local_pose.unit_quaternion().z();
    transformStamped.transform.rotation.w = local_pose.unit_quaternion().w();

    tf_br_->sendTransform(transformStamped);
    /// 2 发布点云
    //    sensor_msgs::PointCloud2 current_map_points_ros;
    //    CloudPtr cloud_show(new PointCloudType);
    //    *cloud_show = *GetCloud();
    //    pcl::VoxelGrid<pcl::PointXYZI> sor;
    //    sor.setInputCloud(cloud_show);
    //        sor.setLeafSize(1.0f, 1.0f, 1.0f);
    ////    sor.setLeafSize(0.2f, 0.2f, 0.2f);
    //    sor.filter(*cloud_show);
    //    pcl::toROSMsg(*cloud_show, current_map_points_ros);
    //    current_map_points_ros.header.frame_id = transformStamped.child_frame_id;///坐标系应该是当前submap 坐标系
    //    submap_points_publisher_.publish(current_map_points_ros);
}

bool MLSubmap::OutSide(std::shared_ptr<MLFrame> frame) {
    Vec2 pw = frame->xy();
    Vec2i pt = World2Image(pw.cast<float>());
    int occu = 0;
    float dist = 0;

    cv::Mat occupancy_img = occupancy_mapping_->GetOccupancyImg();
    if (!(pt[0] < 0 || pt[0] >= image_width_ || pt[1] < 0 || pt[1] >= image_height_)) {
        occu = occupancy_img.ptr<uchar>(pt[1])[pt[0]];
    } else {
        return true;
    }

    //    if (occu > 127 && dist > 0.2 && (frame->xy().cast<float>() - kf_center_).norm() < kf_boundary_radious_ + 0.5)
    //    { if (occu > 127 && (frame->xy_l().cast<float>() - kf_center_).norm() < kf_boundary_radious_ + 0.5) {
    if (occu > 127) {
        return false;
    } else {
        LOG(INFO) << "occu OUTSIDE, occu: " << occu;
        return true;
    }
}

Vec2i MLSubmap::World2Image(const Vec2f pt) {
    Vec2f pt_map = (pt - center_) / resolution_ + center_image_;
    int x = int(pt_map[0] + 0.5f);
    int y = int(pt_map[1] + 0.5f);
    return Vec2i(x, y);
}
//@param pt_image :像素坐标
///将当前栅格地图的像素坐标转换到世界物理坐标
Vec2f MLSubmap::Image2WorldWithAngle(const Vec2i& pt_image) {
    Vec2f pt =
        (Vec2f(pt_image[0] + 0.5f, pt_image[1] + 0.5f) - center_image_) * resolution_;  /// submap 的局部物理坐标 2d
    //    Vec3f pt_3d = Vec3f(pt[0], pt[1], 0.);///submap 的局部物理坐标 3d
    Vec4 pt_3d = Vec4((double)pt[0], (double)pt[1], 0., 1.0);  /// submap 的局部物理坐标 3d
    SE3 local_pose = GetLocalPose();
    Vec4 pt_3d_w = local_pose.matrix() * pt_3d;  /// submap 的世界物理坐标 3d
    return pt_3d_w.head<2>().cast<float>();
}

cv::Mat MLSubmap::GetOccupancyBW(int size, bool convert_color) {
    // 小一点，不用那么大
    cv::Mat occupancy_img = occupancy_mapping_->GetOccupancyImg();  // 占据栅格. uchar类型,一个submap 一个栅格地图
    for (int i = 0; i < occupancy_img.rows * occupancy_img.cols; ++i) {
        if (occupancy_img.data[i] > 127) {
            occupancy_img.data[i] = 255;
        } else if (occupancy_img.data[i] < 127) {
            occupancy_img.data[i] = 0;
        }
    }

    if (size != occupancy_img.rows) {
        cv::Mat img_resized;
        cv::resize(occupancy_img, img_resized, cv::Size(size, size));
        occupancy_img = img_resized;
    }

    if (convert_color) {
        cv::Mat img_color;
        cv::cvtColor(occupancy_img, img_color, cv::COLOR_GRAY2BGR);
        return img_color;
    }

    return occupancy_img;
}

}  // namespace lidar16
}  // namespace scrubber_slam