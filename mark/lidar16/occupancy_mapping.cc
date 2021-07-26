//
// Created by gaoxiang on 2020/10/28.
//

#include "lidar16/occupancy_mapping.h"
#include "common/math_utils.h"
#include "lidar16/ml_frame.h"
#include "lidar16/ml_submap.h"

#include <map>
#include <opencv2/opencv.hpp>

#include <common/global_config.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>

namespace scrubber_slam { namespace lidar16 {

OccupancyMapping::OccupancyMapping(float resolution, float width, float height)
        : resolution_(resolution), metric_width_(width), metric_height_(height) {
  auto yaml = GlobalConfig::Get();
  min_th_floor_ = yaml->GetValue<double>("map_saver_param", "min_th_floor");
  max_th_floor_ = yaml->GetValue<double>("map_saver_param", "max_th_floor");
  lidar_ext_z_ = yaml->GetValue<float>("lidar", "lidar_ext_z");


  image_width_ = width / resolution;///图像长宽
    image_height_ = height / resolution;
    center_image_ = Vec2f(image_width_ / 2, image_height_ / 2);///图像像素中心

    occupancy_ = cv::Mat(image_height_, image_width_, CV_8U, cv::Scalar(127));
    height_map_ = cv::Mat(image_height_, image_width_, CV_32F, cv::Scalar(0));

    quit_ = false;
    is_saving_map_ = false;
    BuildModel();
}

bool OccupancyMapping::Quit(){
    quit_ = true;
//    occupancy_.release();
}

void OccupancyMapping::PushNewKF(std::shared_ptr<MLFrame> frame) {
    UL lock(frame_mutex_);
    current_frame_ = frame;

    current_black_.clear();
    current_white_.clear();

    if (!DetectPlaneCoeffs()) {
        LOG(INFO) << "failed to detect the main floor plane";
        // or use default params
        return;
    }

    Convert3DTo2DScan();
}

void OccupancyMapping::Convert3DTo2DScan() {
    // 3D转2D算法
    // step 1. 计算每个方向上发出射线上的高度分布, // NOTE 转成整形的360度是有精度损失的
    std::vector<std::map<double, double>> rays(360);             // map键值：距离-相对高度（以距离排序）
    std::vector<Vec2> angle_distance_height(360, Vec2::Zero());  // 每个角度上的距离-高度值
    std::vector<Vec3> pts_3d;///距离地面0.3 ～ 1.2米之间的点云，激光坐标系下

    SE3 Twb = current_frame_->Twl_;
    Quat q = Twb.unit_quaternion();
    Vec3 t = Twb.translation();

    SE3 Tsl = current_frame_->Tsl_;///当前帧的点云可以通过这个变换到submap 的局部坐标系下
    Quat qsl = Tsl.unit_quaternion();
    Vec3 tsl = Tsl.translation();

//    const double min_th = 0.3;
//    const double max_th = 1.2;
  double min_th = min_th_floor_;
  double max_th = max_th_floor_;


    /// 把激光系下的点云转到当前submap坐标系
    for(auto &pt: current_frame_->cloud_ptr_->points){
        if(quit_){
            LOG(WARNING)<<"QUIT ComputeGlobalImg";
            return;
        }
        Vec3 pc = Vec3(pt.x, pt.y, pt.z);
        Vec4 pn = Vec4(pt.x, pt.y, pt.z, 1);
        if(pc.norm() > 80) continue;

        double dis_floor = pn.dot(floor_coeffs_);///该点到地面的距离
        if (dis_floor > min_th && dis_floor < max_th) {
            // 特别矮的和特别高的都不计入
            Vec3 p_sm = Tsl * pc;///点转到当前submap坐标系

            pts_3d.emplace_back(p_sm - tsl);

            Vec2 p(p_sm[0] - tsl[0], p_sm[1] - tsl[1]);
            double dis = p.norm();

            double dangle = atan2(p[1], p[0]) * rad2deg;
            int angle = int(round(dangle) + 360) % 360;
            rays[angle].insert({dis, dis_floor});

            AddPoint(p_sm.head<2>().cast<float>(), true, 0);///距离地面0.3米到1.2米的都是障碍物
        }
    }

    std::vector<double> floor_esti_data;  // 地面高度估计值

    // step 2, 考察每个方向上的分布曲线
    // 正常场景中，每个方向由较低高度开始（地面），转到较高的高度（物体）
    // 如若不是，那么可能发生了遮挡或进入盲区
    constexpr double default_ray_distance = -1;
    constexpr double default_max_ray_distance = 100;
    constexpr double floor_th = 0.6f;  // 地面相关阈值，大于地面这个高度时，直接认定为障碍物

    const double floor_rh = floor_coeffs_[3];

    for (int i = 0; i < 360; ++i) {
        if(quit_){
            LOG(WARNING)<<"QUIT ComputeGlobalImg";
            return;
        }
        if (rays[i].size() < 5) {
            // 该方向测量数据很少，在16线中是不太可能出现的（至少地面上应该有线），出现，则说明有严重遮挡或失效，认为该方向取一个默认的最小距离（车宽）
            angle_distance_height[i] = Vec2(default_ray_distance, floor_rh);
            continue;
        }

        /// 取最近的高度值
        angle_distance_height[i] = Vec2(rays[i].begin()->first, rays[i].begin()->second);
    }

    /// 可视化调试
    // PlotAngleDistance(angle_distance_height, pts_3d);
    Add2DPoints(angle_distance_height, t, true);
}

bool OccupancyMapping::AddPoint(const Vec2f &pt, bool occupy, double relative_height, bool enforce) {
    SetPoint(pt, occupy, relative_height);

    if (occupy) {
        current_black_.emplace_back(pt);
    } else {
        current_white_.emplace_back(pt);
    }

    return true;
}

void OccupancyMapping::SetPoint(const Vec2f &pt, bool occupy, float height) {
    Vec2i px = Submap2Image(pt);///submap的物理坐标转到图像的像素坐标
    int x = px[0], y = px[1];

    if (x < 0 || x >= image_width_ || y < 0 || y >= image_height_) {
        has_outside_points_ = true;
        return;
    }

    /// NOTE 这里增加一个软上下限
    uchar value = occupancy_.ptr<uchar>(y)[x];
    float h = height_map_.ptr<float>(y)[x];

    /// 限制范围，防止越界
    if (occupy) {
        if (value > 117) {
            occupancy_.ptr<uchar>(y)[x] -= submap_logit_increase_;
            // height_map_.ptr<float>(y)[x] = height;
        }
    } else {
//        if (value < 147) {
        if (value < 137) {
            occupancy_.ptr<uchar>(y)[x] += submap_logit_increase_;
        }
    }
}

Vec2i OccupancyMapping::Submap2Image(const Vec2f pt) {
    Vec2f pt_map = pt / resolution_ + center_image_;
    int x = int(pt_map[0] + 0.5f);
    int y = int(pt_map[1] + 0.5f);
    return Vec2i(x, y);
}

void OccupancyMapping::SetCenter(const Vec2f& c) { center_ = c; }

void OccupancyMapping::Add2DPoints(const std::vector<Vec2> &pt2d, const Vec3 &t, bool black) {
    assert(pt2d.size() == 360);

    Vec2 cf_xy_sm = current_frame_->Tsl_.translation().head<2>();///当前关键帧的submap系下坐标地xy坐标

    const double range_max = 80;
    for (auto &pt : model_) {
        if(quit_){
            LOG(WARNING)<<"QUIT ComputeGlobalImg";
            return;
        }
        Vec2f p_sm(pt.dx + cf_xy_sm[0], pt.dy + cf_xy_sm[1]);

        if (pt.range < 0.2) {
            // 小距离内认为无物体
            AddPoint(p_sm, false, 0, true);

            continue;
        }

        // TODO 盲区内既不刷白也不刷黑
        // if (pt.range < 4.10 && pt.range > 0.2) {
        //     continue;
        // }

        double angle = pt.angle * deg2rad;  // 激光系下角度
        KeepAngleIn2PI(angle);

        int angle_index = int(angle * rad2deg) % 360;
        if (angle_index < 0 || angle_index >= pt2d.size()) {
            continue;
        }

        int angle_index_p = angle_index + 1;
        double real_angle = angle;

        // take range and height
        Vec2 range_height;
        if (angle_index_p >= pt2d.size()) {
            range_height = pt2d[angle_index];
        } else {
            // 插值
            double s = (angle * rad2deg) - angle_index;
            Vec2 rh1 = pt2d[angle_index];
            Vec2 rh2 = pt2d[angle_index_p];

            double real_angle1 = angle_index * deg2rad;
            double real_angle2 = angle_index_p * deg2rad;

            if (rh2[0] < 0 || rh2[0] > range_max) {
                range_height = rh1;
                real_angle = real_angle1;
            } else if (rh1[0] < 0 || rh1[0] > range_max) {
                range_height = rh2;
                real_angle = real_angle2;
            } else if (std::fabs(rh1[0] - rh2[0]) > 0.3) {
                range_height = s > 0.5 ? rh2 : rh1;
                real_angle = s > 0.5 ? real_angle2 : real_angle1;
            } else {
                range_height = rh1 * (1 - s) + rh2 * s;
            }
        }

        /// 某方向无测量值时，认为无效
        if (range_height[0] < 0 || range_height[0] > range_max) {
            /// 比较近时，涂白
            if (pt.range < 0.1) {
                AddPoint(p_sm, false, 1);
            }
            continue;
        }

        Vec2f pe(range_height[0] * cos(real_angle), range_height[0] * sin(real_angle));
        Vec2f delta = pe - Vec2f(pt.dx, pt.dy);

        /// 末端点与激光中心之间点，涂白
        if (fabs(delta[0]) < resolution_ && fabs(delta[1]) < resolution_) {
            // AddPoint(pw, true, range_height[1]);
        } else if (range_height[0] > pt.range) {
            AddPoint(p_sm, false, range_height[1]);
        }
    }

    ///车身涂白
    for(float dx = -1.5; dx<0.;dx+=resolution_){
        for(float dy = -0.46;dy<0.46;dy+=resolution_){
            Vec2f p_sm(dx + cf_xy_sm[0], dy + cf_xy_sm[1]);
            AddPoint(p_sm, false, 0, true);
        }
    }
}

cv::Mat OccupancyMapping::GetOccupancyImg(){
//    return occupancy_;

    cv::Mat tmp_occu_img = occupancy_.clone();
    return tmp_occu_img;
}


void OccupancyMapping::PlotAngleDistance(const std::vector<Vec2> &pt2d, const std::vector<Vec3> &pt3d) {
    cv::Mat mat(800, 800, CV_8UC3, cv::Scalar(0, 0, 0));
    float pixel_per_meter = 15;
    for (int i = 0; i < 360; ++i) {
        double dis = pt2d[i][0];
        bool useful = dis > 0.5;
        double height = pt2d[i][1];
        double dx = cos(double(i) * M_PI / 180) * dis;
        double dy = sin(double(i) * M_PI / 180) * dis;

        if (useful) {
            cv::line(mat, cv::Point2f(400, 400), cv::Point2f(400 + dx * pixel_per_meter, 400 + dy * pixel_per_meter),
                     cv::Scalar(0, 0, 128 + height * 10.0), 2);
        } else {
            cv::line(mat, cv::Point2f(400, 400), cv::Point2f(400 + dx * pixel_per_meter, 400 + dy * pixel_per_meter),
                     cv::Scalar(250, 0, 0), 2);
        }
    }

    for (auto &pt : pt3d) {
        cv::circle(mat, cv::Point2f(400 + pt[0] * pixel_per_meter, 400 + pt[1] * pixel_per_meter), 1,
                   cv::Scalar(0, 128 + pt[2] * 10.0, 0), 2);
    }
    cv::imshow("2D Cloud", mat);
    cv::waitKey(1);
}

void OccupancyMapping::BuildModel() {
    // build update model
    for (float x = -model_size_; x < model_size_; x += resolution_) {
        for (float y = -model_size_; y < model_size_; y += resolution_) {
            Model2DPoint<float> pt;
            pt.dx = x;
            pt.dy = y;
            pt.range = sqrt(x * x + y * y);
            pt.angle = std::atan2(y, x) * rad2deg;
            pt.angle = pt.angle > 180 ? pt.angle - 360 : pt.angle;
            model_.push_back(pt);
        }
    }
}

bool OccupancyMapping::DetectPlaneCoeffs() {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.25);

    CloudPtr cloud(new PointCloudType);
    for (auto &pt : current_frame_->cloud_ptr_->points) {
      if (pt.z < 0.3 - lidar_ext_z_) {
        cloud->points.push_back(pt);
      }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (coefficients->values[2] < 0.99) {
        LOG(ERROR) << "floor is not horizontal. ";
        return false;
    }

    if (inliers->indices.size() < 100) {
        LOG(ERROR) << "cannot get enough points on floor: " << inliers->indices.size();
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        floor_coeffs_[i] = coefficients->values[i];
    }
    cloud->clear();

//    LOG(INFO) << "floor coeffs: " << floor_coeffs_.transpose();

    //  save floor
/*    pcl::ExtractIndices<PointType> ext;
    ext.setInputCloud(cloud);
    ext.setIndices(inliers);
    CloudPtr output(new PointCloudType);
    ext.filter(*output);

    pcl::io::savePCDFile("./data/floor.pcd", *output);*/

    return true;
}

Vec2i OccupancyMapping::PosToGrid(const Vec2f &pos) {
    Vec2f grid_f = ((pos + Vec2f(50, 50) - origin_point_.cast<float>()) / 100);
    int gx = floor(grid_f[0]);
    int gy = floor(grid_f[1]);

    return Vec2i(gx, gy);
}

Vec2i OccupancyMapping::PosToGrid(const Vec2 &pos) {
    Vec2f pf = pos.cast<float>();
    return PosToGrid(pf);
}

} }  // namespace scrubber_slam::lidar16
