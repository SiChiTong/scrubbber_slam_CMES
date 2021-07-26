//
// Created by gaoxiang on 2020/10/29.
//

#include "lidar16/ml_tracking.h"

#include <utility>
#include <tf/transform_datatypes.h>
#include "common/global_config.h"
#include "lidar16/ml_frame.h"
#include "lidar16/ml_submap.h"
#include "lidar16/ml_tracking_impl.h"

namespace scrubber_slam {
namespace lidar16 {

MLTracking::MLTracking() : impl_(new MLTrackingImpl) {}
MLTracking::~MLTracking(){
    LOG(INFO)<<"DECONSTRUCT MLTracking =========================: "<<this;
}

bool MLTracking::Init() {
    LOG(ERROR) << "ML TRCKING INIT";
    auto yaml = GlobalConfig::Get();
    float resolution = yaml->GetValue<float>("lidar", "submap_resolution");
    float submap_width = yaml->GetValue<float>("lidar", "submap_width");
    float submap_height = yaml->GetValue<float>("lidar", "submap_height");
    std::string tracking_types_name = yaml->GetValue<std::string>("lidar", "tracking_types");
    if(tracking_types_name == "cartographer"){
        impl_->tracking_types_ = cartographer;
    } else if(tracking_types_name == "lego_loam"){
        impl_->tracking_types_ = lego_loam;
    }

    impl_->g_keyframe_id_ = 0;
    MLFrame::SetIdcountZero();
    impl_->current_submap_ = std::make_shared<MLSubmap>(resolution, submap_width, submap_height);

    impl_->perception_interface_ = std::make_shared<mapping::tools::PerceptionInterface>(*yaml);
    impl_->lego_interface_ = std::make_shared<LegoLoamInterface>();

    impl_->map_saver_ = std::make_shared<MapSaver>(*yaml);                ///仅用于点云地图保存
    impl_->dr_pre_integration_ = std::make_shared<DRPreIntegration>();    ///dr积分用

    impl_->quit_ = false;

    ///回环检测
    impl_->loop_closure_ = std::make_shared<MLLoopClosing>(this);
    impl_->loop_closure_thread_ = std::thread([this]() { this->impl_->loop_closure_->Run(); });
    ///单独一个线程用于将关键帧加入地图，构建点云地图和栅格地图
    impl_->submap_thread_ = std::thread([this]() { this->AddKeyFrameToSubmap(); });

    float lidar_ext_x = yaml->GetValue<float>("lidar", "lidar_ext_x");
    float lidar_ext_y = yaml->GetValue<float>("lidar", "lidar_ext_y");
    float lidar_ext_z = yaml->GetValue<float>("lidar", "lidar_ext_z");
    float lidar_ext_theta_roll = yaml->GetValue<float>("lidar", "lidar_ext_delta_roll");
    float lidar_ext_theta_pitch = yaml->GetValue<float>("lidar", "lidar_ext_delta_pitch");
    float lidar_ext_theta_yaw = yaml->GetValue<float>("lidar", "lidar_ext_delta_yaw");
    impl_->use_self_visualization_ = yaml->GetValue<bool>("lidar", "use_self_visualization");
    impl_->using_front_wheelspeed_ = yaml->GetValue<bool>("system", "using_front_wheelspeed");
    impl_->if_show_localization_map_ = yaml->GetValue<bool>("localization", "if_show_localization_map");///将当前地图发布到ROS topic 上，可视化调试用
    impl_->submap_pcd_path_ = yaml->GetValue<std::string>("map_saver_param", "submap_pcd_path");
    impl_->T_IMU_lidar_.translation() = Vec3(lidar_ext_x, lidar_ext_y, lidar_ext_z);
    //    impl_->T_IMU_lidar_.so3() = SO3::exp(Vec3(lidar_ext_theta_roll, lidar_ext_theta_pitch, lidar_ext_theta_yaw));
    impl_->T_IMU_lidar_.so3() = SO3::exp(Vec3(0, 0, lidar_ext_theta_yaw));
    LOG(INFO) << "T_imu_lidar: \n" << impl_->T_IMU_lidar_.matrix();

    impl_->start_position_ = Vec2::Zero();
    impl_->back_to_start_position_ = false;

    ///点云可视化，调试用
    if(impl_->if_show_localization_map_){
        impl_->result_cloud_ptr_ = boost::make_shared<PointCloudType>();

        ros::NodeHandle nh;
        ///用于调试的定位

        impl_->pub_sharp_points_ = nh.advertise<sensor_msgs::PointCloud2> ("/pub_sharp_points_", 1);
        impl_->pub_less_sharp_points_ = nh.advertise<sensor_msgs::PointCloud2> ("/pub_less_sharp_points_", 1);
        impl_->pub_flat_points_ = nh.advertise<sensor_msgs::PointCloud2> ("/pub_flat_points_", 1);
        impl_->pub_less_flat_points_ = nh.advertise<sensor_msgs::PointCloud2> ("/pub_less_flat_points_", 1);

        impl_->pub_current_map_points_ = nh.advertise<sensor_msgs::PointCloud2> ("/tracking/cur_map_pointcloud", 1);

        impl_->pub_pose_ = nh.advertise<geometry_msgs::PoseStamped> ("/tracking/cur_pose_rviz", 1);
        impl_->pub_pose_DR_ = nh.advertise<geometry_msgs::PoseStamped> ("/tracking/cur_pose_DR_rviz", 1);
        impl_->pub_mapping_thread_ = std::thread([this]() { this->PubMap(); });
    }

    impl_->submaps_by_id_.clear();

    return true;
}

void MLTracking::SetFirstPose(const SE3& pose){
    impl_->origin_pose_ = pose;
}
void MLTracking::GetFirstPose(SE3& pose){
    pose = impl_->origin_pose_;
}

void MLTracking::Quit() {
    impl_->quit_ = true;
    LOG(INFO) << "MLTracking::Quit()";


    impl_->submap_thread_.join();
    LOG(INFO) << "submap_thread_ JOIN";
    impl_->loop_closure_->Quit();
    impl_->loop_closure_thread_.join();

    ///点云可视化，调试用
    if(impl_->if_show_localization_map_){
        impl_->pub_mapping_thread_.join();
        LOG(INFO) << "pub_mapping_thread_ JOIN";
    }

    impl_->all_keyframes_.clear();
    MLTracking::GetKeyframByID().clear();

    impl_->perception_interface_.reset();
    impl_->lego_interface_.reset();
    LOG(INFO)<<"CLEAR ALL";
}
///添加imu数据
void MLTracking::AddNewImu(const UImuMsgPtr& imu){
    impl_->dr_pre_integration_->AddImu(imu);
}
///添加前轮速计数据
void MLTracking::AddNewFrontWheelSpeed(const FrontWheelSpeedMsgScru::ConstPtr &wheelspeed){
    impl_->dr_pre_integration_->AddFrontWheelSpeed(wheelspeed);
}
///添加后轮速计数据
void MLTracking::AddNewRearWheelSpeed(UWheelSpeedMsgPtr wheelspeed){
    impl_->dr_pre_integration_->AddRearWheelSpeed(wheelspeed);
}
///添加激光雷达数据
bool MLTracking::PreprocessCloud(double timestamp, const PacketsMsg::ConstPtr& lidar_msg) {
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    if(!impl_->dr_pre_integration_->DrInited()){
        LOG(WARNING)<<"DR NOT INITED YET";
        return true;
    }
    Eigen::Quaterniond delta_q{1.,0.,0.,0.};
    Eigen::Vector3d delta_p(0,0,0);
    impl_->dr_pre_integration_->GetPreIntegration(delta_q, delta_p);
    SE3 delta_pose = SE3(delta_q, delta_p);
//    SE3 delta_pose = SE3(delta_q, Eigen::Vector3d(0,0,0));

    /// todo 将Velodyne激光数据转换为点云并去畸变 start
    mapping::common::PointCloudPtr cloud_XYZIHIRBS(new mapping::common::PointCloudType);
//    TransPacakge2cloudXYZIHIRBS(SE3(), lidar_msg, timestamp, cloud_XYZIHIRBS);
    TransPacakge2cloudXYZIHIRBS(delta_pose, lidar_msg, timestamp, cloud_XYZIHIRBS);
    /// todo 将Velodyne激光数据转换为点云并去畸变 end

    impl_->current_frame_ = MLFrame::CreateMLFrame(cloud_XYZIHIRBS);
    if(!impl_->func_to_set_.empty()){
        std::string func_name = impl_->func_to_set_.front();
        impl_->current_frame_->is_func_point_ = true;
        impl_->current_frame_->func_name_ = func_name;

        impl_->func_to_set_.pop_front();
    }
    impl_->current_frame_->stamp_ = timestamp;

    auto& cf = impl_->current_frame_;

    if (impl_->status_ == LidarTrackingStatus::INIT) {
        impl_->last_lidar_time_ = timestamp;
//        cf->Twi_ = SE3();  /// IMU世界坐标,初始位置应该是000
        cf->Twi_ = impl_->origin_pose_;  /// IMU世界坐标,初始位置应该是000
        cf->Twl_ = cf->Twi_ * impl_->T_IMU_lidar_;  ///激光雷达的世界坐标，初始位置应该是000+激光雷达的安装偏移

        AddKeyFrame();

        impl_->lego_interface_->FrameFeatureExtract(cf);
        impl_->status_ = LidarTrackingStatus::NORMAL;

        ///关键帧特征点队列，用于局部地图匹配
        AddFeatureDeque(cf);

        impl_->start_position_ = cf->Twi_.translation().head<2>();
    } else if (impl_->status_ == LidarTrackingStatus::NORMAL) {
        Eigen::Quaterniond last_q = impl_->last_frame_->Twi_.unit_quaternion();
        Eigen::Quaterniond cur_q = last_q * delta_q;
        Eigen::Vector3d last_position = impl_->last_frame_->Twi_.translation();
        Eigen::Vector3d cur_position = last_position + last_q * delta_p;
        cf->Twi_ = SE3(cur_q, cur_position);  ///当前帧的IMU坐标系的世界坐标（初始位置的IMU坐标），会通过激光匹配更新
        cf->Twl_ = cf->Twi_ * impl_->T_IMU_lidar_;  ///当前帧激光雷达的世界坐标（初始位置的IMU坐标），初值为DR推出来的位姿加上激光的安装偏置，会通过激光匹配更新

//        LOG(INFO)<<"==============================\n预积分预测激光位姿：\n"<<cf->Twl_.matrix();

        /// TODO 考虑到lego-loam的效率，这边可能要求仅处理关键帧
        impl_->lego_interface_->FrameFeatureExtract(cf);

         impl_->lego_interface_->FrameLocalMapper(impl_->local_map_corner_cloud_deque_,
                                                     impl_->local_map_surf_cloud_deque_,
                                                     impl_->local_map_ground_cloud_deque_,
                                                     cf);


        cf->Twi_ = cf->Twl_ * impl_->T_IMU_lidar_.inverse();

        // TODO 退化检测

        if (IsKeyFrame() || cf->is_func_point_)
        {
            ///关键帧特征点队列，用于局部地图匹配
            AddFeatureDeque(cf);
            AddKeyFrame();
        }
    } else if (impl_->status_ == LidarTrackingStatus::LOST) {
        LOG(ERROR) << "LIDAR TRACKING LOST";
        Reset();
    }

    auto pub_cloud = [](const ros::Publisher& publisher,const CloudPtr& cloud){
      sensor_msgs::PointCloud2 cloud_ros;
      pcl::toROSMsg(*cloud, cloud_ros);
      cloud_ros.header.frame_id = "map";///坐标系应该是当前submap 坐标系
      publisher.publish(cloud_ros);
    };

    pub_cloud(impl_->pub_sharp_points_, cf->corner_points_sharp_);
    pub_cloud(impl_->pub_less_sharp_points_, cf->corner_points_less_sharp_);
    pub_cloud(impl_->pub_flat_points_, cf->surf_points_flat_);
    pub_cloud(impl_->pub_less_flat_points_, cf->surf_points_less_flat_);
    impl_->last_frame_ = impl_->current_frame_;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    LOG(INFO) << "激光tracking用时：" << time_used.count() << " 秒。" << endl;
    return true;
}

void MLTracking::AddFeatureDeque(const std::shared_ptr<MLFrame>& cf){
    ///关键帧特征点队列，用于局部地图匹配
    SE3& T_kf2w = cf->Twl_;
    auto insert_cloud_to_deque = [&T_kf2w](const CloudPtr& cloud_in_kf, std::deque<CloudPtr>& cloud_deque){
      CloudPtr cloud_in_world(new PointCloudType);
      pcl::transformPointCloud(*cloud_in_kf, *cloud_in_world, T_kf2w.matrix());
      cloud_deque.push_back(cloud_in_world);

      if(cloud_deque.size()>30) cloud_deque.pop_front();
    };

    insert_cloud_to_deque(cf->corner_points_less_sharp_, impl_->local_map_corner_cloud_deque_);
    insert_cloud_to_deque(cf->surf_points_less_flat_, impl_->local_map_surf_cloud_deque_);
    insert_cloud_to_deque(cf->ground_cloud_, impl_->local_map_ground_cloud_deque_);
//    cf->corner_points_sharp_->clear();cf->corner_points_sharp_= nullptr;
//    cf->corner_points_less_sharp_->clear();cf->corner_points_less_sharp_= nullptr;
//    cf->surf_points_flat_->clear();cf->surf_points_flat_= nullptr;
//    cf->surf_points_less_flat_->clear();cf->surf_points_less_flat_= nullptr;
}

bool MLTracking::IsKeyFrame() {
    if (impl_->all_keyframes_.size() < 3) {
        return true;
    }

    auto last_keyframe = impl_->all_keyframes_.back();
    Vec2 dp = last_keyframe->xy_l() - impl_->current_frame_->xy_l();

    double dtheta = last_keyframe->theta_l() - impl_->current_frame_->theta_l();
    if (dtheta > 2 * M_PI) dtheta -= 2 * M_PI;
    if (dtheta < -2 * M_PI) dtheta += 2 * M_PI;

    const double position_th = 0.3;
//    const double theta_th = 15 * M_PI / 180.0;
    const double theta_th = 10 * M_PI / 180.0;

    return dp.norm() > position_th || fabs(dtheta) > theta_th;
}

void MLTracking::AddKeyFrame() {
    auto& cf = impl_->current_frame_;

    cf->is_keyframe_ = true;
    cf->keyframe_id_ = impl_->all_keyframes_.size();

    {
        std::unique_lock<std::mutex>  akm(impl_->all_kf_mutex_);
        impl_->all_keyframes_.emplace_back(cf);
    }
    if(cf->is_func_point_)
        impl_->all_func_pts_[cf->func_name_] = cf->keyframe_id_;

}

void MLTracking::AddKeyFrameToSubmap() {
    usleep(1000000);
    int last_kf_size = 0;
    while (!impl_->quit_) {
        if (impl_->all_keyframes_.size() > last_kf_size) {
            std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

            last_kf_size = impl_->all_keyframes_.size();
            auto cf = impl_->all_keyframes_[last_kf_size-1];

            if (impl_->submaps_by_id_.empty() || impl_->current_submap_->keyframes_cur_submap_.size() >100) {
                if(impl_->submaps_by_id_.empty()){
                    impl_->origin_point_ = cf->xy();// 地图原点,全局地图物理原点，第一帧点云的坐标位置
                }

                SE3 kf_pose_in_last_submap = impl_->current_submap_->GetLocalPoseOrigin().inverse() * cf->Twl_;
                SE3 real_pose = impl_->current_submap_->GetLocalPose() * kf_pose_in_last_submap;

                CreateSubmap(real_pose);

                impl_->current_submap_->SetLocalPoseOrigin(cf->Twl_);    ///当前submap的相对于世界坐标系的位姿，当前submap的第一帧位姿，不会变，lego_loam关键帧位姿
                impl_->current_submap_->SetLocalPose(real_pose);         ///当前submap的相对于世界坐标系的位姿，当前submap的第一帧真实位姿,后续在pose graph会更新
            }

            cf->Tsl_ = impl_->current_submap_->GetLocalPoseOrigin().inverse() * cf->Twl_;///当前帧的点云可以通过这个变换到submap 的局部坐标系下
            impl_->current_submap_->InsertKeyFrame(cf);
            cf->kf_in_submap_id_ = impl_->current_submap_->id_;///该关键帧所在的submap 的id

            {
                std::unique_lock<std::mutex> kim(impl_->kf_id_mutex_);
                impl_->keyframes_by_id_.insert({cf->keyframe_id_, cf});
            }
            {
                std::unique_lock<std::mutex> kim(impl_->id_submap_mutex_);
                impl_->submaps_by_id_.insert({impl_->current_submap_->id_, impl_->current_submap_});
            }

            impl_->loop_closure_->DectectLoop(cf);///把当前帧拿去检测回环，和历史中的submap 匹配

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
//            LOG(INFO) << "add kf to submap 用时：" << time_used.count() << " 秒。" << endl;
        }
        usleep(10000);
    }
}

void MLTracking::CreateSubmap(const SE3 & local_pose) {
    auto new_submap = std::make_shared<MLSubmap>(impl_->resolution_, impl_->submap_size_, impl_->submap_size_);
    new_submap->id_ = impl_->submap_id_++;
    new_submap->SetCenter(local_pose.translation().head<2>().cast<float>());///图像的物理中心

    new_submap->Init();

    LOG(INFO) << "create submap " << new_submap->id_ << " with center: " << local_pose.translation().head<2>().transpose();

    impl_->current_submap_ = std::move(new_submap);
}

///用于建图时显示用的图，这个函数会被ROS服务一直调用
cv::Mat MLTracking::GetGlobalMapImage(){
    cv::Mat global_map,global_map_img_traj_show;
    Vec2f top_left, bottom_right;
    if(!ComputeOccupancyMap(global_map, global_map_img_traj_show, top_left, bottom_right)){
//        LOG(ERROR)<<"ComputeOccupancyMap ERROR IN GetGlobalMapImage for show";
        return cv::Mat();
    }
    cv::Mat global_map_img_traj_show_fliped;
    flip(global_map_img_traj_show, global_map_img_traj_show_fliped, 0);

    return global_map_img_traj_show_fliped;
}
///用于拓展地图时获取全局栅格地图
bool MLTracking::GetGlobalOccuMapImage(cv::Mat& global_map_out, Vec2f& top_left, Vec2f& bottom_right){
    cv::Mat global_map_img_traj_show;
    return ComputeOccupancyMap(global_map_out, global_map_img_traj_show, top_left, bottom_right);
}
///用于拓展地图时获取全局点云地图
bool MLTracking::GetGlobalCloudMap(CloudPtr& cloud_old){
    std::map<Idtype, std::shared_ptr<MLSubmap>> all_submaps = GetAllSubMap();
    if(all_submaps.empty()){
        LOG(WARNING)<<"NO MAP TO SAVE, PLEASE STOP THE VEHICLE AND SWITCH TO SLAM MODE AND MAPPING";
        return false;
    }

    CloudPtr cloud_submap(new PointCloudType);
    CloudPtr cloud_submap_w(new PointCloudType);

    cloud_old->clear();
    for(auto& sm: all_submaps){
        if(sm.second->GetCloud()){
            *cloud_submap = *sm.second->GetCloud();
        }else{
            pcl::io::loadPCDFile(impl_->submap_pcd_path_+ "submap_" + std::to_string(sm.second->id_) + ".pcd", *cloud_submap);///submap的局部点云在其局部坐标系下
        }

        SE3 T_kf2w = sm.second->GetLocalPose();

        pcl::transformPointCloud(*cloud_submap, *cloud_submap_w, T_kf2w.matrix());///当前关键帧点云转到world中
        *cloud_old += *cloud_submap_w;
    }
    cloud_submap->clear(); cloud_submap = nullptr;
    cloud_submap_w->clear(); cloud_submap_w = nullptr;
}

bool MLTracking::SaveMap(const std::string& data_path) {
    auto save_pcd_map = [this, &data_path](){
        std::map<Idtype, std::shared_ptr<MLSubmap>> all_submaps = GetAllSubMap();
        if(all_submaps.empty()){
            LOG(WARNING)<<"NO MAP TO SAVE, PLEASE STOP THE VEHICLE AND SWITCH TO SLAM MODE AND MAPPING";
            return false;
        }

        impl_->map_saver_->SetPcdPath(data_path);
        impl_->map_saver_->GetGlobalPointcloud(all_submaps);
        impl_->map_saver_->SplitPcd();
        all_submaps.clear();
    };
    std::thread save_pcd_map_thread = std::thread(save_pcd_map);
    std::thread save_occu_map_thread = std::thread([this, &data_path]() { this->SaveOccupancyMap(data_path); });
    save_pcd_map_thread.join();
    save_occu_map_thread.join();
    return true;
}
///保存栅格地图
bool MLTracking::SaveOccupancyMap(const std::string &path) {
    cv::Mat global_map,global_map_img_traj_show;
    Vec2f top_left, bottom_right;

    if(!ComputeOccupancyMap(global_map, global_map_img_traj_show, top_left, bottom_right)){
        LOG(ERROR)<<"ComputeOccupancyMap ERROR IN SAVING OCCUPANCY MAP";
        return false;
    }

    cv::imwrite(path + "global_map.pgm", global_map_img_traj_show);

    // split map
    // 地图分辨率使用50m x 50m，在0.05分辨率下是1000x1000图像
    // 地图分辨率使用100m x 100m，在0.05分辨率下是2000x2000图像
    int map_index = 0;
    std::map<int, Vec2f> mapindex_center;  // map index to center

    const int tile_image_size = 2000;///一块切分地图图像长宽
    const float tile_image_physic_size = 100.0;///一块切分地图物理长宽
//            const int tile_image_size = 1000;
//            const float tile_image_physic_size = 50.0;
    const float half_tile_image_physic_size = tile_image_physic_size / 2;///一块切分地图物理长宽的一半

    int global_width = global_map.cols;///整个地图宽（列数）
    int global_height = global_map.rows;///整个地图高（行数）

    int x_count = global_width / tile_image_size;///列方向个数
    int x_count_mod = global_width % tile_image_size;
    if (x_count_mod > 0) x_count++;
    int target_global_width = x_count * tile_image_size;///整个的宽

    int y_count = global_height / tile_image_size;
    int y_count_mod = global_height % tile_image_size;
    if (y_count_mod > 0) y_count++;
    int target_global_height = y_count * tile_image_size;///整个的高

    cv::Mat full_global_map(target_global_height, target_global_width, CV_8U, cv::Scalar(127));///整个地图
    cv::Mat full_global_map_ori(full_global_map, cv::Rect(0, 0, global_width, global_height));
    global_map.copyTo(full_global_map_ori);

    for (int i = 0; i < x_count; i++) {
        for (int j = 0; j < y_count; j++) {
            cv::Mat output_image(tile_image_size, tile_image_size, CV_8U, cv::Scalar(127));
            for (int x = 0; x < tile_image_size; x++) {
                for (int y = 0; y < tile_image_size; y++) {
                    int u = i * tile_image_size + x;
                    int v = j * tile_image_size + y;

                    output_image.at<uchar>(y, x) = full_global_map.at<uchar>(v, u);
                }
            }
            cv::imwrite(path + "map_" + std::to_string(map_index) + ".pgm", output_image);

            float cur_center_x = (top_left[0] + half_tile_image_physic_size) + i * tile_image_physic_size;
            float cur_center_y = (top_left[1] + half_tile_image_physic_size) + j * tile_image_physic_size;
            mapindex_center.insert({map_index, Vec2f(cur_center_x, cur_center_y)});
            LOG(INFO) << "submap " << map_index << " center: " << cur_center_x << ", " << cur_center_y<<
                      " , top_left_[1]: "<<top_left[1];

            map_index++;
        }
    }

    LOG(INFO) << "total tiles: " << mapindex_center.size();

    auto yaml = GlobalConfig::Get();
    double prob_th_low = yaml->GetValue<double>("lidar", "submap_prob_th_low");
    double prob_th_high = yaml->GetValue<double>("lidar", "submap_prob_th_high");

    /// 写入说明文件
    cv::FileStorage fs;
    fs.open(path + "/map.yaml", cv::FileStorage::WRITE);
    fs.write("image", "map.pgm");
    fs.write("resolution", 0.05);
    fs.write("submaps", int(mapindex_center.size()));
    fs.write("occupied_thresh", prob_th_low);
    fs.write("free_thresh", prob_th_high);

    for (auto &index_center : mapindex_center) {
        cv::Mat data(1, 2, CV_32F);
        data.at<float>(0, 0) = index_center.second[0];
        data.at<float>(0, 1) = index_center.second[1];
        fs.write("submap_" + std::to_string(index_center.first), data);
    }
    LOG(INFO) << "saving done.";
    return true;
}

bool MLTracking::ComputeOccupancyMap(cv::Mat& global_map_img_save,
                                     cv::Mat& global_map_img_traj_show,
                                     Vec2f& top_left, Vec2f& bottom_right){
    auto submaps_by_id = GetAllSubMap();///所有的submaps
    if(submaps_by_id.empty()){
//        LOG(WARNING)<<"submaps_by_id.empty() WHEN ComputeOccupancyMap";
        return false;
    }

    std::vector<std::shared_ptr<MLSubmap>> submap_vec;

    for (auto &dp : submaps_by_id) {
        if(impl_->quit_){
            LOG(WARNING)<<"QUIT ComputeGlobalImg";
            return false;
        }
        submap_vec.push_back(dp.second);
    }

    top_left = Vec2f(999999, 999999);
    bottom_right = Vec2f(-999999, -999999);

    float resolution = submap_vec[0]->Resolution();
    for (int i = 0; i < submap_vec.size(); i++) {
        if(impl_->quit_){
            LOG(WARNING)<<"QUIT ComputeGlobalImg";
            return false;
        }

        auto m = submap_vec[i];
//        Vec2f c = m->Center();///当前submap的物理中心
        Vec2f c = m->GetLocalPose().translation().head<2>().cast<float>();///当前submap的物理中心的xy坐标
        if (top_left[0] > c[0] - m->MetricWidth() / 2) {
            top_left[0] = c[0] - m->MetricWidth() / 2;
        }
        if (top_left[1] > c[1] - m->MetricHeight() / 2) {
            top_left[1] = c[1] - m->MetricHeight() / 2;
        }

        if (bottom_right[0] < c[0] + m->MetricWidth() / 2) {
            bottom_right[0] = c[0] + m->MetricWidth() / 2;
        }
        if (bottom_right[1] < c[1] + m->MetricHeight() / 2) {
            bottom_right[1] = c[1] + m->MetricHeight() / 2;
        }
    }
    if (top_left[0] > bottom_right[0] || top_left[1] > bottom_right[1]) {
        return false;
    }

    ///全局栅格地图图像物理中心
    Vec2f global_center = Vec2f((top_left[0] + bottom_right[0]) / 2.0, (top_left[1] + bottom_right[1]) / 2.0);
    int c_x = global_center[0] / resolution;
    int c_y = global_center[1] / resolution;
    global_center = Vec2f(c_x * resolution, c_y * resolution);
    ///全局栅格地图图像高宽
    int width = int((bottom_right[0] - top_left[0]) / resolution + 0.5);
    int height = int((bottom_right[1] - top_left[1]) / resolution + 0.5);

    Vec2f center_image = Vec2f(width / 2, height / 2);

    cv::Mat output_image(height, width, CV_8UC3, cv::Scalar(127, 127, 127));
    cv::Mat output_image_save(height, width, CV_8U, cv::Scalar(127));
    cv::Mat global_texture(height, width, CV_8U, cv::Scalar(127));

    for (int i = 0; i < submap_vec.size(); i++) {
        auto m = submap_vec[i];

        cv::Mat occu_img = m->GetOccupancy();
        for (int y = 0; y < m->ImageHeight(); ++y) {
            for (int x = 0; x < m->ImageWidth(); ++x) {
                if(impl_->quit_){
                    LOG(WARNING)<<"QUIT ComputeGlobalImg";
                    return false;
                }

                Vec2f pw = m->Image2WorldWithAngle(Vec2i(x, y));//像素坐标转换到世界物理坐标

                Vec2f pt_map = (pw - global_center) / resolution + center_image;///全局地图像素坐标
                Vec2i pt = Vec2i(int(pt_map[0]), int(pt_map[1]));
                if (pt[0] < 0 || pt[0] >= width || pt[1] < 0 || pt[1] >= height) continue;

                uchar logit = occu_img.at<uchar>(y, x);
                if (logit > 127 && (255 - global_texture.ptr<uchar>(pt[1])[pt[0]] > logit - 127)) {
                    global_texture.ptr<uchar>(pt[1])[pt[0]] += logit - 127;
                } else if (logit < 127 && (global_texture.ptr<uchar>(pt[1])[pt[0]] > 127 - logit)){
                    global_texture.ptr<uchar>(pt[1])[pt[0]] -= 127 - logit;
                }
            }
        }
    }

    int boundary = 5;

    // color map, 以BGR定义
//    可通行区域：R219  G237  B255
//    不可通行区域：R139  G170  B200
//    未知区域：R249  G250  B251
///bgr
    Vec3i color_unexplored(251, 250, 249);//255, 255, 255
    Vec3i color_free(255, 237, 219);//255, 168, 115
    Vec3i color_occupied(200, 170, 139);//105, 41, 35

    for (int y = boundary; y < height - boundary; ++y) {
        for (int x = boundary; x < width - boundary; ++x) {
            if(impl_->quit_){
                LOG(WARNING)<<"QUIT ComputeGlobalImg";
                return false;
            }

            uchar logit = global_texture.ptr<uchar>(y)[x];
            Vec3i c;
            if (logit > 127) {
                c = color_free;
                output_image_save.ptr<uchar>(y)[x] = 255;
            } else if (logit < 127) {
                c = color_occupied;
                output_image_save.ptr<uchar>(y)[x] = 0;
            } else {
                c = color_unexplored;

                uchar lg1 = global_texture.ptr<uchar>(y)[x + 1];
                uchar lg2 = global_texture.ptr<uchar>(y)[x - 1];
                uchar lg3 = global_texture.ptr<uchar>(y + 1)[x];
                uchar lg4 = global_texture.ptr<uchar>(y - 1)[x];
                if (lg1 > 127 && lg2 > 127 && lg3 > 127 && lg4 > 127){
                    output_image_save.ptr<uchar>(y)[x] = 255;
                    c = color_free;
                }
            }

            output_image.ptr<uchar>(y)[x * 3] = c[0];
            output_image.ptr<uchar>(y)[x * 3 + 1] = c[1];
            output_image.ptr<uchar>(y)[x * 3 + 2] = c[2];
        }
    }
    ///用于保存的地图
    global_map_img_save = output_image_save.clone();

    //TODO： 用优化后的submap 的坐标，单个关键帧的位姿没更新
    for(auto &sm: submap_vec){
        std::vector<std::shared_ptr<MLFrame>> kfs = sm->GetKeyframes();
        for(auto &kf: kfs){

            if(impl_->quit_){
                LOG(WARNING)<<"QUIT ComputeGlobalImg";
                return false;
            }

            SE3 submap_local_pose = submaps_by_id[kf->kf_in_submap_id_]->GetLocalPose();
            SE3 kf_pose_w_l = submap_local_pose * kf->Tsl_;
            SE3 kf_pose_w_i = kf_pose_w_l * impl_->T_IMU_lidar_.inverse();
            Vec2 pw(kf_pose_w_i.translation()[0],kf_pose_w_i.translation()[1]);

            Vec2f pw_f = pw.cast<float>();
            Vec2f pt_map = (pw_f - global_center) / resolution + center_image;
            Vec2i p = Vec2i(int(pt_map[0] + 0.5f), int(pt_map[1] + 0.5f));

            if (p[0] < 0 || p[0] >= width || p[1] < 0 || p[1] >= height) continue;
            cv::circle(output_image, cv::Point2f(p[0], p[1]), 2, cv::Scalar(0, 0, 255), 1);
        }
    }

/*    auto last_frame = submap_vec.back()->GetKeyframes().back();
    SE3 lf_submap_local_pose = submaps_by_id[last_frame->kf_in_submap_id_]->GetLocalPose();
    SE3 last_kf_pose_w_l = lf_submap_local_pose * last_frame->Tsl_;
    SE3 last_kf_pose_w_i = last_kf_pose_w_l * impl_->T_IMU_lidar_.inverse();
    Vec2 lf_pw(last_kf_pose_w_i.translation()[0],last_kf_pose_w_i.translation()[1]);

    Vec2f pw_f = lf_pw.cast<float>();
    Vec2f pt_map = (pw_f - global_center) / resolution + center_image;
    auto sm = submap_vec[0];
    int submap_img_height = sm->ImageHeight();
    int submap_img_width = sm->ImageWidth();
    int tl_rec_width = std::max(int(pt_map[0] + 0.5f - submap_img_width/2), 0);
    int tl_rec_height = std::max(int(pt_map[1] + 0.5f - submap_img_height/2), 0);
    int rb_rec_width = std::min(int(width - tl_rec_width), submap_img_width);
    int rb_rec_height = std::min(int(height - tl_rec_height), submap_img_height);
    Vec2i p_rec = Vec2i(tl_rec_width, tl_rec_height);
    //x,y为我们所需范围的左上角坐标;xSize,ySize分别为所需范围宽高
    global_map_img_traj_show = output_image(cv::Rect(tl_rec_width, tl_rec_height, rb_rec_width, rb_rec_height)).clone();*/
    global_map_img_traj_show = output_image.clone();

    submaps_by_id.clear();
    return true;
}

void MLTracking::TransPacakge2cloudXYZIHIRBS(const SE3& pose_from_dr_l, const PacketsMsg::ConstPtr& lidar_msg,double timestamp,
                                             mapping::common::PointCloudPtr cloud_XYZIHIRBS){
    PacketsMsgPtr pm = boost::make_shared<PacketsMsg>();
    *pm = *lidar_msg;

    mapping::common::TimedPose timed_pose_from_dr_l(timestamp, pose_from_dr_l);
    std::vector<mapping::common::TimedPose> timed_pose_from_dr_l_vec{timed_pose_from_dr_l, timed_pose_from_dr_l,
                                                                     timed_pose_from_dr_l};

    impl_->perception_interface_->SetDrPose(timed_pose_from_dr_l_vec);
    std::vector<PacketsMsgPtr> two_packets{pm, pm};
    if (impl_->perception_interface_->GetVelodyneConfig().type == 16) {
        impl_->perception_interface_->PointConvertAndHeightProcess(two_packets, cloud_XYZIHIRBS);
    }
}

///发布点云topic ，用于可视化调试
void MLTracking::PubMap() {
    usleep(1000000);
    while (!impl_->quit_) {
        auto submaps_by_id = GetAllSubMap();///所有的submaps
        if(submaps_by_id.empty()){
//            LOG(WARNING)<<"submaps_by_id.empty() WHEN ComputeOccupancyMap";
            continue;
        }
//        CloudPtr all_cloud_show(new PointCloudType);
#if 1
        CloudPtr cloud_submap(new PointCloudType);
        CloudPtr cloud_submap_w(new PointCloudType);

        CloudPtr global_pcl_points_ptr_(new PointCloudType);  // 点云
        global_pcl_points_ptr_->clear();
        for(auto& sm: submaps_by_id){
            if(sm.second->GetCloud()){
                *cloud_submap = *sm.second->GetCloud();
            }else{
                pcl::io::loadPCDFile(impl_->submap_pcd_path_+ "submap_"
                + std::to_string(sm.second->id_) + ".pcd", *cloud_submap);///submap的局部点云在其局部坐标系下
            }

            SE3 T_kf2w = sm.second->GetLocalPose();

            pcl::transformPointCloud(*cloud_submap, *cloud_submap_w, T_kf2w.matrix());///当前关键帧点云转到world中
            *global_pcl_points_ptr_ += *cloud_submap_w;
        }

        if(global_pcl_points_ptr_->size() <1000 ){
            continue;
        }

        ///发布当前全局地图的所有点云
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud(global_pcl_points_ptr_);
//        sor.setLeafSize(1.0f, 1.0f, 1.0f);
        sor.setLeafSize(0.2f, 0.2f, 0.2f);
        sor.filter(*global_pcl_points_ptr_);

        sensor_msgs::PointCloud2 current_map_points_ros;
        pcl::toROSMsg(*global_pcl_points_ptr_, current_map_points_ros);
        current_map_points_ros.header.frame_id = "map";///坐标系应该是当前submap 坐标系
        impl_->pub_current_map_points_.publish(current_map_points_ros);
#endif
        ///发布当前激光雷达位姿
        static SE3 cur_pose_se3 ;
//        cur_pose_se3 = impl_->all_keyframes_.back()->Twl_;
        std::shared_ptr<MLSubmap> sur_submap = submaps_by_id[submaps_by_id.size()-1];
        SE3 submap_local_pose = sur_submap->GetLocalPose();
        std::shared_ptr<MLFrame> cur_kf = sur_submap->keyframes_cur_submap_.back();
        cur_pose_se3 = submap_local_pose * cur_kf->Tsl_;
        geometry_msgs::PoseStamped cur_pose_ros;
        cur_pose_ros.header.seq = 0;
        cur_pose_ros.header.stamp =ros::Time::now();//如果有问题就使用Time(0)获取时间戳，确保类型一致
        cur_pose_ros.header.frame_id = "map";

        cur_pose_ros.pose.position.x = cur_pose_se3.translation()[0];
        cur_pose_ros.pose.position.y = cur_pose_se3.translation()[1];
        cur_pose_ros.pose.position.z = cur_pose_se3.translation()[2];
        cur_pose_ros.pose.orientation.x = cur_pose_se3.unit_quaternion().x();
        cur_pose_ros.pose.orientation.y = cur_pose_se3.unit_quaternion().y();
        cur_pose_ros.pose.orientation.z = cur_pose_se3.unit_quaternion().z();
        cur_pose_ros.pose.orientation.w = cur_pose_se3.unit_quaternion().w();
        impl_->pub_pose_.publish(cur_pose_ros);

        ///发布当前dr位姿
        SE3 pose_from_dr_i;
//        pose_from_dr_i = cur_pose_se3 * impl_->T_IMU_lidar_.inverse();

        geometry_msgs::PoseStamped cur_pose_ros_dr;
        cur_pose_ros_dr.header.seq = 0;
        cur_pose_ros_dr.header.stamp =ros::Time::now();//如果有问题就使用Time(0)获取时间戳，确保类型一致
        cur_pose_ros_dr.header.frame_id = "map";
        cur_pose_ros_dr.pose.position.x = pose_from_dr_i.translation()[0];
        cur_pose_ros_dr.pose.position.y = pose_from_dr_i.translation()[1];
        cur_pose_ros_dr.pose.position.z = pose_from_dr_i.translation()[2];
        cur_pose_ros_dr.pose.orientation.x = pose_from_dr_i.unit_quaternion().x();
        cur_pose_ros_dr.pose.orientation.y = pose_from_dr_i.unit_quaternion().y();
        cur_pose_ros_dr.pose.orientation.z = pose_from_dr_i.unit_quaternion().z();
        cur_pose_ros_dr.pose.orientation.w = pose_from_dr_i.unit_quaternion().w();
        impl_->pub_pose_DR_.publish(cur_pose_ros_dr);
        usleep(100000);
    }
}

void MLTracking::UpdateSubMapPose(Idtype submap_id, SE3 local_pose) {
    std::unique_lock<std::mutex> kim(impl_->submaps_by_id_[submap_id]->submap_pose_mutex_);
    impl_->submaps_by_id_[submap_id]->ResetLocalPose(local_pose);
}
std::map<Idtype, std::shared_ptr<MLSubmap>> MLTracking::GetAllSubMap() {
    std::unique_lock<std::mutex> kim(impl_->id_submap_mutex_);
    return impl_->submaps_by_id_;
}
std::map<Idtype, std::shared_ptr<MLFrame>> MLTracking::GetKeyframByID() {
    std::unique_lock<std::mutex> kim(impl_->kf_id_mutex_);
    return impl_->keyframes_by_id_;
}

bool MLTracking::GetBackToStartPoint() {
    SE3 cur_pose_imu_in_world;
    bool succ = GetCurPose(cur_pose_imu_in_world);
    if(!succ){
        return false;
    }

//    return (lastkf_Twi.translation().head<2>() - impl_->start_position_).norm() < 1.0 &&
//           impl_->all_keyframes_.size() > 10 &&
//           impl_->move_dist_ > 1.0;
    return (cur_pose_imu_in_world.translation().head<2>() - impl_->start_position_).norm() < 1.0 &&
           impl_->all_keyframes_.size() > 10;
}

bool MLTracking::GetCurPose(SE3 & cur_pose){
    auto submaps_by_id = GetAllSubMap();///所有的submaps
    if(submaps_by_id.empty()){
        return false;
    }
    auto last_submap = submaps_by_id[submaps_by_id.size()-1];
    auto last_kf = last_submap->keyframes_cur_submap_.back();
    SE3 submap_local_pose = last_submap->GetLocalPose();
    SE3 kf_pose_w_l = submap_local_pose * last_kf->Tsl_;
    SE3 lastkf_Twi = kf_pose_w_l * impl_->T_IMU_lidar_.inverse();
    cur_pose = lastkf_Twi;
    return true;
}

///获取所有关键帧用于system获取轨迹
std::vector<std::shared_ptr<MLFrame>> MLTracking::GetHistoryKeyFrames() {
    std::vector<std::shared_ptr<MLFrame>> all_keyframes;////所有的关键帧，这里他拿去获取建图轨迹
    auto submaps_by_id = GetAllSubMap();///所有的submaps
    for(auto &sm: submaps_by_id){
        std::vector<std::shared_ptr<MLFrame>> kfs = sm.second->GetKeyframes();
        for(auto &kf: kfs){
            SE3 submap_local_pose = submaps_by_id[kf->kf_in_submap_id_]->GetLocalPose();
            SE3 kf_pose_w_l = submap_local_pose * kf->Tsl_;
            kf->Twi_ = kf_pose_w_l * impl_->T_IMU_lidar_.inverse();

            all_keyframes.emplace_back(kf);
        }
    }

    return all_keyframes;
}

bool MLTracking::SetFuncPoint(const std::string& func_name){
    if(impl_->all_func_pts_.find(func_name) != impl_->all_func_pts_.end()){
        impl_->all_func_pts_.erase(func_name);
    }
    impl_->func_to_set_.emplace_back(func_name);
    sleep(1);
//    while(impl_->all_func_pts_.find(func_name) == impl_->all_func_pts_.end());
    return true;
}

bool MLTracking::GetAllFuncPts(std::vector<geometry_msgs::PoseStamped> &func_pts_vec){
    auto submaps_by_id = GetAllSubMap();///所有的submaps
    std::map<Idtype, std::shared_ptr<MLFrame>> kfs_by_id = GetKeyframByID();
    for(auto &func_pt: impl_->all_func_pts_){
        geometry_msgs::PoseStamped func_pt_posestamp;

        func_pt_posestamp.header.frame_id = func_pt.first;

        auto kf = kfs_by_id[func_pt.second];
        SE3 submap_local_pose = submaps_by_id[kf->kf_in_submap_id_]->GetLocalPose();
        SE3 kf_pose_w_l = submap_local_pose * kf->Tsl_;
        kf->Twi_ = kf_pose_w_l * impl_->T_IMU_lidar_.inverse();

        SE3 trans_pose = kf->Twi_;

        func_pt_posestamp.pose.position.x = trans_pose.translation()[0];
        func_pt_posestamp.pose.position.y = trans_pose.translation()[1];
        func_pt_posestamp.pose.position.z = 0.0;
        func_pt_posestamp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                static_cast<double>(0.0), static_cast<double>(0.0), static_cast<double>(trans_pose.so3().log()[2]));
        func_pts_vec.push_back(func_pt_posestamp);
    }

    return true;
}

}
}  // namespace lidar16