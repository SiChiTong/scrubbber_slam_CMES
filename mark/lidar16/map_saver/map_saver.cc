//
// Created by herenjie on 2020/11/2.
//


#include <lidar16/ml_submap.h>
#include "map_saver.h"

namespace scrubber_slam { namespace lidar16{
    MapSaver::~MapSaver() {
//        std::vector<io::DbPoseAndCloudType>().swap(map_data_);
//        std::vector<int>().swap(frames_read_fail_id_);
//        std::vector<std::string>().swap(files_);
//        std::unordered_map<std::string, common::PointCloudType>().swap(unordered_pcd_);
//
//        files_filter_ = nullptr;
        global_pcl_points_ptr_ = nullptr;
        down_global_pcl_points_ptr_ = nullptr;
    }

    void MapSaver::GetSizeOfMap() {
        min_x_ = (*global_pcl_points_ptr_)[0].x;
        min_y_ = (*global_pcl_points_ptr_)[0].y;
        min_z_ = (*global_pcl_points_ptr_)[0].z;
        max_x_ = (*global_pcl_points_ptr_)[0].x;
        max_y_ = (*global_pcl_points_ptr_)[0].y;
        max_z_ = (*global_pcl_points_ptr_)[0].z;

        for (size_t i = 1; i < (*global_pcl_points_ptr_).size(); ++i) {
            if ((*global_pcl_points_ptr_)[i].x <= min_x_)
                min_x_ = (*global_pcl_points_ptr_)[i].x;
            else if ((*global_pcl_points_ptr_)[i].y <= min_y_)
                min_y_ = (*global_pcl_points_ptr_)[i].y;
            else if ((*global_pcl_points_ptr_)[i].z <= min_z_)
                min_z_ = (*global_pcl_points_ptr_)[i].z;
            else if ((*global_pcl_points_ptr_)[i].x >= max_x_)
                max_x_ = (*global_pcl_points_ptr_)[i].x;
            else if ((*global_pcl_points_ptr_)[i].y >= max_y_)
                max_y_ = (*global_pcl_points_ptr_)[i].y;
            else if ((*global_pcl_points_ptr_)[i].z >= max_z_)
                max_z_ = (*global_pcl_points_ptr_)[i].z;
        }
    }

    void MapSaver::DivCloud() {
        unsigned int map_size_x = (int) (max_x_ - min_x_) / param_.map_length + 1;
        unsigned int map_size_y = (int) (max_y_ - min_y_) / param_.map_length + 1;

        /// 写入说明文件
        cv::FileStorage fs;
        fs.open(param_.pcd_path + "/pointcloud_map.yaml", cv::FileStorage::WRITE);
        fs.write("pcd_path", param_.pcd_path);//保存点云地图的路径
        fs.write("map_length", param_.map_length);//每个点云切片地图的边长
        fs.write("map_size_x", int(map_size_x));//x方向切片地图个数
        fs.write("map_size_y", int(map_size_y));//y方向切片地图个数
        fs.write("map_origin_y", param_.map_origin_y);//地图原点坐标x
        fs.write("map_origin_x", param_.map_origin_x);//地图原点坐标y

        PointCloudType::Ptr temp_pcds[map_size_x * map_size_y];//每一个切片地图
        for (size_t k = 0; k < map_size_x * map_size_y; ++k) {
            temp_pcds[k] = boost::make_shared<PointCloudType>();
        }

        ///根据global_pcl_points_ptr_各个点的坐标，落在哪一个切片中，将其分配到不同的区域
        for (auto point : *global_pcl_points_ptr_) {
            int x = (int) (point.x - min_x_) / param_.map_length;
            int y = (int) (point.y - min_y_) / param_.map_length;
            if( x + y * map_size_x >=  map_size_x * map_size_y || x + y * map_size_x <0 ){
                LOG(ERROR)<<"temp_pcds id error";
                continue;
            }
            temp_pcds[x + y * map_size_x]->push_back(point);
        }
        global_pcl_points_ptr_->clear();
//        global_pcl_points_ptr_= nullptr;

        ///地图中最小的坐标，左下角
        long utmx_min = min_x_ + param_.map_origin_x;
        long utmy_min = min_y_ + param_.map_origin_y;
        //遍历每一个地图切片
        int tmp_map_num = 0;
        for (unsigned int j = 0; j < map_size_x * map_size_y; j++) {
            if (temp_pcds[j]->points.size() > 1) {
                PointCloudType::Ptr temp_pcd = boost::make_shared<PointCloudType>();
                pcl::VoxelGrid<PointType> voxel_grid_filter;
                voxel_grid_filter.setLeafSize(param_.leaf_size, param_.leaf_size, param_.leaf_size);
                voxel_grid_filter.setInputCloud(temp_pcds[j]);
                voxel_grid_filter.filter(*temp_pcd);
                LOG(INFO)<<"temp_pcd SIZE: "<<temp_pcd->points.size();
                pcl::io::savePCDFileBinaryCompressed(param_.pcd_path + "submap_" + std::to_string(tmp_map_num) + ".pcd", *temp_pcd);

                cv::Mat sub_map_center(1, 2, CV_32F);
                sub_map_center.at<float>(0, 0) = utmx_min + param_.map_length * (j % map_size_x);//真实坐标x?
                sub_map_center.at<float>(0, 1) = utmy_min + param_.map_length * (j / map_size_x);
                fs.write("submap_" + std::to_string(tmp_map_num), sub_map_center);//子图及其坐标中心点
                tmp_map_num++;
            }
        }
        fs.write("map_num", tmp_map_num);//点云地图总的个数
    }

    bool MapSaver::GetGlobalPointcloud(const std::map<Idtype, std::shared_ptr<MLSubmap>>& all_submaps){
//        for(auto& sm: all_submaps){
//            auto kfs_in_subamp = sm.second->GetKeyframes();
//            for(auto &kf : kfs_in_subamp){
//                PointCloudType::Ptr cur_kf_cloud_trans = boost::make_shared<PointCloudType>();
//                SE3 T_kf2w = sm.second->GetLocalPose() * kf->Tsl_;
//                pcl::transformPointCloud(*kf->cloud_ptr_, *cur_kf_cloud_trans, T_kf2w.matrix());///当前关键帧点云转到world中
//                *global_pcl_points_ptr_ += *cur_kf_cloud_trans;
//            }
//        }

        CloudPtr cloud_submap(new PointCloudType);
        CloudPtr cloud_submap_w(new PointCloudType);

        global_pcl_points_ptr_->clear();
        for(auto& sm: all_submaps){
            if(sm.second->GetCloud()){
                *cloud_submap = *sm.second->GetCloud();
            }else{
                pcl::io::loadPCDFile(submap_pcd_path_+ "submap_" + std::to_string(sm.second->id_) + ".pcd", *cloud_submap);///submap的局部点云在其局部坐标系下
            }

            SE3 T_kf2w = sm.second->GetLocalPose();

            pcl::transformPointCloud(*cloud_submap, *cloud_submap_w, T_kf2w.matrix());///当前关键帧点云转到world中
            *global_pcl_points_ptr_ += *cloud_submap_w;
        }
        cloud_submap->clear(); cloud_submap = nullptr;
        cloud_submap_w->clear(); cloud_submap_w = nullptr;
        return true;
    }

    bool MapSaver::SetGlobalPointcloud(const CloudPtr& global_pcl_points){
        *global_pcl_points_ptr_ = *global_pcl_points;
        return true;
    }

    std::string MapSaver::GetFileName(long utm_x, long utm_y, std::string suffix) {
        std::stringstream name;
        name << utm_y << "_" << utm_x << "_" << param_.map_length << suffix;
        return name.str();
    }

    bool MapSaver::SplitPcd() {
        GetSizeOfMap();
        DivCloud();
        return true;
    }
} }