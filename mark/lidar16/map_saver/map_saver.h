//
// Created by herenjie on 2020/11/2.
//

#ifndef SCRUBBER_SLAM_MAP_SAVER_H
#define SCRUBBER_SLAM_MAP_SAVER_H

//boost smart pointer
#include <boost/make_shared.hpp>
//glog
#include <glog/logging.h>
//pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "common/point_type.h"
#include "common/yaml_io.h"
#include "lidar16/ml_frame.h"
//opencv
#include <opencv2/opencv.hpp>

namespace scrubber_slam { namespace lidar16{

    struct MapParam {
        int map_length;//每个切片地图的长
        int map_matrix_size;
        int load_type;
        double map_origin_x;
        double map_origin_y;
        double leaf_size;
        std::string pcd_path;//用于保存地图的位置
        std::string db_path;

        void init() {
            map_length = 50;
            map_matrix_size = 5;
            load_type = 0;
            map_origin_x = 0;
            map_origin_y = 0;
            leaf_size = 0.2;
            pcd_path = {};
            db_path = {};
        }
    };


    class MapSaver{
    public:
        explicit MapSaver(const MapParam &parm) : param_(parm) {
            global_pcl_points_ptr_ = boost::make_shared<PointCloudType>();
            down_global_pcl_points_ptr_ = boost::make_shared<PointCloudType>();
            if (0 != access(param_.pcd_path.c_str(), 0)) {
                mkdir(param_.pcd_path.c_str(), 0777);
            }
        };
        MapSaver(const YAML_IO & yaml){
            param_.init();
            param_.map_length = yaml.GetValue<int>("map_saver_param","map_length");
            param_.map_matrix_size = yaml.GetValue<int>("map_saver_param","map_matrix_size");
            param_.map_origin_x = yaml.GetValue<double>("map_saver_param","map_origin_x");
            param_.map_origin_y = yaml.GetValue<double>("map_saver_param","map_origin_y");
            param_.leaf_size = yaml.GetValue<double>("map_saver_param","leaf_size");
            param_.pcd_path = yaml.GetValue<std::string>("map_saver_param","pcd_path");

            submap_pcd_path_ = yaml.GetValue<std::string>("map_saver_param","submap_pcd_path");
            if (0 != access(param_.pcd_path.c_str(), 0)) {
                mkdir(param_.pcd_path.c_str(), 0777);
            }
            global_pcl_points_ptr_ = boost::make_shared<PointCloudType>();
            down_global_pcl_points_ptr_ = boost::make_shared<PointCloudType>();
        };
        ~MapSaver();

        void GetSizeOfMap();
        bool SplitPcd();
        void DivCloud();
        void SetPcdPath(std::string path){ param_.pcd_path = path; };
        std::string GetFileName(long utm_x, long utm_y, std::string suffix);
        bool GetGlobalPointcloud(const std::map<Idtype, std::shared_ptr<MLSubmap>>& all_submaps);
        bool SetGlobalPointcloud(const CloudPtr& global_pcl_points);///地图更新要用
    private:
        std::string submap_pcd_path_;
        MapParam param_;
        CloudPtr global_pcl_points_ptr_ = nullptr;  // 点云
        CloudPtr down_global_pcl_points_ptr_ = nullptr;  // 点云

        float min_x_;
        float min_y_;
        float min_z_;
        float max_x_;
        float max_y_;
        float max_z_;
    };

} }


#endif //SCRUBBER_SLAM_MAP_SAVER_H
