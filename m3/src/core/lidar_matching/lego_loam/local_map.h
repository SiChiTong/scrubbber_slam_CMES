//
// Created by herenjie on 2021/4/22.
//

#ifndef CARLA_ROS_LOCAL_MAP_H
#define CARLA_ROS_LOCAL_MAP_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>

#include <opencv2/core/core.hpp>

#include "../../../common/num_type.h"
#include "../../../common/mapping_point_types.h"


using namespace mapping::common;
namespace mapping::core {
class LocalMap {
   public:
    LocalMap();
    ~LocalMap() = default;
    SE3 track(const CloudPtr& corner_local_map, const CloudPtr& surf_local_map, const CloudPtr& corner_cur_scam,
              const CloudPtr& surf_cur_scam, SE3& opt_pose_kf_w);

   private:
    void ClearCloud();
    void Scan2MapLM();
    void PreCalCorner();
    struct Point_Line_Jac {
        Point_Line_Jac(Vec6f dd, float res, float s) : deltaRes_deltaEspsilong(dd), residual(res), scale(s) {}
        Vec6f deltaRes_deltaEspsilong;
        float residual;
        float scale;
    };
    std::vector<Point_Line_Jac> plJacs_;
    void PreCalSurf();
    struct Point_Plane_Jac {
        Point_Plane_Jac(Vec6f dd, float res, float s) : deltaRes_deltaEspsilong(dd), residual(res), scale(s) {}
        Vec6f deltaRes_deltaEspsilong;
        float residual;
        float scale;
    };
    std::vector<Point_Plane_Jac> ppJacs_;
    void AccuHession(Eigen::Matrix<float, Eigen::Dynamic, 1>& fr);
    Mat66f H_;
    Eigen::Matrix<float, 6, 1> _JTf;

    CloudPtr cur_corner_ds_;  ///降采样后的当前帧的角点
    CloudPtr cur_surf_ds_;    ///降采样后的当前帧的平面点

    CloudPtr localmap_corner_ds_;  ///降采样后的局部地图的角点
    CloudPtr localmap_surf_ds_;    ///降采样后的局部地图的平面点

    pcl::KdTreeFLANN<PointXYZI>::Ptr kdtree_corner_from_map_;
    pcl::KdTreeFLANN<PointXYZI>::Ptr kdtree_surf_from_map_;

    pcl::VoxelGrid<PointXYZI> corner_voxel_filter_;
    pcl::VoxelGrid<PointXYZI> surf_voxel_filter_;

    ///当前帧的点转到世界坐标系的变换，每次迭代优化求解都会更新，初始值为开始传入的值
    Eigen::Quaterniond q_opt_;
    Eigen::Vector3d t_opt_;
    SE3f POSE_SE3_;
};
}
#endif //CARLA_ROS_LOCAL_MAP_H
