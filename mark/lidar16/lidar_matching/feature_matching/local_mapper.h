//
// Created by wangqi on 19-7-19.
//
#ifndef FEATURE_MAPPING_LOCAL_MAPPER_H
#define FEATURE_MAPPING_LOCAL_MAPPER_H

#include "common/mapping_point_types.h"
#include "common/num_type.h"
#include "lidar16/lidar_matching/feature_matching/matching_param.h"
#include "lidar16/ml_frame.h"
#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>



namespace mapping {
namespace core {

struct LocalMapperImpl;

using PointTypePose = common::PointXYZIRPYT;

class LocalMapper {
   public:
    LocalMapper(FeatureMatchingParams &params);

    ~LocalMapper();

    void Initialization();

    ///拆分lego-loam
    bool FrameLocalMapper(const CloudPtr& corner_cloud_local_map,
                          const CloudPtr& surf_cloud_local_map,
                          const CloudPtr& ground_cloud_local_map,
                          const CloudPtr& corner_cloud_cur_scan,
                          const CloudPtr& surf_cloud_cur_scan,
                          const CloudPtr& ground_cloud_cur_scan,
                          SE3& opt_pose_kf_w);
    void Scan2MapOptimization_ceres();
    void AddCornerCost(ceres::Problem& ceres_problem);///向ceres solver添加角点参差
    void AddSurfCost(ceres::Problem& ceres_problem);  ///向ceres solver添加平面点参差
    void AddGroundCost(ceres::Problem& ceres_problem);///向ceres solver添加平面点参差

    void ClearCloud();

   private:
    std::shared_ptr<LocalMapperImpl> impl_ = nullptr;
};

}  // namespace core
}  // namespace mapping
#endif  // MAPPING_LOCAL_MAPPER_H
