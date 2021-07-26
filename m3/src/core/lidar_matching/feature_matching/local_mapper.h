//
// Created by wangqi on 19-7-19.
//
#ifndef MAPPING_LOCAL_MAPPER_H
#define MAPPING_LOCAL_MAPPER_H

#include "common/mapping_point_types.h"
#include "common/num_type.h"
#include "core/lidar_matching/feature_matching/matching_param.h"

namespace mapping {
namespace core {

struct LocalMapperImpl;

using PointTypePose = common::PointXYZIRPYT;

class LocalMapper {
   public:
    LocalMapper(FeatureMatchingParams &params);

    ~LocalMapper();

    void SetTrajectoryID(int id);

    void SetCornerPointLast(const common::PointCloudType::Ptr &laser_cloud_corner_last);

    void SetSurfPointLast(const common::PointCloudType::Ptr &laser_cloud_surf_last);

    void SetOutlierCloud(const common::PointCloudType::Ptr &outlier_cloud);

    void SetCurrentTransform(const float transform_sum[6]);

    void SetFirstFixedPose(const SE3 &first_pose);

    void SetKeyFrameID(int id);

    void SetPreviousFrameID(int id);

    void SetStartID(int id);

    void SetSameHeight(bool flag);

    void SetDrPose(const SE3 &dr_pose);

    void Initialization();

    void Run();

    SE3 GetLatestKeyFramePose();

    double GetFitnessScore();

    std::map<int, float> GetDegeneracyEigenValue();

    bool GetLatestDegeneracy();

    std::map<int, SE3> GetMatchingPoses();

    std::map<int, V6d> GetMatchingNoise();

   public:
    void TransformAssociateToMap();

    void TransformUpdate();

    void UpdatePointAssociateToMapSinCos();

    void PointAssociateToMap(common::PointType const *const pi, common::PointType *const po);

    void UpdateTransformPointCloudSinCos(PointTypePose *tIn);

    common::PointCloudType::Ptr TransformPointCloud(common::PointCloudType::Ptr cloudIn);

    common::PointCloudType::Ptr TransformPointCloud(common::PointCloudType::Ptr cloudIn, PointTypePose *transformIn);

    void ExtractSurroundingKeyFrames();

    void DownsampleCurrentScan();

    void CornerOptimization();

    void SurfOptimization();

    bool LMOptimization(int iterCount);

    void Scan2MapOptimization();

    void SaveKeyFrames();

    void ClearCloud();

    bool DetectLoop();

    void PerformLoopClosure();

    void CorrectPose();

    Aff3f pclPointToAffine3fCameraToLidar(PointTypePose thisPoint);

    SE3 pclPointToEigen(PointTypePose thisPoint);

    inline double poseDistance(PointTypePose pose1, PointTypePose pose2) {
        return std::hypot((pose1.x - pose2.x), (pose1.y - pose2.y));
    }

    inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }

    inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

   private:
    void AddVertex(SE3 &input_pose, int &vertex_id, bool fixed_pose = false);

    void AddEdge(int first_id, int second_id, SE3 delta_pose, M6f covariance);

   private:
    std::shared_ptr<LocalMapperImpl> impl_ = nullptr;
};

}  // namespace core
}  // namespace mapping
#endif  // MAPPING_LOCAL_MAPPER_H
