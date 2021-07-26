#ifndef MAPPING_RESOLUTION_MATCHING_MULTI_RESOLUTION_MATCHER_
#define MAPPING_RESOLUTION_MATCHING_MULTI_RESOLUTION_MATCHER_

#include <memory>

#include "core/resolution_matching/matcher/multi_resolution_map.h"
#include "core/resolution_matching/matcher/score_board.h"

namespace mapping::core {

// 多分辨率点云范围匹配
class MultiResolutionMatcher {
   public:
    MultiResolutionMatcher(const MultiResolutionMap::MapRange& range)
        : linear_search_window_(1), angular_search_window_(10), max_range_(60) {
        maps_ptr_ = std::make_shared<MultiResolutionMap>(range);
    }

    MultiResolutionMatcher() : linear_search_window_(1), angular_search_window_(10), max_range_(60) {}

    void Init(const MultiResolutionMap::MapRange& range) { maps_ptr_ = std::make_shared<MultiResolutionMap>(range); }

    void SetSearchWindow(int linear, int angular) {
        linear_search_window_ = linear;
        angular_search_window_ = angular;
    }

    void SetMaxRange(const float range) { max_range_ = range; }

    /**
     * 计算评分
     * @param raw_points
     * @param z_ground
     * @param init_pose
     * @param min_score
     * @param pose_estimate
     * @return
     */
    float Score(common::PointCloudType::Ptr raw_points, float z_ground, SE3& init_pose, float min_score,
                SE3& pose_estimate);

    inline int AddPoints(common::PointCloudType::Ptr points, float z_ground) {
        return maps_ptr_->AddPoints(points, z_ground);
    }

   private:
    ScoreBoard::BestOffsetPtr ScoreOneLayer(common::PointCloudType::Ptr points, float z_ground, const SE3& init_pose,
                                            int layer, float min_score);

    common::PointCloudType::Ptr TrimRawPoints(common::PointCloudType::Ptr raw_points, float resolution);

    SE3 TxyRz(const SE3& pose, float dyaw, float dx, float dy);

    inline float ComputeAngularSearchStep(float resolution) {
        return std::acos(1. - pow(resolution, 2) / (2. * pow(max_range_, 2)));
    }

    std::shared_ptr<MultiResolutionMap> maps_ptr_;
    int linear_search_window_;
    int angular_search_window_;
    float max_range_;
};
}  // namespace mapping::core

#endif