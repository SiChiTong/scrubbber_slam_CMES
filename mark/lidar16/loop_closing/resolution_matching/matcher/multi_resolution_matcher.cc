#include "multi_resolution_matcher.h"
#include "voxel_filter.h"

#include <pcl/common/transforms.h>

namespace mapping { namespace core {

//  MultiResolutionMatcher
float MultiResolutionMatcher::Score(CloudPtr raw_points, float z_ground, SE3& init_pose,
                                    float min_score, SE3& pose_estimate) {
    auto points = TrimRawPoints(raw_points, maps_ptr_->GetResolution(0));
    auto initpose = init_pose;
    pose_estimate = initpose;

    ScoreBoard::BestOffsetPtr offset;
    for (int i = maps_ptr_->GetResolutionDepth() - 1; i >= 0; --i) {
        offset = ScoreOneLayer(points, z_ground, initpose, i, min_score);
        if (offset == nullptr) {
            return -1;
        }

        float resolution = maps_ptr_->GetResolution(i);
        initpose =
            TxyRz(initpose, offset->offset_rad, offset->offset_col * resolution, offset->offset_row * resolution);
    }

    pose_estimate = initpose;
    pcl::transformPointCloud(*points, *points, pose_estimate.cast<float>().matrix());
    return offset->score;
}

ScoreBoard::BestOffsetPtr MultiResolutionMatcher::ScoreOneLayer(CloudPtr points, float z_ground,
                                                                const SE3& init_pose, int layer, float min_score) {
    int linear_search_window = linear_search_window_;
    int angular_search_window = angular_search_window_;
    float step = ComputeAngularSearchStep(maps_ptr_->GetResolution(layer));
    if (layer < maps_ptr_->GetResolutionDepth() - 1) {
        linear_search_window = 1;
        angular_search_window = ComputeAngularSearchStep(maps_ptr_->GetResolution(layer + 1)) * 1.1 / step;
    }

    ScoreBoard board;
    CloudPtr points0(new PointCloudType);
    for (int i = -angular_search_window; i <= angular_search_window; ++i) {
        auto pose = TxyRz(init_pose, i * step, 0, 0);
        pcl::transformPointCloud(*points, *points0, pose.matrix().cast<float>());
        board.AddLogp(maps_ptr_->Logps(points0, z_ground, linear_search_window, layer), i * step);
    }
    auto offset = board.GetScoreOffset();
    offset->score -= log(gauss_min_p);

    if (offset->score < min_score) {
        return nullptr;
    }
    return offset;
}

    CloudPtr MultiResolutionMatcher::TrimRawPoints(CloudPtr raw_points,
                                                                  float resolution) {
    auto points = VoxelFilter(resolution / 1.1).Filter(raw_points);
    points = PointsTool::FilterByMaxRange(points, max_range_);
    return points;
}

SE3 MultiResolutionMatcher::TxyRz(const SE3& pose, float dyaw, float dx, float dy) {
    SE3 T;
    T.setRotationMatrix(Eigen::AngleAxisd(dyaw, V3d::UnitZ()).matrix());
    T.translation() = Eigen::Vector3d(dx, dy, 0);
    return T * pose;
}
} }  // namespace mapping::core
