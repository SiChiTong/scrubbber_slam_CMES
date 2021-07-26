// lbh  creat 2019.7.31
#ifndef MAPPING_RESOLUTION_MATCHING_VOXELFILTER_
#define MAPPING_RESOLUTION_MATCHING_VOXELFILTER_

#include <glog/logging.h>
#include <Eigen/Core>
#include <bitset>
#include <cmath>
#include <memory>
#include <unordered_set>
#include <vector>

#include "common/num_type.h"
#include "core/resolution_matching/map/map_cells.h"

namespace mapping::core {

class PointsTool {
   public:
    //点云范围滤波
    static common::PointCloudType::Ptr FilterByMaxRange(common::PointCloudType::Ptr points, float max_range) {
        common::PointCloudType::Ptr results(new common::PointCloudType());
        int size = points->size();
        for (int i = 0; i < size; ++i) {
            auto& point = points->at(i);
            if (hypot(point.x, point.y) <= max_range) {
                results->push_back(point);
            }
        }
        return results;
    }

    //点云xy偏移
    static common::PointCloudType::Ptr OffsetPcd(common::PointCloudType::Ptr points, float offset_x, float offset_y) {
        common::PointCloudType::Ptr results(points);
        int size = results->size();
        for (int i = 0; i < size; ++i) {
            auto& point = results->at(i);
            point.x -= offset_x;
            point.y -= offset_y;
        }
        return results;
    }

    static common::PointCloudType::Ptr RemoveGround(common::PointCloudType::Ptr points, float z_min) {
        common::PointCloudType::Ptr results(new common::PointCloudType());
        int size = points->size();
        results->reserve(size * 0.75);
        for (int i = 0; i < size; ++i) {
            common::PointType point = points->at(i);
            if (point.z > z_min) {
                results->push_back(point);
            }
        }
        return results;
    }

    static float ComputeAngularSearchStep(float resolution, float max_range) {
        return std::acos(1. - pow(resolution, 2) / (2. * pow(max_range, 2)));
    }

    static M4f PoseOnGroud(const M4f& pose) {
        Eigen::Vector3f rpy = pose.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
        Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
        T.rotate(Eigen::AngleAxisf(rpy(0), Eigen::Vector3f(0, 0, 1)));
        T.translation() = Eigen::Vector3f(pose(0, 3), pose(1, 3), 0);
        return T.matrix();
    }
};
// 点云体素滤波，取第一个在栅格内的真实点，不是平均值
class VoxelFilter {
   public:
    explicit VoxelFilter(float size) : resolution_(size) {}

    common::PointCloudType::Ptr Filter(common::PointCloudType::Ptr points) {
        common::PointCloudType::Ptr results(new common::PointCloudType());
        int size = points->size();
        for (int i = 0; i < size; ++i) {
            auto& point = points->at(i);
            auto it_inserted = voxel_set_.insert(IndexToKey(GetCellIndex(point)));
            if (it_inserted.second) {
                results->push_back(point);
            }
        }
        return results;
    }

   protected:
    using KeyType = std::bitset<3 * 32>;
    using uint32 = uint32_t;
    static KeyType IndexToKey(const Eigen::Array3i& index) {
        KeyType k_0(static_cast<uint32>(index[0]));
        KeyType k_1(static_cast<uint32>(index[1]));
        KeyType k_2(static_cast<uint32>(index[2]));
        return (k_0 << 2 * 32) | (k_1 << 1 * 32) | k_2;
    }

    Eigen::Array3i GetCellIndex(const common::PointType& point) const {
        return Eigen::Array3i(point.x / resolution_, point.y / resolution_, point.z / resolution_);
    }

   protected:
    float resolution_;
    std::unordered_set<KeyType> voxel_set_;
};
}  // namespace mapping::core

#endif