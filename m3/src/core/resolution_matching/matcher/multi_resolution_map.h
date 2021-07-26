// lbh  creat 2019.7.31
#ifndef MAPPING_RESOLUTION_MATCHING_MULTI_RESOLUTION_MAP_
#define MAPPING_RESOLUTION_MATCHING_MULTI_RESOLUTION_MAP_

#include "common/mapping_point_types.h"
#include "common/num_type.h"
#include "core/resolution_matching/map/resolution_map.h"

#include <memory>
#include <utility>

namespace mapping::core {

class MultiResolutionMap {
   public:
    typedef std::shared_ptr<ResolutionMap> ResolMapPtr;
    typedef std::vector<ResolMapPtr> PtrVec;

    struct MapRange {
        double offset_x = 0, offset_y = 0;
        float length = 0, width = 0;
        float low_resolution = 0.5;
        int depth_resolution = 2;
    };

   public:
    MultiResolutionMap(const MapRange& range);

    int AddPoints(common::PointCloudType::Ptr points, float z_ground);

    float Logp(common::PointCloudType::Ptr points, float z_groud, int layer);

    Eigen::MatrixXf Logps(common::PointCloudType::Ptr points, float z_groud, int linear_search, int layer);

    inline Eigen::MatrixXf LogpsLow(common::PointCloudType::Ptr points, float z_ground, int linear_search) {
        return Logps(points, z_ground, linear_search, range_.depth_resolution - 1);
    }

    int GetResolutionDepth() const { return range_.depth_resolution; }

    const PtrVec& GetMapsRef() { return maps_; }

    float GetResolution(int layer) { return maps_[layer]->resolution(); }

    float GetResolutionLow() { return maps_[maps_.size() - 1]->resolution(); }

    const MapRange& GetMapRange() { return range_; }

   protected:
    MapRange range_;
    PtrVec maps_;
};
}  // namespace mapping::core

#endif