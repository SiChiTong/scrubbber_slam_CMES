// lbh  creat 2019.7.31

#include "core/resolution_matching/matcher/multi_resolution_map.h"

namespace mapping::core {

//  MultiResolutionMap
MultiResolutionMap::MultiResolutionMap(const MapRange& range) : range_(range) {
    for (int i = 0; i < range.depth_resolution; ++i) {
        float resolution = range.low_resolution * (1 << i);
        int rows = range.width / resolution;
        int cols = range.length / resolution;
        maps_.emplace_back(std::make_shared<ResolutionMap>(rows, cols, resolution));
    }
}

int MultiResolutionMap::AddPoints(common::PointCloudType::Ptr points, float z_ground) {
    int size = points->size();
    Eigen::Vector4f point;
    for (int i = 0; i < size; ++i) {
        auto& p = points->at(i);
        point << p.x - range_.offset_x, p.y - range_.offset_y, p.z, p.intensity;
        std::for_each(maps_.begin(), maps_.end(), [&](ResolMapPtr& map) { map->AddPoint(point, z_ground); });
    }
    return 0;
}

float MultiResolutionMap::Logp(common::PointCloudType::Ptr points, float z_ground, int layer) {
    int size = points->size();
    Eigen::Vector4f point;
    float pro = 0.0;
    for (int i = 0; i < size; ++i) {
        auto& p = points->at(i);
        point << p.x - range_.offset_x, p.y - range_.offset_y, p.z, p.intensity;
        pro += maps_[layer]->Logp(point, z_ground);
    }
    return pro / size;
}

Eigen::MatrixXf MultiResolutionMap::Logps(common::PointCloudType::Ptr points, float z_ground, int linear_search,
                                          int layer) {
    int win = 2 * linear_search + 1;
    Eigen::MatrixXf ps(win, win);
    ps.setConstant(0);
    int size = points->size();
    Eigen::Vector4f point;
    for (int i = 0; i < size; ++i) {
        auto& p = points->at(i);
        point << p.x - range_.offset_x, p.y - range_.offset_y, p.z, p.intensity;
        ps += maps_[layer]->Logps(point, z_ground, linear_search);
    }

    ps /= size;
    return ps;
}
}  // namespace mapping::core
