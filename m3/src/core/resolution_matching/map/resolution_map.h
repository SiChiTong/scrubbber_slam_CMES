// lbh  creat 2019.7.31
#ifndef MAPPING_RESOLUTION_MATCHING_RESOLUTION_MAP_
#define MAPPING_RESOLUTION_MATCHING_RESOLUTION_MAP_

#include "common/num_type.h"
#include "core/resolution_matching/map/map_cells.h"

#include <memory>

namespace mapping::core {

constexpr float gauss_min_p = 1e-5;

class ResolutionMap {
   public:
    ResolutionMap(int rows, int cols, float resolution);

    ~ResolutionMap();

    int AddPoint(const Eigen::Vector4f& point, float z_groud);

    float Logp(const Eigen::Vector4f& point, float z_groud);

    Eigen::MatrixXf Logps(const Eigen::Vector4f& point, float z_groud, int linear_search);

    float resolution() const { return resolution_; }

   protected:
    inline bool CheckMap() { return map_cells_ != nullptr; }

    inline bool CheckRange(int row, int col) { return row >= 0 && row < rows_ && col >= 0 && col < cols_; }

    Descriptor* GetDescriptor(const Eigen::Vector4f& point, float z_ground);

   protected:
    float resolution_;
    int rows_;
    int cols_;
    std::unique_ptr<MapCells> map_cells_;
};
}  // namespace mapping::core

#endif