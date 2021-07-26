//
// Created by pengguoqi on 2021/1/10.
//

#ifndef MAPPING_MERGE_INFO_H
#define MAPPING_MERGE_INFO_H

#include <vector>

namespace mapping {
namespace common {

enum class VertexOptimizationType {
    FIXED = 0,  // 在优化时，此点为固定顶点
    UNFIXED,  	// 在优化时，此点为活动顶点
};

// 定义地图合并关键帧状态信息
struct MergeInfo {
    MergeInfo() {}
    int trajectory_id_ = -1;
    VertexOptimizationType vertex_type_ = VertexOptimizationType::FIXED;
};

using MergeInfoVec = std::vector<MergeInfo>;

}  // namespace common
}  // namespace mapping

#endif  // MAPPING_MERGE_INFO_H
