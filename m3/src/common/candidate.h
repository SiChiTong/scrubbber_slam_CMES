//
// Created by gaoxiang on 2020/9/23.
//

#ifndef MAPPING_CANDIDATE_H
#define MAPPING_CANDIDATE_H

#include "common/num_type.h"

namespace mapping::common {

/// 回环候选帧
struct LoopCandidate {
    LoopCandidate() = default;
    LoopCandidate(IdType first, IdType second) : kfid_first(first), kfid_second(second) {}

    IdType kfid_first = 0;   // 第一个
    IdType kfid_second = 0;  // 第二个
    SE3 Tij;                 // 相对运动
    double score = 0;        // 评分

    bool use_mm_match = true;     // 是否使用multi resolution matching来计算初始值
    bool use_init_guess = false;  // 使用Tij作为初始估计，还是使用keyframes里的pose作为初始估计
    int use_pose_stage = 1;       // 使用stage1的pose还是stage2的
};

}  // namespace mapping::common
#endif  // MAPPING_CANDIDATE_H
