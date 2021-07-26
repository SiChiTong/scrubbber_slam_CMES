//
// Created by gaoxiang on 2020/8/11.
//

#ifndef MAPPING_FUNCTION_POINT_H
#define MAPPING_FUNCTION_POINT_H

#include "common/message_def.h"
#include "common/num_type.h"

namespace mapping::common {

static int s_function_point_id = 0;

/// 功能点定义
struct FunctionPoint {
    int type = 0;            // 功能点类型，由协议定义，为整数
    int id = 0;              // 功能点ID，独立
    double timestamp = 0;    // 时间，用于寻找pose
    int number = 1;          // 该类型下的id（从1计算）
    V3d pose = V3d::Zero();  // XYZ
    double heading = 0;      // 航向角
    bool valid = false;      // 是否合法
};

// 从app msg中构建功能点
template <typename T>
inline FunctionPoint CreateFromAppMsg(T msg) {
    return FunctionPoint();
}

/// 两个特化
template <>
inline FunctionPoint CreateFromAppMsg(AppWxbMsgPtr msg) {
    if (msg->specialareatype == 255) {
        return FunctionPoint();
    }

    FunctionPoint function_point;
    function_point.type = msg->specialareatype;
    function_point.id = s_function_point_id;
    function_point.valid = true;
    s_function_point_id++;
    return function_point;
}

template <>
inline FunctionPoint CreateFromAppMsg(AppWbdMsgPtr msg) {
    if (msg->special_point_type == 255) {
        return FunctionPoint();
    }

    FunctionPoint function_point;
    function_point.type = msg->special_point_type;
    function_point.id = s_function_point_id;
    function_point.valid = true;
    s_function_point_id++;
    return function_point;
}

}  // namespace mapping::common

#endif  // MAPPING_FUNCTION_POINT_H
