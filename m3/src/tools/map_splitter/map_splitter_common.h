//
// Created by pengguoqi on 21-2-24.
//

#ifndef MAP_SPLITTER_COMMON_H
#define MAP_SPLITTER_COMMON_H

#include <vector>
#include <map>

namespace mapping::tools {

using IdTypeKFs = std::vector<IdType>;
using IdTypeKFs_map = std::map<IdType, IdType>;

typedef struct PointAttributes {
    IdType id = ULONG_MAX;
    IdType trj_id = ULONG_MAX;
    IdType map_id = ULONG_MAX;
    SE3 optimized_pose;
    double keyframe_distance = 0.0;
    IdTypeKFs approach_points;
} PointAttributes;

typedef struct MapConnection {
	IdType map_id = ULONG_MAX; //当前子地图
    IdType bef_map_id = ULONG_MAX;   // 前向子图id
    IdType aft_map_id = ULONG_MAX;  // 后向子地图id
    IdType bef_connect_id = ULONG_MAX;   // 前向子图连接id
    IdType aft_connect_id = ULONG_MAX;  // 后向子图连接id
    IdType map_begin_id = ULONG_MAX; // 当前子地图起始id
    IdType map_end_id = ULONG_MAX;  // 连接子地图结束id
} MapConnection;
using MC = MapConnection;
using MCS = std::vector<MapConnection>;
using MCMS = std::map<IdType, std::vector<MapConnection>>;

typedef struct TopoAttributes {
    IdType first_kf_id = ULONG_MAX;   // 当前子地图关键帧id
    IdType second_kf_id = ULONG_MAX;  // 连接子地图关键帧id
    IdType map_first_id = ULONG_MAX; // 当前子地图id
    IdType map_second_id = ULONG_MAX;  // 连接子地图id
} TopoAttributes;
using TOPOS = std::vector<TopoAttributes>;

}  // namespace mapping::tools
#endif  // MAP_SPLITTER_COMMON_H