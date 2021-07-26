//
// Created by pengguoqi on 21-2-24.
//

#ifndef MAP_SPLITTER_BASE_H
#define MAP_SPLITTER_BASE_H

#include <pcl/kdtree/kdtree_flann.h>

#include <deque>
#include <fstream>

#include "common/candidate.h"
#include "common/num_type.h"
#include "io/db_io.h"
#include "io/yaml_io.h"
#include "io/file_io.h"

#include "map_splitter_common.h"

namespace mapping::tools {
/**
 * 子地图切割
 * 用于多层次复杂路段场景
 * 说明：子地图分割功能默认情况下是关闭状态的，需要根据自己的需求选择开启
 * 功能开启标志在yaml中enable_map_splite
 */

class MapSplitterBase {
   public:
    explicit MapSplitterBase() {};

    virtual ~MapSplitterBase() {};

    /// 初始化，成功返回 true
    virtual bool Init() = 0;

    /// 开始处理，完成后返回true，无论结果是否成功
    /// NOTE: 模块内容应阻塞此Start函数，Start函数返回表明已执行完毕
    virtual bool Start() = 0;

   protected:
    io::YAML_IO yaml_file_;

    std::string local_data_path_;
    std::map<IdType, std::vector<common::KFPtr>> keyframes_;
    std::vector<common::LoopCandidate> loops_;

    std::map<IdType, IdTypeKFs_map> result_map_id_;
    std::map<IdType, TOPOS> topos_;

    bool enable_map_splitter_ = false;
    bool save_debug_pcd_ = false;
    bool if_merge_maps_ = false;

    double search_ranger = 50.0;
    double height_threshold = 4.0;
    int map_keyframes_size = 5000;
    int max_iter = 3;

    IdType map_no_ = 0;

    std::string topo_path_;
    std::ofstream topo_file_;
};

}

#endif  // MAP_SPLITTER_BASE_H_