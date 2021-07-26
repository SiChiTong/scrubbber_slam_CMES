//
// Created by pengguoqi on 21-2-24.
//

#include "map_splitter.h"
#include <glog/logging.h>

namespace mapping {
namespace tools {

using namespace mapping::common;
using namespace mapping::io;

MapSplitter::MapSplitter(const io::YAML_IO &yaml_file) {
    enable_map_splitter_ = false;
    save_debug_pcd_ = false;
    if_merge_maps_ = false;
    split_kf_ = false;
    yaml_file_ = yaml_file;
    loops_.clear();
    keyframes_.clear();
    open_kfs_.clear();
    close_kfs_.clear();
    result_map_id_.clear();
    mapid_and_pointid_list_.clear();
    topos_.clear();
};

MapSplitter::~MapSplitter() noexcept { 
    std::vector<LoopCandidate>().swap(loops_);
    IdTypeKFs().swap(close_kfs_);
    std::map<IdType, std::vector<common::KFPtr>>().swap(keyframes_);
    std::map<IdType, PointAttributes>().swap(mapid_and_pointid_list_);
    std::map<IdType, IdTypeKFs_map>().swap(result_map_id_);
    std::map<IdType, TOPOS>().swap(topos_);
    LOG(INFO) << "map splitter deconstructed.";
}

bool MapSplitter::Init() {
    enable_map_splitter_ = yaml_file_.GetValue<bool>("map_splitter", "enable_map_splitter");
    size_limit_ = yaml_file_.GetValue<bool>("map_splitter", "enable_map_size_linit");
    search_ranger = yaml_file_.GetValue<double>("map_splitter", "search_ranger");
    height_threshold = yaml_file_.GetValue<double>("map_splitter", "height_threshold");
    map_keyframes_size = yaml_file_.GetValue<int>("map_splitter", "map_keyframes_size");
    max_iter = yaml_file_.GetValue<int>("map_splitter", "maximum_iterations");
    save_debug_pcd_ = yaml_file_.GetValue<bool>("save_debug_pcd");
    if_merge_maps_ = yaml_file_.GetValue<bool>("if_merge_maps");
    local_data_path_ = yaml_file_.GetValue<std::string>("data_fetching", "local_data_path");

    if (!enable_map_splitter_) {
        LOG(INFO) << "子地图分割使能信号未开启";
        return true;
    }

    if (if_merge_maps_) {
        if (!io::LoadKeyframes(local_data_path_ + "merge_keyframes.txt", keyframes_)) {
            LOG(ERROR) << "无法读取全局关键帧文件";
            return false;
        }
    } else {
        if (!io::LoadKeyframes(local_data_path_ + "keyframes.txt", keyframes_)) {
            LOG(ERROR) << "无法读取全局关键帧文件";
            return false;
        }
    }
    if (keyframes_.size() <= 0) {
        LOG(ERROR) << "未加载到任何地图关键帧数据，请查看地图数据是否存在问题";
        return false;
    } 
    if (!io::LoadLoopCandidates(local_data_path_ + "loops.txt", loops_)) {
        LOG(ERROR) << "无法读取闭环位置信息文件";
        return false;
    }
    if (PathExists(local_data_path_ + "splite/")) {
        system(("rm -rf " + local_data_path_ + "splite/").c_str());
    }
    std::string cmd = std::string("mkdir -p ") + local_data_path_ + "splite/";
    system(cmd.c_str());
    topo_path_ = local_data_path_ + "splite/splite_map_topo.txt";
    test_info_path_ = local_data_path_ + "splite/splite_map_info_test.txt";
    return true;
}

bool MapSplitter::Start() {
    LOG(INFO) << "子地图分割开始";
    if (!enable_map_splitter_) {
        LOG(INFO) <<"子地图分割使能信号未开启";
        return true;
    }
    /// 第一步，分割关键帧
    SplitKeyFrames();
    /// 第二步，根据分割好的关键帧对DB进行分割
    SplitMapDB();
    /// 第三步，生成子地图之间的拓扑关系
    GenerateTopology(topos_, true);
    LOG(INFO) << "子地图分割结束";
    return true;
}

void MapSplitter::SplitKeyFrames() {
    IdTypeKFs_map single_map_ids;
    single_map_ids.clear();
    auto add_id_to_deque = [this] (const KFPtr& kf) {
        PointAttributes point;
        point.id = kf->id_;
        point.trj_id = kf->trajectory_id_;
        point.optimized_pose = kf->optimized_pose_stage_2_;
        open_kfs_.push_back(point);
        ref_kfs_.push_back(point);
    };
    auto split = [this] (const PointAttributes& point, IdTypeKFs_map& single_map_ids) {
        auto count = std::count(close_kfs_.begin(), close_kfs_.end(), point.id);
        if (count == 0) {
            single_map_ids.emplace(point.id, point.id);
            close_kfs_.push_back(point.id);
            PointAttributes mapid_and_pointid;
            mapid_and_pointid.id = point.id;
            mapid_and_pointid.map_id = map_no_;
            mapid_and_pointid_list_.insert(std::make_pair(point.id, mapid_and_pointid));
            IdTypeKFs ids;
            ids.clear();
            std::for_each(loops_.begin(), loops_.end(), 
                [&](const LoopCandidate& loopinfo) { 
                    if (loopinfo.kfid_first == point.id) 
                        ids.push_back(loopinfo.kfid_second);
                });
            if (ids.size() >= 1) {
                for (auto &id : ids) {
                     double dis = ComputeDistance(ref_kfs_.at(point.id), ref_kfs_.at(id));
                     if (dis < search_ranger) {
                        auto countid = std::count(close_kfs_.begin(), close_kfs_.end(), id);
                        if (countid == 0) {
                            single_map_ids.emplace(id, id);
                            close_kfs_.push_back(id);
                            PointAttributes s_mapid_and_pointid;
                            s_mapid_and_pointid.id = id;
                            s_mapid_and_pointid.map_id = map_no_;
                            mapid_and_pointid_list_.insert(std::make_pair(id, s_mapid_and_pointid));
                        }
                     }
                }
            }
            // 遍历最近点，来补充闭环的不足
            for (auto& ap : point.approach_points) {
                auto countid = std::count(close_kfs_.begin(), close_kfs_.end(), ap);
                if (countid == 0) {
                    single_map_ids.emplace(ap, ap);
                    close_kfs_.push_back(ap);
                    PointAttributes s_mapid_and_pointid;
                    s_mapid_and_pointid.id = ap;
                    s_mapid_and_pointid.map_id = map_no_;
                    mapid_and_pointid_list_.insert(std::make_pair(ap, s_mapid_and_pointid));
                }
            }
        } else {
            for (auto& ap : point.approach_points) {
                auto countid = std::count(close_kfs_.begin(), close_kfs_.end(), ap);
                if (countid == 0) {
                    single_map_ids.emplace(ap, ap);
                    close_kfs_.push_back(ap);
                    PointAttributes s_mapid_and_pointid;
                    s_mapid_and_pointid.id = ap;
                    s_mapid_and_pointid.map_id = map_no_;
                    mapid_and_pointid_list_.insert(std::make_pair(ap, s_mapid_and_pointid));
                }
            }
        }
    };

    for (auto &trj : keyframes_) {
        LOG(INFO) << "trj : " << trj.first << " , size : " << trj.second.size();
        for (auto &p : trj.second) {
            add_id_to_deque(p);
        }
    }

    size_t kfs_size = open_kfs_.size();
    LOG(INFO) << "open_kf size : " << kfs_size;
    if (kfs_size < map_keyframes_size && kfs_size > 0) {
        LOG(INFO) << "地图关键帧过少，没有必要进行子地图分割";
        return;
    }
    close_kfs_.clear();
    std::map<IdType, IdTypeKFs_map> temp_result_map_id;
    temp_result_map_id.clear();
    while (!open_kfs_.empty()) {
        PointAttributes point = open_kfs_.front();
        open_kfs_.pop_front();
        point.approach_points = ComputeApproachPoints(point);
        if (open_kfs_.size() == 0) {
            point.keyframe_distance = 0.0;    
        } else {
            point.keyframe_distance = ComputeDistance(point, open_kfs_.front());
        }
        split(point, single_map_ids);
        if ((single_map_ids.size() > 0) \
            && (single_map_ids.size() >= map_keyframes_size  \
            || kfs_size == close_kfs_.size()             \
            || point.keyframe_distance >= search_ranger  \
            || (!IsSameLevel(point, single_map_ids)))) {
            split_kf_ = true;
            LOG(INFO) << " split map no : " << map_no_ << ", map size :" << single_map_ids.size(); 
            LOG(INFO) << " split map no : " << map_no_ << ", close_kfs size :" << close_kfs_.size(); 
        }
        if (split_kf_) {
            temp_result_map_id.insert(std::make_pair(map_no_, single_map_ids));
            single_map_ids.clear();
            split_kf_ = false;
            map_no_++;
        }
        if (kfs_size == close_kfs_.size()) {
            while (!open_kfs_.empty()) {
                PointAttributes point = open_kfs_.front();
                open_kfs_.pop_front();
            }
        }
    }
    // OptimizationSubmap1 效果不佳
    // OptimizationSubmap1(temp_result_map_id);
    OptimizationSubmap2(temp_result_map_id);
    UpdateSubMapResultID(temp_result_map_id);
    std::map<IdType, std::map<IdType, SE3>> test_info;
    test_info.clear();
    IdType label = 0;
    for (auto& rmi : temp_result_map_id) {
        std::map<IdType, SE3> single_map_test;
        single_map_test.clear();
        for (auto& smt : rmi.second) {
            single_map_test.emplace(smt.second, ref_kfs_.at(smt.second).optimized_pose);
        }
        result_map_id_.emplace(label, rmi.second);
        test_info.insert(std::make_pair(label, single_map_test));
        LOG(INFO) << " Optimization split map no : " << label << ", map size :" << single_map_test.size(); 
        std::map<IdType, SE3>().swap(single_map_test);
        label++;
    }
    SaveSpliteResults(test_info_path_, test_info);

    topos_.clear();
    for (size_t i = 0; i < result_map_id_.size(); ++i) {
        TOPOS topo;
        topo.clear();
        topos_.emplace(i, topo);
    }
}

void MapSplitter::OptimizationSubmap1(std::map<IdType, IdTypeKFs_map>& result_map_id) {
    std::map<IdType, IdTypeKFs_map> temp;
    temp.clear();
    IdType last_id = ULONG_MAX;
    IdType last_map_no = ULONG_MAX;
    bool change_map_no_begin = false;
    bool change_map_no_end = false;
    IdType change_map_no_begin_id = ULONG_MAX;
    IdType change_map_no_end_id = ULONG_MAX;
    IdType trj_begin_id = ULONG_MAX;
    for (auto &trj : keyframes_) {
        change_map_no_begin = false;
        change_map_no_end = false;
        change_map_no_begin_id = ULONG_MAX;
        change_map_no_end_id = ULONG_MAX;
        size_t size_trj = trj.second.size();
        trj_begin_id = trj.second.at(0)->id_;
        for (auto &p : trj.second) {
            IdType map_no = mapid_and_pointid_list_.at(p->id_).map_id;
            if ((last_map_no != ULONG_MAX) 
                && (last_map_no != map_no)
                && (!change_map_no_begin)) {
                change_map_no_begin = true;
                change_map_no_begin_id = p->id_;
            } else if ((last_map_no != ULONG_MAX) 
                        && (last_map_no != map_no)
                        && change_map_no_begin
                        && (!change_map_no_end)) {
                change_map_no_end = true;
                change_map_no_end_id = p->id_;
            }
            last_map_no = map_no;
            if (change_map_no_end && change_map_no_begin 
                && (change_map_no_end_id - change_map_no_begin_id > size_t(map_keyframes_size / 5))) {
                change_map_no_begin = true;
                change_map_no_end = false;
                trj_begin_id = change_map_no_begin_id;
                change_map_no_begin_id = change_map_no_end_id;
                change_map_no_end_id = ULONG_MAX;
                continue;
            } else if (change_map_no_end && change_map_no_begin && (change_map_no_end_id - change_map_no_begin_id <= size_t(map_keyframes_size / 5))) {
                IdType map_bef = mapid_and_pointid_list_.at(change_map_no_begin_id - 1).map_id;
                IdType map_aft = mapid_and_pointid_list_.at(change_map_no_end_id).map_id;
                if (map_bef == map_aft) {
                    LOG(INFO) << "(between)map_bef == map_aft " << change_map_no_end_id - change_map_no_begin_id;
                    bool same_level = true;
                    IdType map_between = mapid_and_pointid_list_.at(change_map_no_begin_id).map_id;
                    DealWithSubmap(same_level, change_map_no_begin_id, change_map_no_end_id, 
                                   map_bef, map_between, result_map_id);
                    if (same_level) {
                        change_map_no_begin = false;
                        change_map_no_end = false;
                        change_map_no_begin_id = ULONG_MAX;
                        change_map_no_end_id = ULONG_MAX;
                        trj_begin_id = trj_begin_id;
                    } else {
                        change_map_no_end = false;
                        trj_begin_id = change_map_no_end_id;
                        change_map_no_begin_id = change_map_no_end_id;
                        change_map_no_end_id = ULONG_MAX;
                    }
                } else {
                    size_t size_map_bef = result_map_id.at(map_bef).size();
                    size_t size_map_aft = result_map_id.at(map_aft).size();
                    if (size_map_bef >= size_map_aft) {
                        LOG(INFO) << "(between)size_map_bef >= size_map_aft " << change_map_no_end_id - change_map_no_begin_id;
                        bool same_level = true;
                        IdType map_between = mapid_and_pointid_list_.at(change_map_no_begin_id).map_id;
                        DealWithSubmap(same_level, change_map_no_begin_id, change_map_no_end_id, 
                                       map_bef, map_between, result_map_id);
                        if (!same_level) {
                            same_level = true;
                            DealWithSubmap(same_level, change_map_no_begin_id, change_map_no_end_id, 
                                           map_aft, map_between, result_map_id);
                            if (same_level) {
                                trj_begin_id = change_map_no_begin_id;
                                change_map_no_begin = false;
                                change_map_no_end = false;
                                change_map_no_begin_id = ULONG_MAX;
                                change_map_no_end_id = ULONG_MAX;
                            } else {
                                change_map_no_end = false;
                                trj_begin_id = change_map_no_end_id;
                                change_map_no_begin_id = change_map_no_end_id;
                                change_map_no_end_id = ULONG_MAX;
                            }
                        } else {
                            change_map_no_end = false;
                            trj_begin_id = trj_begin_id;
                            change_map_no_begin_id = change_map_no_end_id;
                            change_map_no_end_id = ULONG_MAX;
                        }
                    } else {
                        LOG(INFO) << "(between)size_map_bef < size_map_aft " << change_map_no_end_id - change_map_no_begin_id;
                        bool same_level = true;
                        IdType map_between = mapid_and_pointid_list_.at(change_map_no_begin_id).map_id;
                        DealWithSubmap(same_level, change_map_no_begin_id, change_map_no_end_id, 
                                       map_aft, map_between, result_map_id);
                        if (!same_level) {
                            same_level = true;
                            DealWithSubmap(same_level, change_map_no_begin_id, change_map_no_end_id, 
                                           map_bef, map_between, result_map_id);
                            if (same_level) trj_begin_id = trj_begin_id;
                            else trj_begin_id = change_map_no_end_id;
                            change_map_no_end = false;
                            change_map_no_begin_id = change_map_no_end_id;
                            change_map_no_end_id = ULONG_MAX;
                        } else {
                            trj_begin_id = change_map_no_begin_id;
                            change_map_no_begin = false;
                            change_map_no_end = false;
                            change_map_no_begin_id = ULONG_MAX;
                            change_map_no_end_id = ULONG_MAX;
                        }     
                    }
                }
            } else if (change_map_no_begin && (!change_map_no_end) && (p->id_ == trj.second.at(size_trj - 1)->id_)) {
                IdType map_bef = mapid_and_pointid_list_.at(change_map_no_begin_id - 1).map_id;
                IdType map_aft = mapid_and_pointid_list_.at(trj.second.at(size_trj - 1)->id_).map_id;
                size_t size_map_bef = result_map_id.at(map_bef).size();
                size_t size_map_aft = result_map_id.at(map_aft).size();
                if (size_map_bef >= size_map_aft) {
                    LOG(INFO) << "size_map_bef >= size_map_aft : bef " << map_bef <<  ", aft " << map_aft;
                    bool same_level = true;
                    DealWithSubmap(same_level, change_map_no_end_id, trj.second.at(size_trj - 1)->id_ + 1, 
                                   map_bef, map_aft, result_map_id);
                } else {
                    LOG(INFO) << "size_map_bef < size_map_aft : bef " << map_bef <<  ", aft " << map_aft;
                    bool same_level = true;
                    DealWithSubmap(same_level, trj_begin_id, change_map_no_begin_id, 
                                   map_aft, map_bef, result_map_id);
                }
            } else {
                // 正常遍历过程
            }
        }
    }
}

void MapSplitter::OptimizationSubmap2(std::map<IdType, IdTypeKFs_map>& result_map_id) {
    std::map<IdType, size_t> map_list;
    std::map<IdType, TOPOS> topos;
    static int iter = 0;
    map_list.clear();
    topos.clear();
    UpdateSubMapResultID(result_map_id);
    for (size_t i = 0; i < result_map_id.size(); ++i) {
        TOPOS topo;
        topo.clear();
        topos.emplace(i, topo);
    } 
    // 获取子地图topo关系
    GenerateTopology(topos, false);
    for (auto& map : result_map_id) {
        if (map.second.size() < 1) continue;
        map_list.emplace(map.first, map.second.size());
        LOG(INFO) << "deal map size : " << map.second.size();
    }

    auto find_topo_id = [this] (const IdType& id, const TOPOS& tps) ->IdType {
        for (size_t i = 0; i < tps.size(); ++i) {
            if (id == tps.at(i).first_kf_id) {
                return (IdType)tps.at(i).second_kf_id;
            }
        }
        return ULONG_MAX;
    };

    MCMS mcms;
    mcms.clear();
    for (auto& ml : map_list) {
        bool big_submap = false;
        IdType map_id = ml.first;
        if (size_limit_ && result_map_id.at(map_id).size() >= size_t(map_keyframes_size / 5)) {
            big_submap = true;
        } else {
            big_submap = false;
        }
        TOPOS topo = topos.at(map_id);
        MCS mcs;
        mcs.clear();
        MC mc;
        bool change_mc = false;
        bool change_mc_begin = false;
        bool trj_end_connect = false;
        LOG(INFO) << " map no : " << map_id;
        IdType begin = result_map_id.at(map_id).begin()->first;
        IdType last_id = result_map_id.at(map_id).begin()->first;
        for (auto&mi : result_map_id.at(map_id)) {
            IdType connect_id = ULONG_MAX;
            connect_id = find_topo_id(mi.first, topo);
            if (change_mc) {
                begin = mi.first;
                change_mc = false;
            }
            if (mi.first - last_id >= 2) {
                mc.aft_map_id = ULONG_MAX; 
                mc.map_end_id = last_id;
                mc.aft_connect_id = ULONG_MAX;
                LOG(INFO) << " map no : " << connect_id << " , " << trj_end_connect;
                if (connect_id == ULONG_MAX || (!trj_end_connect)) {
                    if (big_submap) {
                        IdType delta = mc.map_end_id - mc.map_begin_id; 
                        if (delta <= map_keyframes_size / 40) {
                            mcs.push_back(mc);
                        }
                    } else {
                        mcs.push_back(mc);
                    }
                    begin = mi.first;
                    change_mc_begin = false;
                    change_mc = false;
                    trj_end_connect = false;
                }
            } 
            if (connect_id != ULONG_MAX) {
                mc.map_id = map_id;
                LOG(INFO) << mi.first <<  " " << connect_id << " " << last_id  << " "  << begin;
                if (begin == mi.first) {
                    mc.bef_map_id = mapid_and_pointid_list_.at(connect_id).map_id;  
                    mc.map_begin_id = begin;
                    mc.bef_connect_id = connect_id;
                    change_mc_begin = true;
                } else {
                    if (change_mc_begin) {
                        mc.aft_map_id = mapid_and_pointid_list_.at(connect_id).map_id; 
                        mc.map_end_id = mi.first;
                        mc.aft_connect_id = connect_id;
                        change_mc_begin = false;
                    } else {
                        mc.bef_map_id = ULONG_MAX;
                        mc.aft_map_id = mapid_and_pointid_list_.at(connect_id).map_id; 
                        mc.map_begin_id = begin;
                        mc.bef_connect_id = ULONG_MAX;
                        mc.aft_connect_id = connect_id;
                        mc.map_end_id = mi.first;
                    }
                    change_mc = true;
                    if (big_submap) {
                        IdType delta = mc.map_end_id - mc.map_begin_id; 
                        if (delta <= map_keyframes_size / 80) {
                            mcs.push_back(mc);
                        }
                    } else {
                        mcs.push_back(mc);
                    }
                    trj_end_connect = true;
                } 
            } else {
                if (begin == mi.first) {
                    mc.bef_map_id = ULONG_MAX;
                    mc.map_begin_id = begin;
                    mc.bef_connect_id = ULONG_MAX;
                }
            }
            last_id = mi.first;
        }
        if (change_mc_begin) {
            mc.map_id = map_id;
            mc.aft_map_id = ULONG_MAX; 
            mc.map_end_id = result_map_id.at(map_id).rbegin()->first;
            mc.aft_connect_id = ULONG_MAX;
            change_mc_begin = false;
            if (big_submap) {
                IdType delta = mc.map_end_id - mc.map_begin_id; 
                if (delta <= map_keyframes_size / 40) {
                    mcs.push_back(mc);
                }
            } else {
                mcs.push_back(mc);
            }
        }
        ShowMCSINFO(mcs);
        mcms.emplace(map_id, mcs);
    }

    for (auto& msm : mcms) {
        for (size_t i = 0; i < msm.second.size(); ++i) {
            MC mc = msm.second.at(i);
            IdType delta = mc.map_end_id - mc.map_begin_id;
            if (mc.bef_connect_id != ULONG_MAX) {
                mc.bef_map_id = mapid_and_pointid_list_.at(mc.bef_connect_id).map_id;
            } 
            if (mc.aft_connect_id != ULONG_MAX) {
                mc.aft_map_id = mapid_and_pointid_list_.at(mc.aft_connect_id).map_id;
            }
            if (mc.bef_map_id == ULONG_MAX && mc.aft_map_id == ULONG_MAX) {
                continue;
            } else if (mc.bef_map_id != ULONG_MAX && mc.aft_map_id == ULONG_MAX) {
                LOG(INFO) << "delta : " << delta << " , map id :" << mc.map_id;
                if (mc.bef_map_id == mc.map_id) continue;
                if ((!size_limit_) || delta < map_keyframes_size / 10) {
                    bool same_level = true;
                    DealWithSubmap(same_level, mc.map_begin_id, mc.map_end_id + 1, 
                                    mc.bef_map_id, mc.map_id, result_map_id, 0.1);
                }
                continue;
            } else if (mc.bef_map_id == ULONG_MAX && mc.aft_map_id != ULONG_MAX) {
                LOG(INFO) << "delta : " << delta << " , map id :" << mc.map_id;
                if (mc.aft_map_id == mc.map_id) continue;
                if ((!size_limit_) || delta < map_keyframes_size / 10) {
                    bool same_level = true;
                    DealWithSubmap(same_level, mc.map_begin_id, mc.map_end_id + 1, 
                                    mc.aft_map_id, mc.map_id, result_map_id, 0.1);
                }
                continue;
            }
            if (mc.bef_map_id != mc.aft_map_id) {
                LOG(INFO) << "delta : " << delta << " , map id :" << mc.map_id;
                if ((!size_limit_) || delta < map_keyframes_size / 10) {
                    if (mc.bef_map_id == mc.map_id) {
                        continue;
                    }
                    bool same_level = true;
                    DealWithSubmap(same_level, mc.map_begin_id, mc.map_end_id + 1, 
                               mc.bef_map_id, mc.map_id, result_map_id, 0.1);
                    if (!same_level) {
                        same_level = true;
                        DealWithSubmap(same_level, mc.map_begin_id, mc.map_end_id + 1, 
                                       mc.aft_map_id, mc.map_id, result_map_id, 0.1);
                    }
                }
            } else if (mc.bef_map_id == mc.aft_map_id) {
                LOG(INFO) << "delta : " << delta << " , map id :" << mc.map_id;
                if ((!size_limit_) || delta < map_keyframes_size / 10) {
                    if (mc.aft_map_id == mc.map_id || mc.bef_map_id == mc.map_id) {
                        continue;
                    }
                    bool same_level = true;
                    DealWithSubmap(same_level, mc.map_begin_id, mc.map_end_id + 1, 
                                   mc.bef_map_id, mc.map_id, result_map_id, 0.1);
                }
            }
        }
    }

    if (iter >= max_iter - 5) {
        CheckSubMap(result_map_id);
    }

    iter++;
    if (iter >= max_iter) return;
    else {
        OptimizationSubmap2(result_map_id);
    }
}

void MapSplitter::DealWithSubmap(bool& same_level, const IdType& begin, const IdType& end, 
                                 const IdType& push_map_no, const IdType& pop_map_no,
                                 std::map<IdType, IdTypeKFs_map>& result_map_id,
                                 double resolution) {
    for (int id = begin; id < end; ++id) {
        same_level = IsSameLevel(ref_kfs_.at(id), result_map_id.at(push_map_no), resolution);
        if (!same_level) {
            LOG(INFO) << "same_level";
            break;
        }
    }
    LOG(INFO) << "same_level " << same_level;
    if (same_level) {
        UpdateSubMap(begin, end, push_map_no, pop_map_no, result_map_id);
        LOG(INFO) << "add map no : " << push_map_no << ", size : " << result_map_id.at(push_map_no).size();
        LOG(INFO) << "delete map no : " << pop_map_no << ", size : " << result_map_id.at(pop_map_no).size();
    }
}

void MapSplitter::UpdateSubMap(const IdType& begin, const IdType& end, 
                               const IdType& push_map_no, const IdType& pop_map_no,
                               std::map<IdType, IdTypeKFs_map>& result_map_id) {
    LOG(INFO) << "begin : " << begin << ", end : " << end;
    for (int id = begin; id < end; ++id) {
        result_map_id.at(push_map_no).emplace(id, id);
        result_map_id.at(pop_map_no).erase(id);
        mapid_and_pointid_list_.at(id).map_id = push_map_no;
    }
}

void MapSplitter::CheckSubMap(std::map<IdType, IdTypeKFs_map>& result_map_id) {
    std::map<IdType, std::map<IdType, std::vector<IdType>>> maps;
    maps.clear();
    for (auto&tr : result_map_id) {
        std::map<IdType, std::vector<IdType>> map;
        map.clear();
        bool first = true;
        IdType map_no = 0;
        size_t size = tr.second.size();
        LOG(INFO) << "check " << size;
        if (size < 1) continue;
        for (auto id : tr.second) {
            if (first) {
                std::vector<IdType> ps;
                ps.clear();
                ps.push_back(id.first);
                map.emplace(map_no, ps);
                first = false;
                continue;
            }
            bool is_same_map = false;
            IdType same_map_id = tr.first;
            for (auto& ps : map) {
                for (auto& p : ps.second) {
                    double dis = ComputeDistance(ref_kfs_.at(id.first), ref_kfs_.at(p), false);
                    if (dis < search_ranger) {
                        is_same_map = true;
                        same_map_id = ps.first;
                    }
                    if (is_same_map) break;
                }
                if (is_same_map) break;
            }
            
            if (is_same_map) {
                map.at(same_map_id).push_back(id.first);
            } else {
                std::vector<IdType> ps;
                ps.clear();
                LOG(INFO) << "check map : " << map.at(map_no).size();
                map_no++;
                ps.push_back(id.first);
                map.emplace(map_no, ps);
                LOG(INFO) << "   " << id.first;
            }
        }
        maps.emplace(tr.first, map);
        LOG(INFO) << tr.first << " , " << tr.first;
        std::map<IdType, std::vector<IdType>>().swap(map);
    }
    LOG(INFO) << "before update map size : " << result_map_id.size();
    result_map_id.clear();
    IdType result_id = 0;
    for (auto& map : maps) {
        if (map.second.size() < 1) continue;
        for (auto& ids : map.second) {
            IdTypeKFs_map kfs;
            kfs.clear();
            for (auto& id : ids.second) {
                kfs.emplace(id, id);
                mapid_and_pointid_list_.at(id).map_id = result_id;
            }
            result_map_id.emplace(result_id, kfs);
            result_id++;
            IdTypeKFs_map().swap(kfs);
        }
    }
    LOG(INFO) << "after update map size : " << result_map_id.size();
    std::map<IdType, std::map<IdType, std::vector<IdType>>>().swap(maps);
}

void MapSplitter::UpdateSubMapResultID(std::map<IdType, IdTypeKFs_map>& result_map_id) {
    std::map<IdType, IdTypeKFs_map> temp_result_map_id;
    for (auto& map_id : result_map_id) {
        temp_result_map_id.emplace(map_id.first, map_id.second);
    }
    LOG(INFO) << "before update result_map_id size : " << result_map_id.size();
    result_map_id.clear();
    IdType map_no = 0;
    for (auto& map_id : temp_result_map_id) {
        if (map_id.second.size() < 1) continue;
        result_map_id.emplace(map_no, map_id.second);
        for (auto&mpl : mapid_and_pointid_list_) {
            if (mpl.second.map_id == map_id.first) {
                mpl.second.map_id = map_no;
            }
        }
        map_no++;
    }
    LOG(INFO) << "after update result_map_id size : " << result_map_id.size();
}

void MapSplitter::SplitMapDB() {
    if (result_map_id_.size() < 1) return;
    DB_IO map_db(local_data_path_ + "map.db");
    OriginPointInformation origin;
    map_db.ReadOriginPointInformationByDB(origin);
    for (auto& map : result_map_id_) {
        IdTypeKFs_map ids = map.second;
        std::string sub_map_name = "map_" + std::to_string(map.first) + ".db";
        DB_IO sub_map_db(local_data_path_ + sub_map_name);
        LOG(INFO) << "map size is : " << ids.size();
        for (auto &id : ids) {
            std::shared_ptr<KeyFrame> kf = std::make_shared<KeyFrame>();
            map_db.ReadSingleKF(id.second, kf);
            std::vector<std::shared_ptr<KeyFrame>> kfs;
            kfs.clear();
            kfs.push_back(kf);
            sub_map_db.WritePoseAndCloudToDB(kfs);
        }
        sub_map_db.WriteOriginPointInformationToDB(origin);
        if (save_debug_pcd_) {
            LOG(INFO) << "save_debug_pcd is : " << save_debug_pcd_;
            std::string map_pcd_name = "map_" + std::to_string(map.first) + ".pcd";
            SavePCDWithMapDB(local_data_path_ + map_pcd_name, local_data_path_ + sub_map_name);
        }
    }
}

int MapSplitter::GenerateTopology(std::map<IdType, TOPOS>& topos, const bool& model) {
    IdType last_map_no = ULONG_MAX;
    IdType last_id = ULONG_MAX;
    for (auto&mpl : mapid_and_pointid_list_) {
        IdType map_no = mpl.second.map_id;
        IdType id = mpl.first;
        double dis = 0.0;
        if (last_id != ULONG_MAX) {
            dis = ComputeDistance(ref_kfs_.at(last_id), ref_kfs_.at(id));
        }
        if (last_map_no != ULONG_MAX 
            && last_map_no != map_no 
            && (dis <= 3.0)) {
            TopoAttributes topo;
            topo.first_kf_id = last_id;
            topo.second_kf_id = id;
            topo.map_first_id = last_map_no;
            topo.map_second_id = map_no;
            topos.at(last_map_no).push_back(topo);
            topo.first_kf_id = id;
            topo.second_kf_id = last_id;
            topo.map_first_id = map_no;
            topo.map_second_id = last_map_no;
            topos.at(map_no).push_back(topo);           
        }
        last_map_no = map_no;
        last_id = id;
    }

    if (save_debug_pcd_) {
        for (auto&topo : topos) {
            std::cout << "map no : " << topo.first << " -> ";
            for (auto& tp : topo.second) {
                std::cout << " (" << tp.first_kf_id << ") " 
                          << tp.map_second_id << " "
                          << " (" << tp.second_kf_id << ") ";
            }
            std::cout << std::endl;
        }
    }
    if (model) {
        SaveTopology(topos);
    }
    return 0;

}

double MapSplitter::ComputeDistance(const PointAttributes& first, 
                                    const PointAttributes& second,
                                    bool model) {
    double dis = 0.0;
    dis = hypot(first.optimized_pose.translation()[0] - second.optimized_pose.translation()[0],
                first.optimized_pose.translation()[1] - second.optimized_pose.translation()[1]);
    if (!model) {
        return dis;
    } else {
        if (fabs(first.optimized_pose.translation()[2] - second.optimized_pose.translation()[2]) >= height_threshold) {
            dis = 2 * search_ranger;
        }
        return dis;
    }
}

bool MapSplitter::IsSameLevel(const PointAttributes& point, 
                              const IdTypeKFs_map& ids,
                              double resolution) {
    double dis = 0.0;
    for (auto& id : ids) {
        dis = hypot(point.optimized_pose.translation()[0] - ref_kfs_.at(id.second).optimized_pose.translation()[0],
                point.optimized_pose.translation()[1] - ref_kfs_.at(id.second).optimized_pose.translation()[1]);
        if (dis > search_ranger * resolution) continue;
        if (fabs(point.optimized_pose.translation()[2] - ref_kfs_.at(id.second).optimized_pose.translation()[2]) >= height_threshold) {
            return false;
        }
    }
    return true;
}

IdTypeKFs MapSplitter::ComputeApproachPoints(const PointAttributes& point) {
    IdTypeKFs ids;
    ids.clear();
    for (auto&kf : ref_kfs_) {
        double dis = ComputeDistance(point, kf);
        if (dis < search_ranger) {
            ids.push_back(kf.id);
        }
    }
    if (ids.size() < 1) {
        LOG(ERROR) << "error ";
    }
    return ids;
}

void MapSplitter::SaveTopology(const std::map<IdType, TOPOS>& topos) {
    if (result_map_id_.size() < 1) return;
    try {
        topo_file_.open(topo_path_.c_str());
        LOG(INFO) << " topo_path :" << topo_path_;
    } catch (...) {
        LOG(ERROR) << " 打开文件失败"; 
        return ;
    }

    topo_file_ << std::fixed;
    LOG(INFO) << topos.size();
    for (auto&topo : topos) {
        topo_file_ << topo.first << " ";
        for (auto& tp : topo.second) {
            topo_file_ << tp.map_second_id << " " << tp.second_kf_id << " ";
        }
        topo_file_ << std::endl;
    }
    topo_file_.close();
}

void MapSplitter::ShowMCSINFO(const MCS& mcs) {
    if (mcs.size() < 1) {
        LOG(INFO) << " map connection size : 0";
        return;
    }
    LOG(INFO) << " ----- show map connect info begin ----- ";
    LOG(INFO) << " map id : " << mcs.at(0).map_id;
    for (auto& mc : mcs) {
        LOG(INFO) << " bef map : " << mc.bef_map_id << ", aft map : " << mc.aft_map_id 
                  << " bef map connect id: " << mc.bef_connect_id << ", aft map connect id: " << mc.aft_connect_id 
                  << " begin : " << mc.map_begin_id <<", end : " << mc.map_end_id;
    }
    LOG(INFO) << " ----- show map connect info end ----- ";
}

}  // namespace tools
}  // namespace mapping
