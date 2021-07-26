//
// Created by gaoxiang on 2021/2/18.
//

#ifndef QUICK_DEBUG_MAP_DATA_LOADER_H
#define QUICK_DEBUG_MAP_DATA_LOADER_H

#include <atomic>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <g2o/core/hyper_graph.h>
#include <pcl/filters/voxel_grid.h>

#include <glog/logging.h>

#include "algorithm/mc_math.h"
#include "color_interp.h"
#include "common/candidate.h"
#include "common/keyframe.h"
#include "core/mt_search/multi_thread_search.h"
#include "dialog.h"
#include "io/db_io.h"
#include "optimizationdialog.h"
#include "pipeline/optimization_s2/optimization_s2.h"
#include "renderGL/camera.h"
#include "renderGL/point_drawable.h"
#include "renderGL/render_info.h"
#include "renderGL/render_leaf.h"

typedef std::map<size_t, std::set<size_t>> PoseGraphType;

/// 地图数据加载
/// 根据视点位置进行剪裁和渲染
class MapDataLoader {
   public:
    MapDataLoader();
    ~MapDataLoader();

    /// 读取的数据与渲染的数据
    struct LoadData {
        LoadData(mapping::common::KFPtr kfptr)
            : point_drawable_(new HAMO::PointDrawable()), render_leaf_(new HAMO::RenderLeaf) {
            keyframe_ptr_ = kfptr;
        }

        void Clean() {
            keyframe_ptr_ = nullptr;
            if (render_leaf_) {
                render_leaf_->m_drawable = nullptr;  // don't call delete
            }
            render_leaf_ = nullptr;
            point_drawable_ = nullptr;
        }

        ~LoadData() { Clean(); }

        bool enable = true;
        mapping::common::KFPtr keyframe_ptr_ = nullptr;
        std::shared_ptr<HAMO::PointDrawable> point_drawable_ = nullptr;
        std::shared_ptr<HAMO::RenderLeaf> render_leaf_ = nullptr;
    };

    struct Pos3d {
        V3f pos;

        Pos3d(V3f v) { pos = v; }

        bool operator<(const Pos3d& v) const {
            return (pos[0] < v.pos[0] || (fabs(pos[0] - v.pos[0]) < FLT_EPSILON && pos[1] < v.pos[1]));
        }
    };

    static inline Color GetIntensityColor(float intensity) {
        static std::vector<Color> iColor;

        if (iColor.size() <= 0) {
            for (int i = 0; i < 256; i++) {
                Color p;
                p.r = 0;
                p.g = i;
                p.b = 255;
                iColor.push_back(p);
            }

            for (int i = 0; i < 256; i++) {
                Color p;
                p.r = 0;
                p.g = 255;
                p.b = 255 - i;
                iColor.push_back(p);
            }
            for (int i = 0; i < 256; i++) {
                Color p;
                p.r = i;
                p.g = 255;
                p.b = 0;
                iColor.push_back(p);
            }
            for (int i = 0; i < 256; i++) {
                Color p;
                p.r = 255;
                p.g = 255 - i;
                p.b = 0;
                iColor.push_back(p);
            }
            for (int i = 0; i < 256; i++) {
                Color p;
                p.r = 255;
                p.g = 0;
                p.b = i;
                iColor.push_back(p);
            }
            for (int i = 0; i < 256; i++) {
                Color p;
                p.r = 255 - i;
                p.g = 0;
                p.b = 255;
                iColor.push_back(p);
            }
        }

        intensity = HAMO::Mathf::Clamp(intensity, 0.0f, 255.0f);
        int index = (int)(intensity * 6);
        return iColor[index];
    }

    enum class COLOR { WHITE, RED, YELLOW, GREEN, BLUE };

   public:
    /**
     * 指定DB文件开始载入
     * @param db_path
     * @param keyframe_txt_path keyframe.txt位置
     * @return
     */
    bool LoadDB(const std::string& directory);

    // 渲染点云
    void Draw(HAMO::RenderInfo& render_info);

    // 指定视点中心
    void SetViewCenter(double x, double y);

    // 停止加载并退出
    void Stop();

    /// 更新相机矩阵
    void Update(const HAMO::Matrix4x4f& svMatrix, std::shared_ptr<HAMO::Camera> pCamera);

    bool IsLoading() { return loading_; }
    bool IsRunning() { return running_; }

    void SetShowGlobalCloud(bool show) { show_global_cloud_ = show; }

    /// 更新当前点云
    void UpdateCurrentCloud(IdType selected_index, SE3& coordinate,
                            const HAMO::V4f& color = COLOR_MAP_.at(COLOR::WHITE));

    void FinetuneX(int selected_index, double arg);
    void FinetuneY(int selected_index, double arg);
    void FinetuneZ(int selected_index, double arg);
    void FinetuneRoll(int selected_index, double arg);
    void FinetunePitch(int selected_index, double arg);
    void FinetuneYaw(int selected_index, double arg);

    int AddLoop(const int id1, const int id2, const std::string yaml_file, Dialog::LoopCloseParams& params);

    SE3 UpdateLoop(int index, COLOR color);

    /// 获取keyframe最大值
    int GetKeyframeSize() { return keyframe_map_.rbegin()->first; }

    void FixCurrent(int id);

    void ResetLeafSize(const double res);

    std::map<IdType, mapping::common::KFPtr> GetKfMap() { return keyframe_map_; }

    bool PoseOptimization(const std::string yaml_path, const std::vector<int>& fixed_kf_ids, std::string& report,
                          const OptimizationDialog::OptimizationParams& params);

    void RebuildDisplayCloud();

    /// 获取Pose Graph，用于显示回环
    PoseGraphType GetPoseGraph() const { return pose_graph_; }

    IdType GetSuggestKfIdx(IdType idx);

    void SetMarkingKf1() { mark_kf1_ = true; }

    void SetMarkingKf2() { mark_kf2_ = true; }

    bool SaveMap() {
        if (optimizer_ == nullptr) {
            return false;
        } else {
            optimizer_->UpdateDb();
        }
        return true;
    }

    void CloseMap();

    bool SavePcd() {
        if (optimizer_ == nullptr) {
            return false;
        } else {
            optimizer_->SavePcd();
        }
        return true;
    }

    std::map<Pos3d, IdType> GetIndexMap() { return index_map_; }

    std::shared_ptr<g2o::HyperGraph> GetGraph() { return graph_; }

    void SetShowLoopFrames(const bool show) { show_loop_frames_ = show; }

    void SetRubberBandPoint(const Eigen::Vector3f& point);

    void SetTrackId(const IdType track_id, bool show_single_track) {
        show_single_track_ = show_single_track;
        track_id_ = track_id;
    };

    void SetTrackId(const IdType track_id) { track_id_ = track_id; };

    void FixCurrentTrack(const IdType id) { fixed_kfs_[id] = true; }

    void ClearFixedTracks() { std::map<int, bool>().swap(fixed_kfs_); }

    void Switch2Track(bool show_single_track) { show_single_track_ = show_single_track; }

    void FixClosedLoopFrames() {
        for (const auto& i : lc_) {
            if (std::find(fixed_kfs_vec_.begin(), fixed_kfs_vec_.end(), i.kfid_first) == fixed_kfs_vec_.end()) {
                fixed_kfs_vec_.emplace_back(i.kfid_first);
                LOG(INFO) << "i.kfid_first :" << i.kfid_first;
            }
            if (std::find(fixed_kfs_vec_.begin(), fixed_kfs_vec_.end(), i.kfid_second) == fixed_kfs_vec_.end()) {
                fixed_kfs_vec_.emplace_back(i.kfid_second);
                LOG(INFO) << "i.kfid_second : " << i.kfid_second;
            }
        }
    }

    void ClearClosedLoopFrames() {
        for (const auto& i : lc_) {
            if (std::find(fixed_kfs_vec_.begin(), fixed_kfs_vec_.end(), i.kfid_first) != fixed_kfs_vec_.end()) {
                fixed_kfs_vec_.erase(std::find(fixed_kfs_vec_.begin(), fixed_kfs_vec_.end(), i.kfid_first));
            }
            if (std::find(fixed_kfs_vec_.begin(), fixed_kfs_vec_.end(), i.kfid_second) != fixed_kfs_vec_.end()) {
                fixed_kfs_vec_.erase(std::find(fixed_kfs_vec_.begin(), fixed_kfs_vec_.end(), i.kfid_second));
            }
        }
    }

    void FixGnssFixedSolution() { fix_gnss_fixed_solution_ = true; }

   private:
    void PointCloudLoadThread();

    /// 根据view center来确定要读取的point cloud
    /// 并生成point drawable和render leaf
    void LoadNecessaryPointCloud();

    // 读取单个关键帧
    // render leaf中的点云是在world系下的，所以model系和world系重合
    std::shared_ptr<LoadData> LoadSingleKF(mapping::common::KFPtr kf, bool custom_color = false,
                                           HAMO::V4f color = {0.0, 0.0, 0.0, 0.0});

    /// 从已经读取的点云中生成当前帧
    bool CreateCurrentKFFromLoaded(IdType id, const HAMO::V4f& color);

    // 卸载单个关键帧
    void UnLoadSingleKF(mapping::common::KFPtr kf);

    void UpdateSingleKF(mapping::common::KFPtr kf);

    void DetectNodesConnectivityCandidate(const int n1, const int n2, const std::string method = "NDT");

    void AddPair(size_t vi, size_t vj) {
        auto i1 = pose_graph_.find(vi);
        if (i1 == pose_graph_.end()) {
            pose_graph_.insert({vi, {vj}});
        } else {
            auto i2 = i1->second.find(vj);
            if (i2 == i1->second.end()) {
                i1->second.insert(vj);
            }
        }
    }

    bool UpdateLoadedKfs();

    std::string db_path_;
    std::string keyframe_txt_path_;
    std::string hyper_graph_path_;

    std::map<IdType, mapping::common::KFPtr> keyframe_map_;
    std::map<Pos3d, IdType> index_map_;

    std::atomic<bool> loading_;  // 指示加载过程是否正在运行
    std::atomic<bool> running_;  // 指示整个加载器是否应该运行
    std::shared_ptr<mapping::io::DB_IO> db_io_ = nullptr;

    std::mutex render_mutex_;                 // 渲染时不加载新数据
    std::atomic<bool> show_global_cloud_;     // 是否显示整个点云
    std::atomic<bool> rebuild_optimization_;  // 是否需要重跑优化

    std::thread load_thread_;  // 加载线程

    V2d view_center_ = V2d::Zero();

    std::map<IdType, std::shared_ptr<LoadData>> loaded_kfs_;  // 被载入的关键帧
    std::shared_ptr<LoadData> current_kf_ = nullptr;          // 当前关键帧
    std::shared_ptr<LoadData> kf1_ = nullptr;                 // 关键帧1 小蓝
    std::shared_ptr<LoadData> kf2_ = nullptr;                 // 关键帧2 小绿

    std::atomic<bool> update_needed_;  // 更新位置后，应该刷新缓存
    std::atomic<bool> show_loop_frames_;
    std::atomic<bool> first_point_set_;
    std::atomic<bool> second_point_set_;

    static constexpr double load_distance_near_ = 50.0;  // 读取半径（小）
    static constexpr double load_distance_far_ = 100.0;  // 读取半径（大），超出此半径后场景被卸载

    pcl::VoxelGrid<mapping::common::PointType> voxel_filter_;

    std::vector<mapping::common::LoopCandidate> loop_candidates_;  // 可能存在的回环
    std::vector<int> fixed_keyframes_;

    const static std::map<COLOR, HAMO::V4f> COLOR_MAP_;

    PoseGraphType pose_graph_;  // 用于显示

    bool mark_kf1_ = false;
    bool mark_kf2_ = false;

    std::shared_ptr<mapping::pipeline::OptimizationStage2> optimizer_ = nullptr;
    std::shared_ptr<g2o::HyperGraph> graph_;  // g2o graph

    Eigen::Vector3f first_point_;
    Eigen::Vector3f second_point_;

    std::atomic<bool> show_single_track_;
    std::atomic<bool> read_pose_from_db_;
    std::atomic<bool> fix_gnss_fixed_solution_;

    IdType track_id_ = 0;

    std::map<int, bool> fixed_kfs_;
    std::map<int, SE3> map_id_pose_;

    std::vector<mapping::common::LoopCandidate> lc_;
    std::vector<int> fixed_kfs_vec_;
};
#endif  // QUICK_DEBUG_MAP_DATA_LOADER_H
