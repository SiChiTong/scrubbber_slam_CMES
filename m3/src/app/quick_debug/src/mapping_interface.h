//
// Created by gaoxiang on 2021/2/3.
//

#ifndef QUICK_DEBUG_MAPPING_INTERFACE_H
#define QUICK_DEBUG_MAPPING_INTERFACE_H
#include <g2o/core/hyper_graph.h>
#include <atomic>
#include <memory>

#include "algorithm/common.h"
#include "algorithm/viewport.h"
#include "common/merge_info.h"
#include "common/num_type.h"
#include "dialog.h"
#include "optimizationdialog.h"
#include "renderGL/camera.h"
#include "renderGL/position_transform_node.h"
#include "renderGL/render_info.h"
#include "renderGL/render_leaf.h"

class MapDataLoader;

namespace mapping {

/// 建图数据与操作
/// 也包括一些GUI方面的数据
class MappingInterface {
   public:
    MappingInterface();
    ~MappingInterface();

    // 初始化
    void Init();

    // 重置大小
    void Resize(int x, int y, int width, int height);

    // 设置2D相机
    void Set2DView(bool switch_to_this_frame = false);

    // 绘制地图
    void Draw();

    // 坐标拾取接口
    void MousePointToCart(int x, int y, double &dx, double &dy);

    void UpdateBackground(const HAMO::Matrix4x4f &Tvs);

    // 左键逻辑
    void LButton(float x, float y);

    // 右键逻辑
    Eigen::Vector3f RButton(float x, float y, IdType &id, bool &found);

    // 移动逻辑
    void MouseMove(float x, float y);

    // 旋转逻辑
    void MouseRotate(float dx, float dy);

    /// 缩放逻辑
    void Zoom(float f_zoom);

    void ResetMapScale() { cam_height_ = 1.0; }

    /// 解析一个YAML文件
    bool ParseYAML(const std::string &yaml_path);

    /// 根据指定ID，将当前帧设置为该点云
    bool UpdateCurrentCloud(IdType selected_index, SE3 &coordinate, bool camera_follow_current = true);

    // 设置是否显示全局点云
    void SetShowPointCloud(bool show);

    void FinetuneX(int selected_index, double arg);
    void FinetuneY(int selected_index, double arg);
    void FinetuneZ(int selected_index, double arg);
    void FinetuneRoll(int selected_index, double arg);
    void FinetunePitch(int selected_index, double arg);
    void FinetuneYaw(int selected_index, double arg);

    int AddLoop(int id1, int id2, Dialog::LoopCloseParams &params);

    void UpdateLoopFirst(const int first_index);

    void UpdateLoopSecond(const int second_index);

    int GetKeyframeSize();

    /// 如果idx不满足条件，给出合适的index
    IdType GetSuggestKfIdx(IdType idx);

    int LoopClosing();

    void FixCurrent(int id);

    bool CallOptimization(std::vector<int> &fixed_kf_ids, std::string &report,
                          const OptimizationDialog::OptimizationParams &params);

    void SetPointCloudResolution(const double res);

    /// 获取相机位置
    V3d GetCameraPosition();

    void UpdateTrajectory(const HAMO::Matrix4x4f &Tsv);

    void SetShowTrack(const bool show) { show_track_ = show; }

    void SetShowPoseGraph(const bool show) { show_pose_graph_ = show; }

    void SetShowLoopFrames(const bool show);

    void UpdatePoseGraph(const HAMO::Matrix4x4f &Tsv);

    bool SaveMap();

    void CloseMap();

    bool SavePcd();

    void ResetCamera();

    void SetRubberBandPoint();

    void SetTrackId(const IdType track_id);

    void FixCurrentTrack();

    void ClearFixedTracks();

    void Switch2Track();

    void FixClosedLoopFrames();

    void ClearClosedLoopFrames();

    void FixGnssFixedSolution();

   private:
    Eigen::Vector3f BackProjection(const Eigen::Vector2f &p) const;

   private:
    HAMO::Viewport viewport_;
    HAMO::RenderInfo render_info_;

    HAMO::V3f start_mouse_;

    std::shared_ptr<HAMO::Camera> camera_ = nullptr;
    std::shared_ptr<HAMO::StereoCamera> stereo_camera_;

    float cam_height_ = 50.0;

    std::vector<std::shared_ptr<HAMO::RenderLeaf>> grids_render_leaf_;       // 网格线渲染元素
    std::vector<std::shared_ptr<HAMO::RenderLeaf>> trajectory_render_leaf_;  // 轨迹的渲染元素
    std::vector<std::shared_ptr<HAMO::RenderLeaf>> pose_graph_render_leaf_;  // 闭环的渲染元素

    std::shared_ptr<MapDataLoader> data_loader_;

    HAMO::V3d center_;

    std::string yaml_path_;

    std::atomic<bool> show_track_;
    std::atomic<bool> show_pose_graph_;
    std::atomic<bool> show_single_track_;
    std::atomic<bool> view_init_;

    IdType track_id_ = 0;

    common::MergeInfoVec merge_info_vec_;
};
}  // namespace mapping

#endif  // QUICK_DEBUG_MAPPING_INTERFACE_H
