//
// Created by gaoxiang on 2021/2/3.
//

#include "mapping_interface.h"

#include <g2o/types/slam3d/vertex_se3.h>
#include <glog/logging.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "io/file_io.h"
#include "io/yaml_io.h"
#include "map_data_loader.h"
#include "pipeline/loop_closing/loop_closing.h"
#include "renderGL/factory_drawable.h"
#include "renderGL/graph_drawable.h"
#include "renderGL/line_drawable.h"
#include "renderGL/mc_technique_manager.h"
#include "renderGL/point3d.h"
#include "renderGL/render_leaf.h"

using namespace HAMO;

namespace mapping {

MappingInterface::MappingInterface() : stereo_camera_(new HAMO::StereoCamera), data_loader_(new MapDataLoader()) {
    camera_ = stereo_camera_;
    std::atomic_init(&show_track_, true);
    std::atomic_init(&show_pose_graph_, false);
    std::atomic_init(&show_single_track_, false);
    std::atomic_init(&view_init_, false);
}

MappingInterface::~MappingInterface() = default;

void MappingInterface::Init() {
    render_info_.SetState(new State);
    TechniqueManager::GetInstance()->Initialize();

    // 网格线
    const double grid_size = 1.0;
    const double dminX = -100;
    const double dminY = -100;
    const double dmaxX = 100.0;
    const double dmaxY = 100.0;

    std::vector<Point3d> line_seg[2];
    for (int8_t i = dminX; i <= dmaxX; i += grid_size) {
        Point3d pt1, pt2;
        pt1.x = i;
        pt1.y = dminY;
        pt2.x = i;
        pt2.y = dmaxY;

        line_seg[0].push_back(pt1);
        line_seg[0].push_back(pt2);
    }

    for (int8_t j = dminY; j <= dmaxY; j += grid_size) {
        Point3d pt1, pt2;
        pt1.x = dminX;
        pt1.y = j;
        pt2.x = dmaxX;
        pt2.y = j;

        line_seg[1].push_back(pt1);
        line_seg[1].push_back(pt2);
    }

    //------------------------------------------

    double cntx = (dminX + dmaxX) * 0.5;
    double cnty = (dminY + dmaxY) * 0.5;

    for (auto &seg : line_seg) {
        if (seg.size() > 1) {
            auto *pLine = new LineDrawable();

            for (size_t i = 0; i < seg.size(); i += 2) {
                Point3d p1, p2;

                p1 = seg[i];
                p2 = seg[i + 1];

                if ((i / 2) % 2 == 0) {
                    pLine->add(p1.x - cntx, p1.y - cnty, 0.0, HAMO::V3f(0.2, 0.2, 0.2));
                    pLine->add(p2.x - cntx, p2.y - cnty, 0.0, HAMO::V3f(0.2, 0.2, 0.2));
                } else {
                    pLine->add(p2.x - cntx, p2.y - cnty, 0.0, HAMO::V3f(0.2, 0.2, 0.2));
                    pLine->add(p1.x - cntx, p1.y - cnty, 0.0, HAMO::V3f(0.2, 0.2, 0.2));
                }
            }

            auto pRenderLeaf = std::make_shared<RenderLeaf>();
            pRenderLeaf->SetDrawable(pLine);
            pRenderLeaf->SetRenderTechnique(TechniqueManager::GetInstance()->GetTechnique(4));
            pRenderLeaf->SetViewport(camera_->GetViewport());
            pRenderLeaf->SetProjectionMatrix(camera_->GetProjectionMatrix());

            grids_render_leaf_.push_back(pRenderLeaf);
        }
    }
}

void MappingInterface::Resize(int x, int y, int width, int height) {
    viewport_.SetViewport(0, 0, width, height);

    stereo_camera_->SetViewport(viewport_);
    stereo_camera_->SetPerspective(100.0f, viewport_.AspectRatio(), 0.5f, 500.0f);

    // TODO 3D camera
}

void MappingInterface::Set2DView(bool switch_to_this_frame) {
    if (switch_to_this_frame) {
        stereo_camera_->SetPosition(HAMO::V3d(center_.x, center_.y, camera_->GetPosition().z));
    } else if (camera_) {
        stereo_camera_->SetPosition(camera_->GetPosition());
    } else {
        stereo_camera_->SetPosition(HAMO::V3d(0, 0, 0.0));
    }

    stereo_camera_->SetPerspective(100.0f, viewport_.AspectRatio(), 0.5f, 500.0f);
    camera_ = stereo_camera_;
}

void MappingInterface::Draw() {
    render_info_.GetState()->ApplyViewport(&viewport_);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 背景参考线生成并渲染

    HAMO::V3d center = stereo_camera_->GetPosition();
    data_loader_->SetViewCenter(center.x, center.y);

    Matrix4x4f Tsv = camera_->GetViewMatrix();
    data_loader_->Update(Tsv, camera_);

    UpdateBackground(Tsv);
    if (show_track_) {
        UpdateTrajectory(Tsv);

        // 渲染轨迹
        for (auto &r : trajectory_render_leaf_) {
            r->Render(render_info_, nullptr);
        }
    }
    if (show_pose_graph_) {
        UpdatePoseGraph(Tsv);

        // 渲染pose graph
        if (!pose_graph_render_leaf_.empty()) {
            for (auto &r : pose_graph_render_leaf_) {
                if (r != nullptr) {
                    r->Render(render_info_, nullptr);
                }
            }
        }
    }

    // 渲染网格
    for (auto &r : grids_render_leaf_) {
        r->Render(render_info_, nullptr);
    }

    // 渲染地图
    if (data_loader_) {
        data_loader_->Draw(render_info_);
    }
}

void MappingInterface::UpdateBackground(const Matrix4x4f &Tvs) {
    for (auto &r : grids_render_leaf_) {
        r->SetModelViewMatrix(Tvs);
        r->SetProjectionMatrix(stereo_camera_->GetProjectionMatrix());
    }
}

void MappingInterface::MousePointToCart(int x, int y, double &dx, double &dy) {
    HAMO::V3d position = stereo_camera_->GetPosition();

    HAMO::V3f vec3;
    vec3[0] = x * 2.0f / viewport_.Width() - 1.0f;
    vec3[1] = 1.0f - y * 2.0f / viewport_.Height();

    Matrix4x4f invViewProj = stereo_camera_->GetViewProjMatrix().Inverse();
    HAMO::V3f temp1 = invViewProj.TransformCoord(vec3);

    dx = temp1[0] + position[0];
    dy = temp1[1] + position[1];
}

void MappingInterface::LButton(float x, float y) {
    start_mouse_[0] = x;
    start_mouse_[1] = y;
}

void MappingInterface::MouseMove(float x, float y) {
    HAMO::V3f vec1, vec2;

    vec1[0] = x * 2.0f / viewport_.Width() - 1.0f;
    vec1[1] = 1.0f - y * 2.0f / viewport_.Height();
    vec2[0] = start_mouse_[0] * 2.0f / viewport_.Width() - 1.0f;
    vec2[1] = 1.0f - start_mouse_[1] * 2.0f / viewport_.Height();

    Matrix4x4f invProj = camera_->GetViewProjMatrix().Inverse();
    HAMO::V3f vec = invProj.TransformNormal(vec1 - vec2);

    HAMO::V3d dir(vec[0], vec[1], vec[2]);

    double factor = camera_->GetPosition().z * 0.5;
    camera_->MoveCamera(dir, -dir.norm() * factor);

    //----------------------------------------
    start_mouse_[0] = x;
    start_mouse_[1] = y;
}

void MappingInterface::MouseRotate(float dx, float dy) {
    // TODO 3D视角下旋转地图
    camera_->Rotate(dx, dy);
}

::V3d MappingInterface::GetCameraPosition() { return camera_->GetPosition().ToEigen(); }

void MappingInterface::Zoom(float f_zoom) {
    cam_height_ = cam_height_ * f_zoom;
    //    LOG(INFO) << "height: " << cam_height_;
    camera_->SetHeight(cam_height_);
}

bool MappingInterface::ParseYAML(const std::string &yaml_path) {
    yaml_path_ = yaml_path;
    io::YAML_IO yaml(yaml_path);
    if (yaml.IsOpened() == false) {
        return false;
    }

    auto local_data_path = yaml.GetValue<std::string>("data_fetching", "local_data_path");
    std::string db_path = local_data_path + "/map.db";
    std::string keyframe_txt_path = local_data_path + "/keyframes.txt";

    if (io::PathExists(db_path) == false) {
        LOG(ERROR) << "cannot locate map.db at " << db_path;
        return false;
    }

    if (io::PathExists(keyframe_txt_path) == false) {
        LOG(ERROR) << "cannot locate keyframes.txt at " << keyframe_txt_path;
        return false;
    }

    if (!io::LoadMergeInfo(local_data_path + "merge_info.txt", merge_info_vec_)) {
        LOG(ERROR) << "Cannot load keyframes at " << local_data_path << "merge_info.txt\n";
    }

    return data_loader_->LoadDB(local_data_path);
}

bool MappingInterface::UpdateCurrentCloud(IdType selected_index, SE3 &coordinate, bool camera_follow_current) {
    data_loader_->UpdateCurrentCloud(selected_index, coordinate);

    if (camera_follow_current) {
        center_.x = coordinate.translation()[0];
        center_.y = coordinate.translation()[1];

        Set2DView(true);
    }

    return true;
}

void MappingInterface::FinetuneX(int selected_index, double arg) { data_loader_->FinetuneX(selected_index, arg); }

void MappingInterface::FinetuneY(int selected_index, double arg) { data_loader_->FinetuneY(selected_index, arg); }

void MappingInterface::FinetuneZ(int selected_index, double arg) { data_loader_->FinetuneZ(selected_index, arg); }

void MappingInterface::FinetuneRoll(int selected_index, double arg) { data_loader_->FinetuneRoll(selected_index, arg); }

void MappingInterface::FinetunePitch(int selected_index, double arg) {
    data_loader_->FinetunePitch(selected_index, arg);
}

void MappingInterface::FinetuneYaw(int selected_index, double arg) { data_loader_->FinetuneYaw(selected_index, arg); }

int MappingInterface::AddLoop(int id1, int id2, Dialog::LoopCloseParams &params) {
    return data_loader_->AddLoop(id1, id2, yaml_path_, params);
}

void MappingInterface::UpdateLoopFirst(const int first_index) {
    data_loader_->SetMarkingKf1();

    SE3 coordinate = data_loader_->UpdateLoop(first_index, MapDataLoader::COLOR::BLUE);

    center_.x = coordinate.translation()[0];
    center_.y = coordinate.translation()[1];

    Set2DView(true);
}

void MappingInterface::UpdateLoopSecond(const int second_index) {
    data_loader_->SetMarkingKf2();

    SE3 coordinate = data_loader_->UpdateLoop(second_index, MapDataLoader::COLOR::GREEN);

    center_.x = coordinate.translation()[0];
    center_.y = coordinate.translation()[1];

    Set2DView(true);
}

int MappingInterface::GetKeyframeSize() { return data_loader_->GetKeyframeSize(); }

IdType MappingInterface::GetSuggestKfIdx(IdType idx) { return data_loader_->GetSuggestKfIdx(idx); }

int MappingInterface::LoopClosing() {
    if (yaml_path_.empty()) {
        LOG(ERROR) << "config yaml is empty";
        return -1;
    }

    mapping::io::YAML_IO yaml(yaml_path_);
    if (yaml.IsOpened() == false) {
        LOG(ERROR) << "failed to open yaml";
        return -1;
    }

    mapping::pipeline::LoopClosing loop_closing(yaml);

    if (loop_closing.Init() == false) {
        LOG(ERROR) << "failed to init loop closing";
        return -1;
    }

    if (!loop_closing.Start()) {
        LOG(ERROR) << "failed to start loop closing";
        return -1;
    }

    LOG(INFO) << "loop closing done.";

    return 0;
}

void MappingInterface::FixCurrent(int id) { data_loader_->FixCurrent(id); }

bool MappingInterface::CallOptimization(std::vector<int> &fixed_kf_ids, std::string &report,
                                        const OptimizationDialog::OptimizationParams &params) {
    if (yaml_path_.empty()) {
        LOG(ERROR) << "config yaml is empty";
        return false;
    }

    if (data_loader_->PoseOptimization(yaml_path_, fixed_kf_ids, report, params)) {
        LOG(INFO) << "opti s2 done.";
        return true;
    } else {
        return false;
    }
}

void MappingInterface::SetPointCloudResolution(const double res) {
    data_loader_->ResetLeafSize(res);
    data_loader_->RebuildDisplayCloud();
}

void MappingInterface::SetShowPointCloud(bool show) { data_loader_->SetShowGlobalCloud(show); }

void MappingInterface::UpdateTrajectory(const HAMO::Matrix4x4f &Tsv) {
    trajectory_render_leaf_.clear();

    auto raw_keyframe = data_loader_->GetKfMap();

    auto *pLine = new LineDrawable();

    for (auto iter = raw_keyframe.begin(); iter != raw_keyframe.end(); ++iter) {
        if (show_single_track_) {
            if (iter->second->trajectory_id_ != track_id_) {
                continue;
            } else {
                if (false == view_init_) {
                    center_.x = iter->second->optimized_pose_stage_2_.translation()[0];
                    center_.y = iter->second->optimized_pose_stage_2_.translation()[1];

                    Set2DView(true);

                    view_init_ = true;
                }
            }
        }
        auto iter_next = iter;
        ++iter_next;
        if (iter_next == raw_keyframe.end()) {
            break;
        }

        Point3d pt1, pt2;
        pt1.x = iter->second->optimized_pose_stage_2_.translation()[0];
        pt1.y = iter->second->optimized_pose_stage_2_.translation()[1];
        pt1.z = iter->second->optimized_pose_stage_2_.translation()[2];
        pt2.x = iter_next->second->optimized_pose_stage_2_.translation()[0];
        pt2.y = iter_next->second->optimized_pose_stage_2_.translation()[1];
        pt2.z = iter_next->second->optimized_pose_stage_2_.translation()[2];

        pLine->add(pt1.x, pt1.y, 0.0, HAMO::V3f(0, 0.8, 0));
        pLine->add(pt2.x, pt2.y, 0.0, HAMO::V3f(0, 0.8, 0));
    }

    auto pRenderLeaf = std::make_shared<RenderLeaf>();
    pRenderLeaf->SetDrawable(pLine);
    pRenderLeaf->SetRenderTechnique(TechniqueManager::GetInstance()->GetTechnique(4));
    pRenderLeaf->SetModelViewMatrix(Tsv);
    pRenderLeaf->SetProjectionMatrix(camera_->GetProjectionMatrix());
    pRenderLeaf->SetViewport(camera_->GetViewport());

    trajectory_render_leaf_.push_back(pRenderLeaf);

    Drawable *pNode = FactoryDrawable::CreateNodeDrawable(pLine->GetPointArray(), 0.1, HAMO::V4f(0, 0, 1, 1.0));

    auto pRenderLeafPoint = std::make_shared<RenderLeaf>();
    pRenderLeafPoint->SetDrawable(pNode);
    pRenderLeafPoint->SetRenderTechnique(TechniqueManager::GetInstance()->GetTechnique(4));
    pRenderLeafPoint->SetModelViewMatrix(Tsv);
    pRenderLeafPoint->SetProjectionMatrix(camera_->GetProjectionMatrix());
    pRenderLeafPoint->SetViewport(camera_->GetViewport());

    trajectory_render_leaf_.push_back(pRenderLeafPoint);

    pLine = nullptr;
    pNode = nullptr;
}

void MappingInterface::UpdatePoseGraph(const HAMO::Matrix4x4f &Tsv) {
    pose_graph_render_leaf_.clear();

    auto graph = data_loader_->GetGraph();

    if (nullptr == graph || graph->edges().empty()) {
        return;
    }

    auto *pLine = new GraphDrawable();
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors;

    for (const auto &edge : graph->edges()) {
        if (edge->vertices().size() > 1) {
            auto v1 = dynamic_cast<g2o::VertexSE3 *>(edge->vertices()[0]);
            auto v2 = dynamic_cast<g2o::VertexSE3 *>(edge->vertices()[1]);

            if (nullptr != v1 && nullptr != v2) {
                Eigen::Vector3f p1 = v1->estimate().translation().cast<float>();
                Eigen::Vector3f p2 = v2->estimate().translation().cast<float>();

                GLfloat timeValue = clock() / CLOCKS_PER_SEC;
                GLfloat greenValue = (sin(timeValue) / 2) + 0.5;
                HAMO::V3f color(0.48627451, greenValue, 1);

                pLine->add(HAMO::V3f(p1.x(), p1.y(), 0), color);
                pLine->add(HAMO::V3f(p2.x(), p2.y(), 0), color);
            }
        }
    }

    pLine->Init();

    auto pRenderLeaf = std::make_shared<RenderLeaf>();
    pRenderLeaf->SetDrawable(pLine);
    pRenderLeaf->SetRenderTechnique(TechniqueManager::GetInstance()->GetTechnique(4));
    pRenderLeaf->SetModelViewMatrix(Tsv);
    pRenderLeaf->SetProjectionMatrix(camera_->GetProjectionMatrix());
    pRenderLeaf->SetViewport(camera_->GetViewport());

    pose_graph_render_leaf_.push_back(pRenderLeaf);

    auto pose_graph = data_loader_->GetPoseGraph();
    if (pose_graph.empty()) {
        return;
    }
    auto kf_map = data_loader_->GetKfMap();

    auto *pLineMarked = new GraphDrawable();
    for (const auto &edge_set : pose_graph) {
        size_t vi = edge_set.first;
        if (vi < 0 || vi >= kf_map.size()) {
            LOG(INFO) << "Index out of range";
            continue;
        }
        for (const auto &vj : edge_set.second) {
            if (vj < 0 || vj >= kf_map.size()) {
                LOG(INFO) << "Index out of range";
                continue;
            }
            pLineMarked->add(kf_map.at(vi)->optimized_pose_stage_2_.translation()[0],
                             kf_map.at(vi)->optimized_pose_stage_2_.translation()[1], 0, HAMO::V3f(0.95f, 0.0f, 0.0f));
            pLineMarked->add(kf_map.at(vj)->optimized_pose_stage_2_.translation()[0],
                             kf_map.at(vj)->optimized_pose_stage_2_.translation()[1], 0, HAMO::V3f(0.95f, 0.0f, 0.0f));
        }
    }

    pLineMarked->Init();

    auto pRenderMarkedLeaf = std::make_shared<RenderLeaf>();
    pRenderMarkedLeaf->SetDrawable(pLineMarked);
    pRenderMarkedLeaf->SetRenderTechnique(TechniqueManager::GetInstance()->GetTechnique(4));
    pRenderMarkedLeaf->SetModelViewMatrix(Tsv);
    pRenderMarkedLeaf->SetProjectionMatrix(camera_->GetProjectionMatrix());
    pRenderMarkedLeaf->SetViewport(camera_->GetViewport());

    pose_graph_render_leaf_.push_back(pRenderMarkedLeaf);

    pLine = NULL;
    pLineMarked = NULL;
}

bool MappingInterface::SaveMap() { return data_loader_->SaveMap(); }

void MappingInterface::CloseMap() { data_loader_->CloseMap(); }

bool MappingInterface::SavePcd() { return data_loader_->SavePcd(); }

Eigen::Vector3f MappingInterface::RButton(float x, float y, IdType &id, bool &found) {
    static Eigen::Vector3f picked_pos;
    picked_pos = BackProjection(Eigen::Vector2f(x, y));

    static const double thresh = 0.141421356;

    auto MyPredicate = [](const std::pair<MapDataLoader::Pos3d, IdType> &p) -> bool {
        return fabs(p.first.pos[0] - picked_pos[0]) <= thresh && fabs(p.first.pos[1] - picked_pos[1]) <= thresh;
    };

    auto index_map = data_loader_->GetIndexMap();

    auto iter = std::find_if(index_map.begin(), index_map.end(), MyPredicate);
    if (iter != index_map.end()) {
        id = iter->second;
        found = true;
    } else {
        found = false;
    }

    return picked_pos;
}

Eigen::Vector3f MappingInterface::BackProjection(const Eigen::Vector2f &p) const {
    Eigen::Matrix4f view_matrix = camera_->GetViewMatrix().ToEigen();
    glm::mat4 view = glm::make_mat4(view_matrix.data());

    Eigen::Matrix4f projection_matrix = camera_->GetProjectionMatrix().ToEigen();
    glm::mat4 projection = glm::make_mat4(projection_matrix.data());

    auto view_port = camera_->GetViewport();
    glm::vec4 viewport = glm::vec4(view_port.X(), view_port.Y(), view_port.Width(), view_port.Height());

    GLfloat winZ;

    float pixel_w = 1, pixel_h = 1;

    glm::vec2 screen_pos = glm::vec2(p[0], p[1]);
    glm::vec2 pixel_pos = screen_pos * glm::vec2(pixel_w, pixel_h) /
                          glm::vec2(view_port.Width(), view_port.Height());  // note: not necessarily integer
    pixel_pos = pixel_pos + glm::vec2(0.5f, 0.5f);                           // shift to GL's center convention
    glm::vec3 win = glm::vec3(pixel_pos.x, view_port.Height() - pixel_pos.y, 0.0f);

    glReadPixels((GLint)win.x, (GLint)win.y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);

    win.x = 2.0 * p[0] / (float)view_port.Width() - 1;
    win.y = -(win.y = 2.0f * p[1] / (float)view_port.Height()) + 1;

    auto near = glm::unProject(glm::vec3(p[0], view_port.Height() - p[1], 0), view, projection, viewport);
    auto far = glm::unProject(glm::vec3(p[0], view_port.Height() - p[1], 1), view, projection, viewport);

    float ratio = -near.z / (far.z - near.z);
    auto delta = (far - near) * ratio;

    return Eigen::Vector3f(near.x + delta.x, near.y + delta.y, near.z + delta.z);
}

void MappingInterface::ResetCamera() { camera_->ResetCamera(); }

void MappingInterface::SetShowLoopFrames(const bool show) { data_loader_->SetShowLoopFrames(show); }

void MappingInterface::SetRubberBandPoint() {}

void MappingInterface::SetTrackId(const IdType track_id) {
    track_id_ = track_id;
    LOG(INFO) << "track_id_ : " << track_id_;
};

void MappingInterface::FixCurrentTrack() {
    data_loader_->FixCurrentTrack(track_id_);
    view_init_ = false;
}

void MappingInterface::ClearFixedTracks() { data_loader_->ClearFixedTracks(); }

void MappingInterface::Switch2Track() {
    show_single_track_ = true;
    bool found = false;
    if (!merge_info_vec_.empty()) {
        for (const auto &i : merge_info_vec_) {
            if (track_id_ == i.trajectory_id_) {
                found = true;
                continue;
            }
        }
    } else if (track_id_ < 99) {
        found = true;
    }
    data_loader_->Switch2Track(found);
    show_single_track_ = found;
}

void MappingInterface::FixClosedLoopFrames() { data_loader_->FixClosedLoopFrames(); }

void MappingInterface::ClearClosedLoopFrames() { data_loader_->ClearClosedLoopFrames(); }

void MappingInterface::FixGnssFixedSolution() { data_loader_->FixGnssFixedSolution(); }

}  // namespace mapping