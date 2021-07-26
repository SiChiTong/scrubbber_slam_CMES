#include <QApplication>
#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QtConcurrent/QtConcurrent>

#include <glog/logging.h>
#include <pcl/filters/voxel_grid.h>

#include "debug_ui_mainwindow.h"
#include "ui_debug_ui_mainwindow.h"

#include "common/mapping_math.h"
#include "common/num_type.h"
#include "io/file_io.h"
#include "io/yaml_io.h"


using namespace mapping;
using namespace mapping::pipeline;

DebugUIMainWindow::DebugUIMainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      cloud_display_(new ViewCloudType()),
      current_kf_display_(new ViewCloudType()),
      loop_kf_first_(new ViewCloudType()),
      loop_kf_second_(new ViewCloudType()),
      color_handler_(
          pcl::visualization::PointCloudColorHandlerGenericField<ViewPointType>(cloud_display_, "intensity")),
      color_handler_current_(
          pcl::visualization::PointCloudColorHandlerCustom<ViewPointType>(current_kf_display_, 0.0, 0.9, 0.1)),
      color_handler_loop_first_(
          pcl::visualization::PointCloudColorHandlerCustom<ViewPointType>(current_kf_display_, 0.0, 0.9, 0.1)),
      color_handler_loop_second_(
          pcl::visualization::PointCloudColorHandlerCustom<ViewPointType>(current_kf_display_, 0.0, 0.9, 0.1)),
      play_timer_(new QTimer(this)),
      current_keyframe_(new common::KeyFrame()) {
    ui->setupUi(this);

    for (int i = 0; i < available_resolution_.size(); ++i) {
        ui->resolution_box_->insertItem(i, QString(std::to_string(available_resolution_[i]).c_str()));
    }

    // setup vtk
    pcl_viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->pcl_window_->SetRenderWindow(pcl_viewer_->getRenderWindow());
    pcl_viewer_->setupInteractor(ui->pcl_window_->GetInteractor(), ui->pcl_window_->GetRenderWindow());

    ui->cur_map_info_->setText("就绪");
    pcl_viewer_->addPointCloud<ViewPointType>(cloud_display_, color_handler_, "cloud");
    pcl_viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    pcl_viewer_->addPointCloud<ViewPointType>(current_kf_display_, color_handler_current_, "current");
    pcl_viewer_->addPointCloud<ViewPointType>(current_kf_display_, color_handler_current_, "loop_kf_first");
    pcl_viewer_->addPointCloud<ViewPointType>(current_kf_display_, color_handler_current_, "loop_kf_second");

    pcl_viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "current");
    pcl_viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "loop_kf_first");
    pcl_viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "loop_kf_second");

    pcl_viewer_->resetCamera();
    ui->pcl_window_->update();

    connect(play_timer_.get(), SIGNAL(timeout()), this, SLOT(playing()));
}

DebugUIMainWindow::~DebugUIMainWindow() { delete ui; }

void DebugUIMainWindow::on_focus_cur_kf_btn__clicked() {
    int selected_id = ui->current_kf_id_box_->value();
    LOG(INFO) << "focus on kf " << selected_id;
    update_current_cloud(selected_id);
}

void DebugUIMainWindow::on_load_map_btn__clicked() {
    // 读取地图
    if (db_path_selected_ == false) {
        QMessageBox::warning(this, "Warning", "请选择db文件");
        return;
    }

    if (kf_path_selected_ == false) {
        QMessageBox::warning(this, "Warning", "请选择关键帧文件");
        return;
    }

    if (cloud_display_->points.empty() == false) {
        if (QMessageBox::question(this, "请确认", "已有地图数据，需要载入新地图吗？") == QMessageBox::Yes) {
            clean();
        } else {
            return;
        }
    }

    ui->cur_map_info_->setText("读取地图中...");
    disable_all_input();
    QApplication::processEvents();

    auto prepare_map = QtConcurrent::run(this, &DebugUIMainWindow::prepare_point_cloud);
    prepare_map.waitForFinished();
    enable_all_input();
    ui->cur_map_info_->setText("就绪");

    // set keyframe information
    int num_kf = raw_keyframe_map_.size() - 1;
    ui->current_kf_id_box_->setMinimum(0);
    ui->current_kf_id_box_->setMaximum(num_kf);
    ui->loop_kf_1_->setMinimum(0);
    ui->loop_kf_1_->setMaximum(num_kf);
    ui->loop_kf_2_->setMinimum(0);
    ui->loop_kf_2_->setMaximum(num_kf);
    ui->cur_kf_info_->setText(std::string("Keyframes: " + std::to_string(raw_keyframe_map_.size())).c_str());

    update_current_cloud(0);
}

void DebugUIMainWindow::update_current_cloud(int selected_index) {
    if (raw_keyframe_map_.empty() || selected_index < 0 || selected_index >= raw_keyframe_map_.size()) {
        LOG(INFO) << "current kf id invalid";
        return;
    }

    if (ui->highlight_current_->isChecked() == false) {
        return;
    }

    db_io_->ReadSingleKF(selected_index, current_keyframe_);
    auto cloud_to_display = current_keyframe_->cloud_;

    // transform and display it
    common::PointCloudType::Ptr tmp_t(new common::PointCloudType);
    SE3 pose = raw_keyframe_map_.at(selected_index)->optimized_pose_stage_2_;

    pcl::transformPointCloud(*cloud_to_display, *tmp_t, pose.matrix());
    current_kf_display_ = convert_to_view_cloud(tmp_t);
    color_handler_current_ =
        pcl::visualization::PointCloudColorHandlerCustom<ViewPointType>(current_kf_display_, 255.0, 255.0, 255.0);
    pcl_viewer_->updatePointCloud<ViewPointType>(current_kf_display_, color_handler_current_, "current");

    // update pose
    auto pose_rpy = common::SE3ToRollPitchYaw(pose);

    ui->cur_kf_x_display_->setValue(pose_rpy.x);
    ui->cur_kf_y_display_->setValue(pose_rpy.y);
    ui->cur_kf_z_display_->setValue(pose_rpy.z);
    ui->cur_kf_roll_display_->setValue(pose_rpy.roll * 180 / M_PI);
    ui->cur_kf_pitch_display_->setValue(pose_rpy.pitch * 180 / M_PI);
    ui->cur_kf_yaw_display_->setValue(pose_rpy.yaw * 180 / M_PI);

    if (ui->camera_follow_current_->isChecked()) {
        double height = ui->camera_height_->value();
        pcl_viewer_->setCameraPosition(pose_rpy.x, pose_rpy.y, pose_rpy.z + height, pose_rpy.x, pose_rpy.y, pose_rpy.z,
                                       0, 1, 0);
    }

    ui->pcl_window_->update();

    auto tc = raw_keyframe_map_.at(selected_index)->optimized_pose_stage_2_.translation().head<2>();
    auto tl = raw_keyframe_map_.at(last_selected_index_)->optimized_pose_stage_2_.translation().head<2>();

    if ((tc - tl).norm() > nearby_keyframe_distance_th_) {
        rebuild_display_cloud();
    }
}

void DebugUIMainWindow::update_loop_first(int first_index) {
    if (raw_keyframe_map_.empty() || first_index < 0 || first_index >= raw_keyframe_map_.size()) {
        return;
    }
    if (!ui->highlight_loop_->isChecked()) {
        return;
    }

    std::shared_ptr<common::KeyFrame> kf(new common::KeyFrame);
    db_io_->ReadSingleKF(first_index, kf);
    auto cloud_to_display = kf->cloud_;

    // transform and display it
    common::PointCloudType::Ptr tmp_t(new common::PointCloudType);
    auto pose = raw_keyframe_map_.at(first_index)->optimized_pose_stage_2_;

    pcl::transformPointCloud(*cloud_to_display, *tmp_t, pose.matrix());
    loop_kf_first_ = convert_to_view_cloud(tmp_t);
    color_handler_loop_first_ =
        pcl::visualization::PointCloudColorHandlerCustom<ViewPointType>(loop_kf_first_, 58.0, 74.0, 255.0);
    pcl_viewer_->updatePointCloud<ViewPointType>(loop_kf_first_, color_handler_loop_first_, "loop_kf_first");
    ui->pcl_window_->update();
}

void DebugUIMainWindow::clean() {
    raw_keyframe_map_.clear();
    cloud_display_->clear();
    current_kf_display_->clear();
    loop_kf_first_->clear();
    loop_kf_second_->clear();

    db_io_ = nullptr;
    pcl_viewer_->removeAllShapes();

    update_point_cloud();
}

void DebugUIMainWindow::update_loop_second(int second_index) {
    if (raw_keyframe_map_.empty() || second_index < 0 || second_index >= raw_keyframe_map_.size()) {
        return;
    }
    if (!ui->highlight_loop_->isChecked()) {
        return;
    }

    std::shared_ptr<common::KeyFrame> kf(new common::KeyFrame);
    db_io_->ReadSingleKF(second_index, kf);
    auto cloud_to_display = kf->cloud_;

    // transform and display it
    common::PointCloudType::Ptr tmp_t(new common::PointCloudType);
    auto pose = raw_keyframe_map_.at(second_index)->optimized_pose_stage_2_;
    pcl::transformPointCloud(*cloud_to_display, *tmp_t, pose.matrix());

    loop_kf_second_ = convert_to_view_cloud(tmp_t);
    color_handler_loop_second_ =
        pcl::visualization::PointCloudColorHandlerCustom<ViewPointType>(loop_kf_second_, 125.0, 232.0, 171.0);
    pcl_viewer_->updatePointCloud<ViewPointType>(loop_kf_second_, color_handler_loop_second_, "loop_kf_second");
    ui->pcl_window_->update();
}

void DebugUIMainWindow::on_add_loop_btn__clicked() {
    // 检测loop candidates之间的回环
    if (raw_keyframe_map_.empty()) {
        return;
    }

    SE3 pose = raw_keyframe_map_.at(ui->loop_kf_2_->value())->optimized_pose_stage_2_;

    /// TODO 改为直接设置约束的方式？
    LOG(INFO) << "Testing loop between " << ui->loop_kf_1_->value() << " and " << ui->loop_kf_2_->value();
    // auto res = gui_matcher_->Matching(ui->loop_kf_1_->value(), ui->loop_kf_2_->value(), pose);

    // std::string info = "Detected loops: " + std::to_string(res.size());
    // QMessageBox::information(this, "Info", QString(info.c_str()));

    // num_loops_marked_ += res.size();
    // ui->num_loops_->setText(QString(std::to_string(num_loops_marked_).c_str()));

    // if (res.size() > 0) {
    //     rebuild_optimization_ = true;
    // }
}

void DebugUIMainWindow::on_call_optimize_btn__clicked() {
    if (raw_keyframe_map_.empty()) {
        QMessageBox::warning(this, "Warning", "地图未读取");
        return;
    }

    ui->cur_map_info_->setText("优化中...");
    disable_all_input();
    QApplication::processEvents();

    auto optimization = QtConcurrent::run(this, &DebugUIMainWindow::call_optimization);
    optimization.waitForFinished();

    enable_all_input();
    ui->opti_report_->setText(opti_report_.c_str());
    ui->cur_map_info_->setText("重新生成地图中");
    QApplication::processEvents();

    // 因为关键帧pose发生了改变，需要重新生成地图
    rebuild_display_cloud();

    // 重置显示部分
    pcl_viewer_->removeAllShapes();
    plot_extra_info();
    ui->cur_map_info_->setText("就绪");
}

void DebugUIMainWindow::rebuild_display_cloud() {
    if (cloud_display_) cloud_display_->clear();
    if (raw_keyframe_map_.empty()) return;

    int current_index = ui->current_kf_id_box_->value();
    auto tc = raw_keyframe_map_.at(current_index)->optimized_pose_stage_2_.translation().head<2>();

    int num_picked = 0;

    double res = available_resolution_[resolution_index_];
    pcl::VoxelGrid<ViewPointType> v;
    v.setLeafSize(res, res, res);

    ViewCloudType::Ptr local_cloud(new ViewCloudType);
    ViewCloudType::Ptr local_cloud_filtered(new ViewCloudType);
    for (auto &kf : raw_keyframe_map_) {
        auto t = kf.second->optimized_pose_stage_2_.translation().head<2>();
        if ((tc - t).norm() < nearby_keyframe_distance_th_) {
            db_io_->ReadSingleKF(kf.first, kf.second);
            auto cloud = kf.second->cloud_;

            common::PointCloudType::Ptr cloud_trans(new common::PointCloudType);
            pcl::transformPointCloud(*cloud, *cloud_trans, kf.second->optimized_pose_stage_2_.matrix());

            auto c = convert_to_view_cloud(cloud_trans);
            ViewCloudType ::Ptr cloud_filtered(new ViewCloudType);
            v.setInputCloud(c);
            v.filter(*cloud_filtered);
            *local_cloud += *cloud_filtered;

            v.setInputCloud(local_cloud);
            v.filter(*local_cloud_filtered);
            local_cloud.swap(local_cloud_filtered);

            num_picked++;
        }
    }

    v.setInputCloud(local_cloud);
    v.filter(*cloud_display_);

    LOG(INFO) << "local orig size: " << local_cloud->points.size() << ", filtered: " << cloud_display_->points.size();
    last_selected_index_ = current_index;
    LOG(INFO) << "picked local cloud: " << num_picked << " points: " << cloud_display_->points.size();

    update_point_cloud();
    LOG(INFO) << "rebuild ok";
}

void DebugUIMainWindow::on_reset_optimize_btn__clicked() { QMessageBox::information(this, "消息", "优化已重置"); }

void DebugUIMainWindow::on_save_map_btn__clicked() {
    LOG(INFO) << "Saving map";
    ui->cur_map_info_->setText("存储地图中...");

    disable_all_input();
    QApplication::processEvents();

    auto save = QtConcurrent::run(this, &DebugUIMainWindow::save_map);
    save.waitForFinished();

    QMessageBox::information(this, "消息", "存储完毕");
    enable_all_input();
    ui->cur_map_info_->setText("就绪");
}

void DebugUIMainWindow::save_map() {
    // io::YAML_IO yaml_io("./config/mapping.yaml");
    // tools::SaveFile save_file(yaml_io);

    // update pose in db
    // std::map<int, DBPose> db_poses;
    // for (auto &kf_pair : *raw_keyframe_map_) {
    //     DBPose dbpose;
    //     Eigen::Quaternionf q(kf_pair.second.optimized_pose.block<3, 3>(0, 0));
    //     common::V3f t = kf_pair.second.optimized_pose.block<3, 1>(0, 3);
    //     dbpose.pos.x = t.x();
    //     dbpose.pos.y = t.y();
    //     dbpose.pos.z = t.z();
    //     dbpose.quat.x = q.x();
    //     dbpose.quat.y = q.y();
    //     dbpose.quat.z = q.z();
    //     dbpose.quat.w = q.w();
    //     db_poses.insert({kf_pair.first, dbpose});
    // }

    // db_io_->UpdatePoseByID(db_poses);
    // save_file.KeyFramesPoses(raw_keyframe_map_, ui->keyframe_path_->text().toUtf8().toStdString(), false);
}

void DebugUIMainWindow::on_reset_map_btn__clicked() {
    clean();
    QMessageBox::information(this, "消息", "地图已重置");
}

void DebugUIMainWindow::prepare_point_cloud() {
    // keyframe pose
    raw_keyframe_map_.clear();

    bool kf_read_success = io::LoadKeyframes(ui->keyframe_path_->text().toUtf8().toStdString(), raw_keyframe_map_);
    if (!kf_read_success) {
        LOG(ERROR) << "Read keyframe failed.";
        return;
    }

    LOG(INFO) << "raw keyframes: " << raw_keyframe_map_.size();

    std::string db_path = ui->db_path_->text().toUtf8().toStdString();
    if (db_io_ == nullptr) {
        db_io_ = std::make_shared<mapping::io::DB_IO>(db_path);
    }

    local_data_path_ = db_path.substr(0, db_path.rfind('/'));
    LOG(INFO) << "Local data path set to: " << local_data_path_;

    rebuild_display_cloud();
    LOG(INFO) << "done";

    pcl_viewer_->resetCamera();
    plot_extra_info();
}

void DebugUIMainWindow::plot_extra_info() {
    /// TODO PCL的addText3D会导致segment fault
    int cnt = 0;
    for (auto iter = raw_keyframe_map_.begin(); iter != raw_keyframe_map_.end(); ++iter) {
        auto iter_next = iter;
        iter_next++;
        if (iter_next == raw_keyframe_map_.end()) break;

        auto ti = iter->second->optimized_pose_stage_2_.translation();
        auto tj = iter_next->second->optimized_pose_stage_2_.translation();
        pcl::PointXYZ p1, p2;
        p1.x = ti[0];
        p1.y = ti[1];
        p1.z = ti[2];
        p2.x = tj[0];
        p2.y = tj[1];
        p2.z = tj[2];

        pcl_viewer_->addLine<pcl::PointXYZ>(p1, p2, 0.0, 0.8, 0.0, "keyframe line " + std::to_string(cnt));
        pcl_viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5,
                                                 "keyframe line " + std::to_string(cnt));
        cnt++;
    }

    ui->pcl_window_->update();
}

void DebugUIMainWindow::plot_pose_graph() { ui->pcl_window_->update(); }

ViewCloudType::Ptr DebugUIMainWindow::convert_to_view_cloud(mapping::common::PointCloudType::Ptr in) {
    ViewCloudType::Ptr ret(new ViewCloudType);
    ret->points.resize(in->points.size());
    for (size_t i = 0; i < in->points.size(); ++i) {
        ViewPointType p;
        p.x = in->points[i].x;
        p.y = in->points[i].y;
        p.z = in->points[i].z;
        p.intensity = in->points[i].intensity;
        ret->points[i] = p;
    }
    ret->is_dense = in->is_dense;
    ret->width = in->width;
    ret->height = in->height;
    ret->header = in->header;
    return ret;
}

void DebugUIMainWindow::update_point_cloud() {
    color_handler_ = pcl::visualization::PointCloudColorHandlerGenericField<ViewPointType>(cloud_display_, "intensity");
    pcl_viewer_->updatePointCloud<ViewPointType>(cloud_display_, color_handler_, "cloud");
    ui->pcl_window_->update();
}

void DebugUIMainWindow::disable_all_input() {
    ui->add_loop_btn_->setEnabled(false);
    ui->call_optimize_btn_->setEnabled(false);
    ui->db_path_btn_->setEnabled(false);
    ui->kf_path_btn_->setEnabled(false);
    ui->focus_cur_kf_btn_->setEnabled(false);
    ui->load_map_btn_->setEnabled(false);
    ui->reset_map_btn_->setEnabled(false);
    ui->reset_optimize_btn_->setEnabled(false);
    ui->save_map_btn_->setEnabled(false);
    ui->play_through_btn_->setEnabled(false);
    ui->resolution_box_->setEnabled(false);
    ui->fix_current_btn_->setEnabled(false);
    ui->clear_fixed_btn_1->setEnabled(false);
    ui->call_loop_closing_->setEnabled(false);
}

void DebugUIMainWindow::enable_all_input() {
    ui->add_loop_btn_->setEnabled(true);
    ui->call_optimize_btn_->setEnabled(true);
    ui->db_path_btn_->setEnabled(true);
    ui->kf_path_btn_->setEnabled(true);
    ui->focus_cur_kf_btn_->setEnabled(true);
    ui->load_map_btn_->setEnabled(true);
    ui->reset_map_btn_->setEnabled(true);
    ui->reset_optimize_btn_->setEnabled(true);
    ui->save_map_btn_->setEnabled(true);
    ui->play_through_btn_->setEnabled(true);
    ui->resolution_box_->setEnabled(true);
    ui->fix_current_btn_->setEnabled(true);
    ui->clear_fixed_btn_1->setEnabled(true);
    ui->call_loop_closing_->setEnabled(true);
}

void DebugUIMainWindow::on_db_path_btn__clicked() {
    // 选一个db文件
    QFileDialog fd(this, "Select file", "", "地图数据库(*.db);;");
    if (fd.exec() == QDialog::Accepted) {
        auto filelist = fd.selectedFiles();  //返回文件列表的名称
        ui->db_path_->setText(filelist[0]);
        db_path_selected_ = true;
    } else {
        db_path_selected_ = false;
        fd.close();
    }
}

void DebugUIMainWindow::on_kf_path_btn__clicked() {
    // 选择keyframe.txt 位置
    QFileDialog fd(this, "Select file", "", "关键帧文件(keyframes.txt);;");
    if (fd.exec() == QDialog::Accepted) {
        auto filelist = fd.selectedFiles();  //返回文件列表的名称
        ui->keyframe_path_->setText(filelist[0]);
        kf_path_selected_ = true;
    } else {
        kf_path_selected_ = false;
        fd.close();
    }
}

void DebugUIMainWindow::on_current_kf_id_box__valueChanged(int arg1) { update_current_cloud(arg1); }

void DebugUIMainWindow::on_play_through_btn__clicked() {
    if (raw_keyframe_map_.empty()) {
        return;
    }

    if (playing_) {
        // stop playing
        playing_ = false;
        ui->play_through_btn_->setText("Play!");
        play_timer_->stop();
    } else {
        // play the keyframes
        playing_ = true;
        ui->play_through_btn_->setText("Stop!");
        play_timer_->start(80);
    }
}

void DebugUIMainWindow::playing() {
    int v = ui->current_kf_id_box_->value();
    LOG(INFO) << "playing on " << v;
    v += ui->play_speed_->value();

    if (v < raw_keyframe_map_.size() - 1) {
        ui->current_kf_id_box_->setValue(v);
        update_current_cloud(v);
    } else {
        playing_ = false;
        v = raw_keyframe_map_.size() - 1;

        update_current_cloud(v);
        ui->current_kf_id_box_->setValue(v);
        ui->play_through_btn_->setText("Play!");
        play_timer_->stop();
    }
}

bool DebugUIMainWindow::call_optimization() {
    /// TODO：(1) 增加用户指定的Loop Closing (2) 画出inlier pose graph
    // auto yaml_file = io::YAML_IO("./config/mapping.yaml");
    // OptimizationParams optimization_params;
    // optimization_params.LoadFromYAML(yaml_file);

    // optimization_params.use_rk_dr = false;
    // optimization_params.use_rk_matching = false;

    // if (rebuild_optimization_) {
    //     optimization_params.lidar_continous_num = 1;
    //     optimization_params.dr_continous_num = 2;
    // }

    // optimizer_.reset(new Optimizer(optimization_params));
    // optimizer_->SetLayerConnections(layers_, connections_);
    // optimizer_->SetFixedKeyframes(fixed_keyframes_);

    // std::string report;
    // if (!optimizer_->PoseOptimization(raw_keyframe_map_, report, true)) {
    //     LOG(ERROR) << "optimization failed.";
    //     return false;
    // }

    // if (rebuild_optimization_) {
    //     // perform another optimization
    //     report.clear();
    //     optimization_params.lidar_continous_num = 5;
    //     optimization_params.dr_continous_num = 2;
    //     optimization_params.with_height = true;
    //     optimizer_.reset(new Optimizer(optimization_params));
    //     optimizer_->SetLayerConnections(layers_, connections_);
    //     optimizer_->SetFixedKeyframes(fixed_keyframes_);
    //     optimizer_->PoseOptimization(raw_keyframe_map_, report, true);
    // }

    // LOG(INFO) << "finish optimization";
    // rebuild_optimization_ = false;

    // inlier_pose_graph_ = optimizer_->GetInlierPoseGraph();
    // opti_report_ = report;
    // LOG(INFO) << "Pose graph size: " << inlier_pose_graph_.size();
    return true;
}

void DebugUIMainWindow::on_highlight_current__clicked() {
    if (ui->highlight_current_->isChecked()) {
        // show current
        ui->focus_cur_kf_btn_->setEnabled(true);
        ui->play_through_btn_->setEnabled(true);
        update_current_cloud(ui->current_kf_id_box_->value());
    } else {
        // hide current
        current_kf_display_.reset(new ViewCloudType);
        pcl_viewer_->updatePointCloud<ViewPointType>(current_kf_display_, "current");
        ui->pcl_window_->update();
        ui->focus_cur_kf_btn_->setEnabled(false);
        ui->play_through_btn_->setEnabled(false);
    }
}

void DebugUIMainWindow::on_show_point_cloud__clicked() {}

void DebugUIMainWindow::on_cur_kf_x_display__valueChanged(double arg1) {
    if (ui->highlight_current_->isChecked() == false) {
        return;
    }
    int selected_index = ui->current_kf_id_box_->value();
    auto iter = raw_keyframe_map_.find(selected_index);
    if (iter == raw_keyframe_map_.end()) {
        return;
    }

    auto &cur_kf = iter->second;
    double old_arg = cur_kf->optimized_pose_stage_2_.translation()[0];
    cur_kf->optimized_pose_stage_2_.translation()[0] = arg1;
    if (fabs(old_arg - arg1) > 1e-2) {
        update_current_cloud(selected_index);
    }
}

void DebugUIMainWindow::on_cur_kf_y_display__valueChanged(double arg1) {
    if (ui->highlight_current_->isChecked() == false) {
        return;
    }
    int selected_index = ui->current_kf_id_box_->value();
    auto iter = raw_keyframe_map_.find(selected_index);
    if (iter == raw_keyframe_map_.end()) {
        return;
    }

    auto &cur_kf = iter->second;
    double old_arg = cur_kf->optimized_pose_stage_2_.translation()[1];
    cur_kf->optimized_pose_stage_2_.translation()[1] = arg1;
    if (fabs(old_arg - arg1) > 1e-2) {
        update_current_cloud(selected_index);
    }
}

void DebugUIMainWindow::on_cur_kf_z_display__valueChanged(double arg1) {
    if (ui->highlight_current_->isChecked() == false) {
        return;
    }

    int selected_index = ui->current_kf_id_box_->value();
    auto iter = raw_keyframe_map_.find(selected_index);
    if (iter == raw_keyframe_map_.end()) {
        return;
    }

    auto &cur_kf = iter->second;
    double old_arg = cur_kf->optimized_pose_stage_2_.translation()[2];
    cur_kf->optimized_pose_stage_2_.translation()[2] = arg1;

    if (fabs(old_arg - arg1) > 1e-2) {
        update_current_cloud(selected_index);
    }
}

void DebugUIMainWindow::on_cur_kf_roll_display__valueChanged(double arg1) {
    if (ui->highlight_current_->isChecked() == false) {
        return;
    }

    int selected_index = ui->current_kf_id_box_->value();
    auto iter = raw_keyframe_map_.find(selected_index);
    if (iter == raw_keyframe_map_.end()) {
        return;
    }
    arg1 *= M_PI / 180;

    auto &cur_kf = iter->second;

    auto rpy = common::SE3ToRollPitchYaw(cur_kf->optimized_pose_stage_2_);
    double old_arg = rpy.roll;
    rpy.roll = arg1;
    cur_kf->optimized_pose_stage_2_ = common::XYZRPYToSE3(rpy);

    if (fabs(old_arg - arg1) > 1e-3) {
        update_current_cloud(selected_index);
    }
}

void DebugUIMainWindow::on_cur_kf_pitch_display__valueChanged(double arg1) {
    if (ui->highlight_current_->isChecked() == false) {
        return;
    }

    int selected_index = ui->current_kf_id_box_->value();
    auto iter = raw_keyframe_map_.find(selected_index);
    if (iter == raw_keyframe_map_.end()) {
        return;
    }

    arg1 *= M_PI / 180;

    auto &cur_kf = iter->second;
    auto rpy = common::SE3ToRollPitchYaw(cur_kf->optimized_pose_stage_2_);
    double old_arg = rpy.pitch;
    rpy.pitch = arg1;
    cur_kf->optimized_pose_stage_2_ = common::XYZRPYToSE3(rpy);

    if (fabs(old_arg - arg1) > 1e-3) {
        update_current_cloud(selected_index);
    }
}

void DebugUIMainWindow::on_cur_kf_yaw_display__valueChanged(double arg1) {
    if (ui->highlight_current_->isChecked() == false) {
        return;
    }

    int selected_index = ui->current_kf_id_box_->value();
    auto iter = raw_keyframe_map_.find(selected_index);
    if (iter == raw_keyframe_map_.end()) {
        return;
    }

    arg1 *= M_PI / 180;

    auto &cur_kf = iter->second;
    auto rpy = common::SE3ToRollPitchYaw(cur_kf->optimized_pose_stage_2_);
    double old_arg = rpy.yaw;
    rpy.yaw = arg1;
    cur_kf->optimized_pose_stage_2_ = common::XYZRPYToSE3(rpy);

    if (fabs(old_arg - arg1) > 1e-3) {
        update_current_cloud(selected_index);
    }
}

void DebugUIMainWindow::on_loop_kf_1__valueChanged(int arg1) { update_loop_first(arg1); }

void DebugUIMainWindow::on_loop_kf_2__valueChanged(int arg1) { update_loop_second(arg1); }

void DebugUIMainWindow::on_highlight_loop__clicked() {
    if (ui->highlight_loop_->isChecked()) {
        // show loop candidates
        update_loop_first(ui->loop_kf_1_->value());
        update_loop_second(ui->loop_kf_2_->value());
    } else {
        // hide loop candidates
        loop_kf_first_.reset(new ViewCloudType);
        loop_kf_second_.reset(new ViewCloudType);
        pcl_viewer_->updatePointCloud<ViewPointType>(loop_kf_first_, "loop_kf_first");
        pcl_viewer_->updatePointCloud<ViewPointType>(loop_kf_second_, "loop_kf_second");
    }

    ui->pcl_window_->update();
}

void DebugUIMainWindow::on_fix_current_btn__clicked() {
    int id = ui->loop_kf_1_->value();
    LOG(INFO) << "FIX keyframe " << id << " pose to "
              << raw_keyframe_map_.at(id)->optimized_pose_stage_2_.translation().transpose();
    int id2 = ui->loop_kf_2_->value();
    LOG(INFO) << "FIX keyframe " << id2 << " pose to "
              << raw_keyframe_map_.at(id2)->optimized_pose_stage_2_.translation().transpose();

    fixed_keyframes_.insert(id);
    fixed_keyframes_.insert(id2);
    rebuild_optimization_ = true;
}

void DebugUIMainWindow::on_call_loop_closing__clicked() {
    // 进行闭环检测
    auto yaml_file = io::YAML_IO("./config/mapping.yaml");
    // std::shared_ptr<LoopClosureMultiMap> loop_closure = std::make_shared<LoopClosureMultiMap>(yaml_file);
    // loop_closure->SetForceDBPath(ui->db_path_->text().toUtf8().toStdString());
    // loop_closure->SetKeyFrames(raw_keyframe_map_);
    // if (!loop_closure->Run()) {
    //     LOG(ERROR) << "loop closure failed.";
    // }

    LOG(INFO) << "finish loop closing.";
    rebuild_optimization_ = true;
}

void DebugUIMainWindow::on_clear_fixed_btn_1_clicked() { fixed_keyframes_.clear(); }

void DebugUIMainWindow::on_resolution_box__currentIndexChanged(int index) {
    resolution_index_ = index;
    rebuild_display_cloud();
}
