#include "quickdebugmainwindow.h"

#include <glog/logging.h>
#include <QCloseEvent>
#include <QMessageBox>
#include <QtConcurrent/QtConcurrent>

#include "common/mapping_math.h"
#include "mapping_interface.h"
#include "ui_quickdebugmainwindow.h"

#define RAD2DEG 57.295779513

QuickDebugMainWindow::QuickDebugMainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::QuickDebugMainWindow), play_timer_(new QTimer(this)) {
    ui->setupUi(this);
    timer_ = std::make_shared<QTimer>(this);
    connect(timer_.get(), SIGNAL(timeout()), this, SLOT(update()));
    timer_->start(60);

    map_interface_ = std::make_shared<mapping::MappingInterface>();

    ui->display_widget_->SetMappingInterface(map_interface_);

    connect(play_timer_.get(), SIGNAL(timeout()), this, SLOT(playing()));

    for (int i = 0; i < available_resolution_.size(); ++i) {
        ui->resolution_box_->insertItem(i, QString(std::to_string(available_resolution_[i]).c_str()));
    }

    ui->statusbar->setSizeGripEnabled(false);

    keyframe_num_label_ = new QLabel("关键帧数量: ", this);
    position_label_ = new QLabel("相机 x= y=", this);

    keyframe_num_label_->setMinimumWidth(200);
    position_label_->setMinimumWidth(400);

    keyframe_num_label_->setAlignment(Qt::AlignCenter);
    position_label_->setAlignment(Qt::AlignCenter);

    ui->statusbar->addPermanentWidget(keyframe_num_label_);
    ui->statusbar->addPermanentWidget(position_label_);

    ui->statusbar->show();

    connect(ui->display_widget_, SIGNAL(ShowCameraPos(double, double)), this, SLOT(ShowCameraPos(double, double)));

    connect(ui->display_widget_, SIGNAL(valueChanged(bool, unsigned long, Eigen::Vector3f)), this,
            SLOT(setValue(bool, unsigned long, Eigen::Vector3f)));

    ui->lcdNumber->setLineWidth(0);

    ui->dockWidgetVertex->hide();
}

QuickDebugMainWindow::~QuickDebugMainWindow() {
    delete position_label_;
    delete keyframe_num_label_;
    delete ui;
}

void QuickDebugMainWindow::update() { ui->display_widget_->update(); }

void QuickDebugMainWindow::ShowCameraPos(double x, double y) {
    QString str;
    str.sprintf("相机 x=%3.2f y=%3.2f", x, y);
    position_label_->setText(str);
}

void QuickDebugMainWindow::ShowKeyframeNum(int keyframe_num) {
    QString str;
    str.sprintf("关键帧数量: %d", keyframe_num);
    keyframe_num_label_->setText(str);
}

void QuickDebugMainWindow::on_action_open_yaml_triggered() {
    if (ui->display_widget_->OnOpenYAML()) {
        int n = map_interface_->GetKeyframeSize();
        ui->current_kf_id_box_->setMinimum(0);
        ui->current_kf_id_box_->setMaximum(n - 1);
        ShowKeyframeNum(n);
        ShowCameraPos(0, 0);
    } else {
        QMessageBox::critical(this, "Warning", "Failed to open the map !");
    }
}

void QuickDebugMainWindow::on_action_save_map_triggered() {
    auto result = QtConcurrent::run(this, &QuickDebugMainWindow::save_map);
    if (result.result() == false) {
        QMessageBox::warning(this, "Warning", "The map has not been updated, no need to save");
    } else {
        QMessageBox::information(this, "Information", "Map updated successfully !");
    }
}

void QuickDebugMainWindow::on_action_close_map_triggered() { map_interface_->CloseMap(); };

void QuickDebugMainWindow::on_action_drop_out_triggered() {
    QMessageBox::StandardButton rb =
        QMessageBox::question(NULL, "Save the map", "Whether to save the map before closing the program ?",
                              QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    if (rb == QMessageBox::Yes) {
        if (map_interface_->SaveMap()) {
            QMessageBox::information(this, "Information", "Database update successfully !");
        } else {
            QMessageBox::information(this, "Information", "Database update failed !");
        }
    }

    qApp->quit();
}

void QuickDebugMainWindow::on_action_export_point_cloud_triggered() {
    auto result = QtConcurrent::run(this, &QuickDebugMainWindow::save_pcd);
    if (result.result() == false) {
        QMessageBox::warning(this, "Warning", "The map has not been updated, no need to save");
    } else {
        QMessageBox::information(this, "Information", "Point cloud update successfully !");
    }
}

void QuickDebugMainWindow::on_focus_cur_kf_btn__clicked() {
    if (!ui->highlight_current_->isChecked()) {
        return;
    }

    int selected_id = ui->current_kf_id_box_->value();
    if (selected_id >= map_interface_->GetKeyframeSize() || selected_id < 0) {
        return;
    }

    LOG(INFO) << "focus on kf " << selected_id;

    SE3 coordinate;
    if (!map_interface_->UpdateCurrentCloud(selected_id, coordinate)) {
        QMessageBox::warning(this, "Warning", "请输入有效的关键帧ID");
    } else {
        update_coordinates(coordinate);
    }
}

void QuickDebugMainWindow::on_cur_kf_x_display__valueChanged(double arg1) {
    if (ui->highlight_current_->isChecked() == false || playing_) {
        return;
    }
    int selected_index = ui->current_kf_id_box_->value();
    map_interface_->FinetuneX(selected_index, arg1);
}

void QuickDebugMainWindow::on_cur_kf_y_display__valueChanged(double arg1) {
    if (ui->highlight_current_->isChecked() == false || playing_) {
        return;
    }
    int selected_index = ui->current_kf_id_box_->value();

    map_interface_->FinetuneY(selected_index, arg1);
}

void QuickDebugMainWindow::on_cur_kf_z_display__valueChanged(double arg1) {
    if (ui->highlight_current_->isChecked() == false || playing_) {
        return;
    }
    int selected_index = ui->current_kf_id_box_->value();

    map_interface_->FinetuneZ(selected_index, arg1);
}

void QuickDebugMainWindow::on_cur_kf_roll_display__valueChanged(double arg1) {
    if (ui->highlight_current_->isChecked() == false || playing_) {
        return;
    }
    int selected_index = ui->current_kf_id_box_->value();

    map_interface_->FinetuneRoll(selected_index, arg1);
}

void QuickDebugMainWindow::on_cur_kf_pitch_display__valueChanged(double arg1) {
    if (ui->highlight_current_->isChecked() == false || playing_) {
        return;
    }
    int selected_index = ui->current_kf_id_box_->value();

    map_interface_->FinetunePitch(selected_index, arg1);
}

void QuickDebugMainWindow::on_cur_kf_yaw_display__valueChanged(double arg1) {
    if (ui->highlight_current_->isChecked() == false || playing_) {
        return;
    }
    int selected_index = ui->current_kf_id_box_->value();

    map_interface_->FinetuneYaw(selected_index, arg1);
}

void QuickDebugMainWindow::on_add_loop_btn__clicked() {
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    // 检测loop candidates之间的回环
    LOG(INFO) << "Testing loop between " << ui->loop_kf_1_->value() << " and " << ui->loop_kf_2_->value();

    dialog_ = new Dialog();
    dialog_->exec();
    Dialog::LoopCloseParams params;
    dialog_->GetLoopCloseParams(params);

    add_loop_ = QtConcurrent::run(this, &QuickDebugMainWindow::call_add_loop, params);
    add_loop_.waitForFinished();

    std::string info = "Detected loops: " + std::to_string(add_loop_.result());

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    double time_usage = time_used.count();

    LOG(INFO) << "time_usage : " << time_usage;

    QMessageBox::information(this, "Info", QString(info.c_str()));
}

void QuickDebugMainWindow::on_loop_kf_1__valueChanged(int arg1) {
    if (ui->highlight_loop_->isChecked() == false) {
        return;
    }

    map_interface_->UpdateLoopFirst(arg1);
}

void QuickDebugMainWindow::on_loop_kf_2__valueChanged(int arg1) {
    if (ui->highlight_loop_->isChecked() == false) {
        return;
    }

    map_interface_->UpdateLoopSecond(arg1);
}

void QuickDebugMainWindow::on_play_through_btn__clicked() {
    if (map_interface_->GetKeyframeSize() == 0) {
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

void QuickDebugMainWindow::playing() {
    int v = ui->current_kf_id_box_->value();
    v += ui->play_speed_->value();
    SE3 coordinate;

    if (v < map_interface_->GetKeyframeSize() - 1) {
        v = map_interface_->GetSuggestKfIdx(v);
        ui->current_kf_id_box_->setValue(v);

        if (!map_interface_->UpdateCurrentCloud(v, coordinate)) {
            QMessageBox::warning(this, "Warning", "请输入有效的关键帧ID");
        } else {
            update_coordinates(coordinate);
        }
    } else {
        playing_ = false;
        v = map_interface_->GetKeyframeSize() - 1;

        if (!map_interface_->UpdateCurrentCloud(v, coordinate)) {
            QMessageBox::warning(this, "Warning", "请输入有效的关键帧ID");
        } else {
            update_coordinates(coordinate);
        }

        ui->current_kf_id_box_->setValue(v);
        ui->play_through_btn_->setText("Play!");
        play_timer_->stop();
    }
}

void QuickDebugMainWindow::update_coordinates(const SE3& coordinate) {
    ui->cur_kf_x_display_->setValue(coordinate.translation()[0]);
    ui->cur_kf_y_display_->setValue(coordinate.translation()[1]);
    ui->cur_kf_z_display_->setValue(coordinate.translation()[2]);

    auto rpy = mapping::common::SE3ToRollPitchYaw(coordinate);

    ui->cur_kf_roll_display_->setValue(rpy.roll * RAD2DEG);
    ui->cur_kf_pitch_display_->setValue(rpy.pitch * RAD2DEG);
    ui->cur_kf_yaw_display_->setValue(rpy.yaw * RAD2DEG);
}

void QuickDebugMainWindow::on_call_loop_closing__clicked() {
    // 进行闭环检测
    if (map_interface_->LoopClosing()) {
        LOG(ERROR) << "loop closure failed.";
    }
    LOG(INFO) << "finish loop closing.";
    rebuild_optimization_ = true;
}

void QuickDebugMainWindow::on_fix_current_btn__clicked() {
    int id1 = ui->loop_kf_1_->value();

    map_interface_->FixCurrent(id1);

    int id2 = ui->loop_kf_2_->value();

    map_interface_->FixCurrent(id2);

    fixed_keyframes_.emplace_back(id1);
    fixed_keyframes_.emplace_back(id2);

    rebuild_optimization_ = true;

    QMessageBox::information(this, "Info",
                             QString::fromStdString(std::to_string(id1) + "和" + std::to_string(id2) + "已固定"));
}

void QuickDebugMainWindow::on_clear_fixed_btn_1_clicked() {
    fixed_keyframes_.clear();
    QMessageBox::information(this, "Info", "固定帧已清除");
}

void QuickDebugMainWindow::on_call_optimize_btn__clicked() {
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    if (map_interface_->GetKeyframeSize() == 0) {
        QMessageBox::warning(this, "Warning", "地图未读取");
        return;
    }

    disable_all_input();

    QApplication::processEvents();

    optimization_dialog_ = new OptimizationDialog();
    optimization_dialog_->exec();
    OptimizationDialog::OptimizationParams params;
    optimization_dialog_->GetOptimizationaParams(params);

    auto optimization = QtConcurrent::run(this, &QuickDebugMainWindow::call_optimization, params);
    optimization.waitForFinished();

    enable_all_input();

    ui->opti_report_->setText(optimization_report_.c_str());

    LOG(INFO) << "finish optimization";

    rebuild_optimization_ = false;

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    double time_usage = time_used.count();

    LOG(INFO) << "time_usage : " << time_usage;
}

void QuickDebugMainWindow::disable_all_input() {
    ui->add_loop_btn_->setEnabled(false);
    ui->call_optimize_btn_->setEnabled(false);
    ui->focus_cur_kf_btn_->setEnabled(false);
    ui->reset_optimize_btn_->setEnabled(false);
    ui->play_through_btn_->setEnabled(false);
    ui->resolution_box_->setEnabled(false);
    ui->fix_current_btn_->setEnabled(false);
    ui->clear_fixed_btn_1->setEnabled(false);
    ui->call_loop_closing_->setEnabled(false);
}

void QuickDebugMainWindow::enable_all_input() {
    ui->add_loop_btn_->setEnabled(true);
    ui->call_optimize_btn_->setEnabled(true);
    ui->focus_cur_kf_btn_->setEnabled(true);
    ui->reset_optimize_btn_->setEnabled(true);
    ui->play_through_btn_->setEnabled(true);
    ui->resolution_box_->setEnabled(true);
    ui->fix_current_btn_->setEnabled(true);
    ui->clear_fixed_btn_1->setEnabled(true);
    ui->call_loop_closing_->setEnabled(true);
}

bool QuickDebugMainWindow::call_optimization(const OptimizationDialog::OptimizationParams& params) {
    return map_interface_->CallOptimization(fixed_keyframes_, optimization_report_, params);
}

void QuickDebugMainWindow::on_reset_optimize_btn__clicked() { QMessageBox::information(this, "消息", "优化已重置"); }

void QuickDebugMainWindow::on_show_point_cloud__stateChanged(int state) {
    map_interface_->SetShowPointCloud(ui->show_point_cloud_->isChecked());
}

void QuickDebugMainWindow::on_resolution_box__currentIndexChanged(int index) {
    map_interface_->SetPointCloudResolution(available_resolution_[index]);
}

void QuickDebugMainWindow::on_show_pose_graph_check_1_stateChanged(int arg1) {
    map_interface_->SetShowTrack(ui->show_pose_graph_check_1->isChecked());
}

int QuickDebugMainWindow::call_add_loop(Dialog::LoopCloseParams& params) {
    int res = map_interface_->AddLoop(ui->loop_kf_1_->value(), ui->loop_kf_2_->value(), params);

    num_loops_marked_ += res;
    ui->num_loops_->setText(QString(std::to_string(num_loops_marked_).c_str()));

    if (res > 0) {
        rebuild_optimization_ = true;
    }

    return res;
}

void QuickDebugMainWindow::closeEvent(QCloseEvent* event) {
    QMessageBox::StandardButton button =
        QMessageBox::question(this, tr("退出程序"), QString(tr("确认退出程序")), QMessageBox::Yes | QMessageBox::No);
    if (QMessageBox::No == button) {
        event->ignore();  // 忽略退出信号，程序继续进行
    } else if (QMessageBox::Yes == button) {
        event->accept();  // 接受退出信号，程序退出
    }
}

bool QuickDebugMainWindow::save_map() { return map_interface_->SaveMap(); }

bool QuickDebugMainWindow::save_pcd() { return map_interface_->SavePcd(); }

void QuickDebugMainWindow::setValue(bool found, unsigned long value, Eigen::Vector3f pos) {
    if (found == false) {
        ui->dockWidgetVertex->hide();
        return;
    }
    if (value != index_) {
        index_ = value;
    }

    ui->dockWidgetVertex->show();
    ui->lcdNumber->display(index_);

    if (std::find(fixed_keyframes_.begin(), fixed_keyframes_.end(), index_) != fixed_keyframes_.end()) {
        ui->checkBoxFixed->setCheckState(Qt::CheckState::Checked);
    }

    ui->current_kf_id_box_->setValue(index_);
}

void QuickDebugMainWindow::on_checkBoxFixed_clicked() {
    if (ui->checkBoxFixed->isChecked()) {
        fixed_keyframes_.emplace_back(index_);
        LOG(INFO) << "Keyframe " << index_ << " has been fixed";
    } else {
        fixed_keyframes_.erase(std::find(fixed_keyframes_.begin(), fixed_keyframes_.end(), index_));
        LOG(INFO) << "Keyframe " << index_ << " has been unfixed";
    }

    rebuild_optimization_ = true;
}

void QuickDebugMainWindow::on_show_pose_graph_check__stateChanged(int arg1) {
    map_interface_->SetShowPoseGraph(ui->show_pose_graph_check_->isChecked());
}

void QuickDebugMainWindow::on_action_reset_camera_triggered() {
    map_interface_->ResetCamera();
    ui->display_widget_->clearBttuonState();
}

void QuickDebugMainWindow::on_pushButtonLoopBegin_clicked() { ui->loop_kf_1_->setValue(index_); }

void QuickDebugMainWindow::on_pushButtonLoopEnd_clicked() { ui->loop_kf_2_->setValue(index_); }

void QuickDebugMainWindow::on_highlight_loop__stateChanged(int arg1) {
    map_interface_->SetShowLoopFrames(ui->highlight_loop_->isChecked());
}

void QuickDebugMainWindow::on_spinBoxTrackId_valueChanged(const QString& arg1) {
    map_interface_->SetTrackId(arg1.toULong());
}

void QuickDebugMainWindow::on_pushButtonSwitch2Track_clicked() { map_interface_->Switch2Track(); }

void QuickDebugMainWindow::on_pushButtonFixCurrentTrajectory_clicked() {
    map_interface_->FixCurrentTrack();
    if (std::find(fixed_track_ids_.begin(), fixed_track_ids_.end(), ui->spinBoxTrackId->value()) ==
        fixed_track_ids_.end()) {
        fixed_track_ids_.emplace_back(ui->spinBoxTrackId->value());
    }
    std::string fixed_tracks_win;
    std::sort(fixed_track_ids_.begin(), fixed_track_ids_.end());
    for (size_t i = 0; i < fixed_track_ids_.size(); ++i) {
        fixed_tracks_win += std::to_string(fixed_track_ids_[i]);
        fixed_tracks_win += ", ";
    }
    ui->textBrowserFixedTracks->setText(fixed_tracks_win.c_str());
}

void QuickDebugMainWindow::on_pushButtonClearFixedTrack_clicked() {
    map_interface_->ClearFixedTracks();
    fixed_track_ids_.clear();
    ui->textBrowserFixedTracks->clear();
}

void QuickDebugMainWindow::on_pushButtonFixClosedLoopFrame_clicked() { map_interface_->FixClosedLoopFrames(); }

void QuickDebugMainWindow::on_pushButtonClearClosedLoopFrame_clicked() { map_interface_->ClearClosedLoopFrames(); }
