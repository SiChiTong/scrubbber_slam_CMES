#ifndef QUICKDEBUGMAINWINDOW_H
#define QUICKDEBUGMAINWINDOW_H

#include <QLabel>
#include <QMainWindow>
#include <QTimer>
#include <QtConcurrent>
#include <memory>

#include "common/num_type.h"
#include "dialog.h"
#include "optimizationdialog.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class QuickDebugMainWindow;
}
QT_END_NAMESPACE

namespace mapping {
class MappingInterface;
}

class QuickDebugMainWindow : public QMainWindow {
    Q_OBJECT

   public:
    explicit QuickDebugMainWindow(QWidget *parent = nullptr);
    ~QuickDebugMainWindow() override;

   public slots:
    // 显示相机位置
    void ShowCameraPos(double x, double y);
    void ShowKeyframeNum(int keyframe_num);

    void on_action_open_yaml_triggered();

    void on_action_save_map_triggered();

    void on_action_close_map_triggered();

    void on_action_drop_out_triggered();

    void on_action_export_point_cloud_triggered();

    void on_action_reset_camera_triggered();

    void setValue(bool found, unsigned long value, Eigen::Vector3f);

   private slots:

    // 刷新中央UI
    void update();

    /// 镜头切到当前帧
    void on_focus_cur_kf_btn__clicked();

    void on_cur_kf_x_display__valueChanged(double);

    void on_cur_kf_y_display__valueChanged(double);

    void on_cur_kf_z_display__valueChanged(double);

    void on_cur_kf_roll_display__valueChanged(double);

    void on_cur_kf_pitch_display__valueChanged(double);

    void on_cur_kf_yaw_display__valueChanged(double);

    // 添加回环的回调
    void on_add_loop_btn__clicked();

    void on_loop_kf_1__valueChanged(int);

    void on_loop_kf_2__valueChanged(int);

    void on_play_through_btn__clicked();

    // 播放的更新函数
    void playing();

    void on_call_loop_closing__clicked();

    void on_fix_current_btn__clicked();

    void on_clear_fixed_btn_1_clicked();

    void on_call_optimize_btn__clicked();

    void on_reset_optimize_btn__clicked();

    void on_resolution_box__currentIndexChanged(int index);

    void on_show_point_cloud__stateChanged(int);

    void on_show_pose_graph_check_1_stateChanged(int arg1);

    void on_checkBoxFixed_clicked();

    void on_show_pose_graph_check__stateChanged(int arg1);

    void on_pushButtonLoopBegin_clicked();

    void on_pushButtonLoopEnd_clicked();

    void on_highlight_loop__stateChanged(int arg1);

    void on_spinBoxTrackId_valueChanged(const QString &arg1);

    void on_pushButtonSwitch2Track_clicked();

    void on_pushButtonFixCurrentTrajectory_clicked();

    void on_pushButtonClearFixedTrack_clicked();

    void on_pushButtonFixClosedLoopFrame_clicked();

    void on_pushButtonClearClosedLoopFrame_clicked();

   private:
    // 更新当前帧信息
    void update_current_cloud(int selected_index);

    void update_coordinates(const SE3 &coordinate);

    // 关掉所有button
    void disable_all_input();

    // 开启所有button
    void enable_all_input();

    // 实际调用优化
    bool call_optimization(const OptimizationDialog::OptimizationParams &params);

    int call_add_loop(Dialog::LoopCloseParams &params);

    void closeEvent(QCloseEvent *event);

    bool save_map();

    bool save_pcd();

   private:
    Ui::QuickDebugMainWindow *ui;
    std::shared_ptr<QTimer> timer_ = nullptr;

    QLabel *keyframe_num_label_ = nullptr;  // 指示关键帧数量
    QLabel *position_label_ = nullptr;      // 指示相机位置

    std::shared_ptr<mapping::MappingInterface> map_interface_ = nullptr;

    int num_loops_marked_ = 0;

    bool playing_ = false;  // 是否正在播放关键帧序列

    std::shared_ptr<QTimer> play_timer_ = nullptr;

    bool rebuild_optimization_ = false;  // 是否需要重跑优化

    std::vector<int> fixed_keyframes_;
    std::vector<int> fixed_track_ids_;

    std::string optimization_report_;  // 优化报告
    std::string directory_;

    std::vector<double> available_resolution_ = {0.08, 0.1, 0.5, 1.0};  // 可选分辨率

    Dialog *dialog_ = nullptr;
    OptimizationDialog *optimization_dialog_ = nullptr;

    QFuture<int> add_loop_;

    int index_ = 0;
};
#endif  // QUICKDEBUGMAINWINDOW_H
