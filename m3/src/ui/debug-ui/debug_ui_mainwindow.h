#ifndef DEBUG_UI_MAINWINDOW_H
#define DEBUG_UI_MAINWINDOW_H

#include <QMainWindow>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

#include "common/keyframe.h"
#include "io/db_io.h"
#include "pipeline/optimization_s2/optimization_s2.h"

namespace Ui {
class MainWindow;
}

typedef pcl::PointXYZI ViewPointType;
typedef pcl::PointCloud<ViewPointType> ViewCloudType;

class DebugUIMainWindow : public QMainWindow {
    Q_OBJECT

   public:
    explicit DebugUIMainWindow(QWidget *parent = 0);

    ~DebugUIMainWindow();

   private slots:

    void on_focus_cur_kf_btn__clicked();

    // 读地图信息的回调
    void on_load_map_btn__clicked();

    // 添加回环的回调
    void on_add_loop_btn__clicked();

    void on_call_optimize_btn__clicked();

    void on_reset_optimize_btn__clicked();

    void on_save_map_btn__clicked();

    void on_reset_map_btn__clicked();

    void on_db_path_btn__clicked();

    void on_kf_path_btn__clicked();

    void on_current_kf_id_box__valueChanged(int arg1);

    void on_play_through_btn__clicked();

    void on_highlight_current__clicked();

    void on_show_point_cloud__clicked();

    void on_resolution_box__currentIndexChanged(int index);

    // 播放的更新函数
    void playing();

    void on_cur_kf_x_display__valueChanged(double);

    void on_cur_kf_y_display__valueChanged(double);

    void on_cur_kf_z_display__valueChanged(double);

    void on_cur_kf_roll_display__valueChanged(double);

    void on_cur_kf_pitch_display__valueChanged(double);

    void on_cur_kf_yaw_display__valueChanged(double);

    void on_loop_kf_1__valueChanged(int);

    void on_loop_kf_2__valueChanged(int);

    void on_highlight_loop__clicked();

    void on_fix_current_btn__clicked();

    void on_call_loop_closing__clicked();

    void on_clear_fixed_btn_1_clicked();

   private:
    // 读db，生成多分辨率地图
    void prepare_point_cloud();

    // 关掉所有button
    void disable_all_input();

    // 开启所有button
    void enable_all_input();

    // 把cloud_display更新到当前窗口中
    void update_point_cloud();

    // 更新当前帧信息
    void update_current_cloud(int selected_index);

    // 更新回环帧
    void update_loop_first(int first_index);

    void update_loop_second(int second_index);

    // 将mapping3.0点云转换为显示时点云
    ViewCloudType::Ptr convert_to_view_cloud(mapping::common::PointCloudType::Ptr in);

    // 关键帧Pose改变后，重新生成display cloud
    void rebuild_display_cloud();

    // 重画所有额外信息
    inline void rebuild_extra_info() {
        pcl_viewer_->removeAllShapes();
        plot_extra_info();
        plot_pose_graph();
    }

    // 显示关键帧轨迹/参考平面
    void plot_extra_info();

    // 显示pose graph
    void plot_pose_graph();

    // 实际调用优化
    bool call_optimization();

    // 清空所有数据
    void clean();

    // 存储地图
    void save_map();

   private:
    Ui::MainWindow *ui;

    pcl::visualization::PCLVisualizer::Ptr pcl_viewer_;

    // data
    std::shared_ptr<mapping::io::DB_IO> db_io_ = nullptr;
    std::string local_data_path_;
    std::map<IdType, std::shared_ptr<mapping::common::KeyFrame>> raw_keyframe_map_;  // 原始关键帧轨迹
    std::shared_ptr<mapping::common::KeyFrame> current_keyframe_ = nullptr;

    /// 可选分辨率
    std::vector<double> available_resolution_ = {0.5};
    int resolution_index_ = 0;

    int last_selected_index_ = 0;

    ViewCloudType::Ptr cloud_display_ = nullptr;       // 当前显示的地图
    ViewCloudType::Ptr current_kf_display_ = nullptr;  // 当前关键帧

    // 回环候选
    ViewCloudType::Ptr loop_kf_first_ = nullptr;
    ViewCloudType::Ptr loop_kf_second_ = nullptr;

    int num_loops_marked_ = 0;

    std::set<int> fixed_keyframes_;
    bool rebuild_optimization_ = false;  // 是否需要重跑优化

    std::string opti_report_;  // 优化报告

    // utils
    pcl::visualization::PointCloudColorHandlerGenericField<ViewPointType> color_handler_;    // 全局颜色显示
    pcl::visualization::PointCloudColorHandlerCustom<ViewPointType> color_handler_current_;  // 当前帧颜色显示
    pcl::visualization::PointCloudColorHandlerCustom<ViewPointType> color_handler_loop_first_;
    pcl::visualization::PointCloudColorHandlerCustom<ViewPointType> color_handler_loop_second_;

    bool playing_ = false;  // 是否正在播放关键帧序列
    std::shared_ptr<QTimer> play_timer_ = nullptr;

    bool db_path_selected_ = false;
    bool kf_path_selected_ = false;

    static constexpr float nearby_keyframe_distance_th_ = 20.0;
};

#endif  // DEBUG_UI_MAINWINDOW_H
