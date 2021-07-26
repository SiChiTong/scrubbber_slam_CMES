#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <memory>
#include <thread>
#include <QFuture>
#include <QCloseEvent>

#include <atomic>
#include <mutex>

#include "pipeline/pipeline_engine.h"
#include "ui_mainwindow.h"
#include "common/origin_point_info.h"

namespace Ui {
class MainWindow;
}

class MappingUIMainWindow : public QMainWindow {
Q_OBJECT

public:
    explicit MappingUIMainWindow(QWidget *parent = 0);

    ~MappingUIMainWindow();

    void SetAccountInfo(const std::string &account_name, const std::string &token) {
        account_name_ = account_name;
        account_token_ = token;

        ui->account_name_edt->setText(account_name_.c_str());
    }

signals:

    void progress(double value);

    void set_status(const QString &str);

private slots:

    void on_start_mapping_clicked();

    void on_toolButton_clicked();

    void update();

    void on_detailed_message_textChanged();

    void on_upload_result_clicked();

    inline void set_process_value(double value) {
        ui->mapping_progress->setValue(int(value));
        ui->mapping_process_number->display(value);
    }

    void show_status(const QString &str) {
        ui->statusBar->showMessage(str, 0);
        ui->statusBar->update();
    }

    void uploadind_status_update();

private:
    bool check_mapping_start_condition();

    /// 启动建图流水线
    bool start_mapping();

    /// 执行建图线程
    void run_mapping();

    /// 收尾工作，上传结果至OSS
    void finish_mapping();

    void closeEvent(QCloseEvent *event);

    /// 设置地图偏移量
    mapping::common::OriginPointInformation
    GetMapOffsetFromConfig(const double &lon, const double &lat, const double &height);

    void reupload_mapping_result();

private:
    Ui::MainWindow *ui;

    QString config_file_name_;  // 配置文件的路径
    QString data_file_name_;    // 实际数据的完整路径
    QString data_path_;         // 实际数据所在目录

    std::shared_ptr<QTimer> timer_ = nullptr;
    std::shared_ptr<std::thread> mapping_thread_ = nullptr;

    std::shared_ptr<QTimer> timer_uploading_ = nullptr;

    QFuture<void> mapping_result_;
    QFuture<void> uploading_result_;

    std::shared_ptr<mapping::pipeline::PipelineEngine> mapping_engine_ = nullptr;
    std::mutex engine_mutex_;
    std::string account_name_;
    std::string account_token_;

    std::atomic<bool> preparing_;
    std::atomic<bool> running_;

    std::atomic<int> uploading_status_;

    double current_process_ = 0;
    std::string current_step_;
    double cnt_seconds_ = 0;
    mapping::pipeline::TaskInfo task_info_;
    std::atomic<int> if_update_date_ready_;
    std::atomic<bool> using_local_data_for_updating_;
};

#endif // MAINWINDOW_H
