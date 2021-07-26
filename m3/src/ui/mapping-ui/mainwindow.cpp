#include "mainwindow.h"
#include <json/json.h>
#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>
#include <QtConcurrent>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

#include <fstream>
#include <iostream>

#include <glog/logging.h>
#include "common/origin_point_info.h"
#include "core/coordinate_transform/gps_trans.h"
#include "io/oss_io.h"
#include "io/yaml_io.h"
#include "pipeline/pipeline_engine.h"

using namespace mapping;
using namespace mapping::pipeline;
using namespace mapping::core;

MappingUIMainWindow::MappingUIMainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    ui->check_points_num->insertItem(0, "fixed");
    ui->check_points_num->setCurrentIndex(0);
    ui->check_points_num->setEnabled(false);

    ///是否有坡： 0：未知;1：有坡；2：无坡
    ui->if_slop->insertItem(0, "未知");
    ui->if_slop->insertItem(1, "有坡");
    ui->if_slop->insertItem(2, "无坡");
    ui->if_slop->setCurrentIndex(0);
    ui->if_slop->setEnabled(true);

    ui->task_type_box->insertItem(0, "新建");
    // ui->task_type_box->insertItem(1, "拓展");
    // ui->task_type_box->insertItem(2, "更新");
    ui->task_type_box->setCurrentIndex(0);

    ui->upload_result->setEnabled(false);

    running_ = false;
    preparing_ = false;
    uploading_status_ = -1;
    if_update_date_ready_ = 1;
    using_local_data_for_updating_ = true;

    ui->statusBar->showMessage("就绪", 0);

    timer_ = std::make_shared<QTimer>(this);
    timer_uploading_ = std::make_shared<QTimer>(this);
    connect(timer_.get(), SIGNAL(timeout()), this, SLOT(update()));
    connect(this, SIGNAL(progress(double)), this, SLOT(set_process_value(double)));
    connect(this, SIGNAL(set_status(const QString &)), this, SLOT(show_status(const QString &)));
    connect(timer_uploading_.get(), SIGNAL(timeout()), this, SLOT(uploadind_status_update()));
}

MappingUIMainWindow::~MappingUIMainWindow() { delete ui; }

void MappingUIMainWindow::on_start_mapping_clicked() {
    if (!check_mapping_start_condition()) {
        QMessageBox::warning(this, "Warning", "无法开始建图");
        return;
    }

    if (start_mapping() == false) {
        QMessageBox::warning(this, "Warning", "无法开始建图");
        ui->start_mapping->setEnabled(true);
        return;
    }
}

void MappingUIMainWindow::run_mapping() {
    LOG(INFO) << "call run_mapping";
    // set_status("准备文件中");
    preparing_ = true;

    mapping::io::YAML_IO config_yaml(config_file_name_.toUtf8().constData());
    std::string area_id = config_yaml.GetValue<std::string>("area_id");
    int task_id = config_yaml.GetValue<int>("task_id");
    int task_type = config_yaml.GetValue<int>("task_type");

    if (task_type == 2) {  //如果是拓展任务，下载被拓展地图数据，解压并判断是否存在所需要的数据。
        LOG(ERROR) << "Current not support to expand map!";
        return;
        // std::string target_map_data_path = "/home/idriver/results/" + area_id;
        // if (!using_local_data_for_updating_) {
        //     std::string cmd =
        //             R"(curl
        //             http://cluster2.avcp.idriverplus.com/frontend/web/index.php/mapCollection/process/match-data?access-token\=)"
        //             + account_token_ + R"(\&task_id\=)" + std::to_string(task_id) + R"(\&area_id\=)" + area_id +
        //             R"(\&file_type\=2)" + " > ./err.json";
        //     LOG(INFO) << cmd;
        //     system(cmd.c_str());

        //     std::ifstream fin("./err.json");
        //     Json::CharReaderBuilder builder;
        //     Json::Value root;
        //     builder["collectComments"] = true;
        //     JSONCPP_STRING errs;
        //     std::string data_file_url;
        //     if (!Json::parseFromStream(builder, fin, &root, &errs)) {
        //         LOG(INFO) << "parse failed";
        //         if_update_date_ready_ = -3;
        //         return;
        //     } else {
        //         int err_no = root["err_no"].asInt();
        //         LOG(INFO) << "err no = " << err_no;
        //         if (err_no == 200) {
        //             data_file_url = root["result"]["PreProcessingData_file_url"].asString();
        //             LOG(INFO) << "data_file_url: " << data_file_url;
        //         } else {
        //             LOG(INFO) << "err no is wrong: err_no = " << err_no;
        //             if_update_date_ready_ = -4;
        //             return;
        //         }
        //     }

        //     cmd = "rm -rf " + target_map_data_path + "/*";
        //     LOG(INFO) << "cmd: " << cmd;
        //     system(cmd.c_str());

        //     cmd = "mkdir -p " + target_map_data_path;
        //     LOG(INFO) << "cmd: " << cmd;
        //     system(cmd.c_str());

        //     cmd = std::string("wget -c ") + " \"" + data_file_url + "\" -O \"" + target_map_data_path +
        //           "/mapping_result.zip\"";
        //     LOG(INFO) << "cmd: " << cmd;
        //     system(cmd.c_str());

        //     cmd = "unzip -qq -o " + target_map_data_path + "/mapping_result.zip -d " + target_map_data_path + "/";
        //     if (system(cmd.c_str())) {
        //         LOG(ERROR) << "unzip target map data failed.";
        //         if_update_date_ready_ = -1;
        //         return;
        //     }
        // }

        // std::string keyframes_path, map_ori_path, map_path;
        // if (!mapping::common::FindFile(target_map_data_path, "keyframes.txt", keyframes_path)
        //     || !mapping::common::FindFile(target_map_data_path, "map_ori.db", map_ori_path)
        //     || !mapping::common::FindFile(target_map_data_path, "map.db", map_path)) {
        //     LOG(ERROR) << "Some files doesn't exist.";
        //     if_update_date_ready_ = -2;
        //     return;
        // }
    }

    double lon = config_yaml.GetValue<double>("lon");
    double lat = config_yaml.GetValue<double>("lat");
    double height = 0.0;
    mapping::common::OriginPointInformation map_origin;
    map_origin = GetMapOffsetFromConfig(lon, lat, height);

    // 写配置
    mapping::io::YAML_IO server_yaml("./config/server.yaml");
    server_yaml.SetValue<std::string>("server.ip", "127.0.0.1");
    server_yaml.SetValue<bool>("server.use_internal_ip", false);
    server_yaml.Save("./config/server.yaml");

    mapping::io::YAML_IO mapping_yaml("./config/mapping.yaml");
    mapping_yaml.SetValue<bool>("send_email_after_complete", ui->auto_send_email->isChecked());
    mapping_yaml.SetValue<bool>("open_report_after_complete", ui->auto_open_report->isChecked());
    mapping_yaml.SetValue<std::string>("map_name", area_id);
    mapping_yaml.SetValue<std::string>("data_fetching", "data_url", data_file_name_.toUtf8().constData());

    if (ui->if_slop->currentIndex() == 1) {
        mapping_yaml.SetValue<bool>("optimization_params", "with_height", false);  ////1：有坡
    } else {
        mapping_yaml.SetValue<bool>("optimization_params", "with_height", true);  /////0：未知；2：无坡
    }

    if (task_type == 2) {
        mapping_yaml.SetValue("if_merge_maps", true);
        std::string origin_map_db_path = "/home/idriver/results/" + area_id;
        mapping_yaml.SetValue("origin_map_db_path", origin_map_db_path);
    } else {
        mapping_yaml.SetValue("if_merge_maps", false);
    }

    if (map_origin.map_origin_zone > 0) {
        mapping_yaml.SetValue<double>("map_origin_param", "map_origin_x", map_origin.map_origin_x);
        mapping_yaml.SetValue<double>("map_origin_param", "map_origin_y", map_origin.map_origin_y);
        mapping_yaml.SetValue<double>("map_origin_param", "map_origin_z", map_origin.map_origin_z);
        mapping_yaml.SetValue<int>("map_origin_param", "map_origin_zone", map_origin.map_origin_zone);
        mapping_yaml.SetValue<bool>("map_origin_param", "is_southern", map_origin.is_southern);
    }
    mapping_yaml.Save("./config/mapping.yaml");

    // 拷贝数据文件
    std::string data_file_name = data_file_name_.toUtf8().constData();
    std::string file_name = config_file_name_.toUtf8().constData();

    LOG(INFO) << "copy files";

    // clean directory if exist
    if (boost::filesystem::exists("/home/idriver/data/" + area_id)) {
        std::string cmd = "rm -rf /home/idriver/data/" + area_id;
        LOG(INFO) << cmd;
        system(cmd.c_str());

        cmd = "mkdir -p /home/idriver/data/" + area_id;
        LOG(INFO) << cmd;
        system(cmd.c_str());
    } else {
        // create a new directory here
        std::string cmd = "mkdir -p /home/idriver/data/" + area_id;
        LOG(INFO) << cmd;
        system(cmd.c_str());
    }

    // create results directory if not exist
    if (boost::filesystem::exists("/home/idriver/results/" + area_id)) {
        std::string cmd = "mkdir -p /home/idriver/results/" + area_id;
        LOG(INFO) << cmd;
        system(cmd.c_str());
    }

    // copy the data file and config to target directory
    std::string cmd = "cp " + data_file_name + " /home/idriver/data/" + area_id + "/";
    LOG(INFO) << cmd;
    system(cmd.c_str());

    cmd = "cp " + file_name + " /home/idriver/data/" + area_id + "/";
    LOG(INFO) << cmd;
    system(cmd.c_str());

    // 启动流水线
    LOG(INFO) << "creating pipeline";
    mapping_engine_ = std::make_shared<PipelineEngine>("127.0.0.1");
    mapping_engine_->Init("./config/mapping.yaml");

    LOG(INFO) << "Setting task info";
    // 设置task info
    auto task_info = mapping_engine_->GetTaskInfo();
    task_info.enable_cloud_config = true;
    task_info.account_name = account_name_;
    task_info.access_token = account_token_;
    task_info.area_id = area_id;
    task_info.project_name = config_yaml.GetValue<std::string>("project_name");
    task_info.version = config_yaml.GetValue<std::string>("version");
    task_info.task_id = config_yaml.GetValue<int>("task_id");

    LOG(INFO) << "Set task info";
    mapping_engine_->SetTaskInfo(task_info);

    preparing_ = false;
    running_ = true;

    LOG(INFO) << "Start pipeline";
    show_status("准备文件中");
    mapping_engine_->StartPipeline();
}

bool MappingUIMainWindow::start_mapping() {
    // 一些准备工作
    ui->start_mapping->setEnabled(false);
    preparing_ = false;
    running_ = false;

    mapping::io::YAML_IO config_yaml(config_file_name_.toUtf8().constData());
    std::string area_id = config_yaml.GetValue<std::string>("area_id");
    int task_id = config_yaml.GetValue<int>("task_id");
    int task_type = config_yaml.GetValue<int>("task_type");

    if (task_type == 2) {  //如果是拓展任务，下载被拓展地图数据，解压并判断是否存在所需要的数据。
        ui->task_type_box->setCurrentIndex(1);
        if (QMessageBox::question(this, "提示", "需要拓展的地图是否位于本地？") != QMessageBox::Yes) {
            using_local_data_for_updating_ = false;
        } else {
            using_local_data_for_updating_ = true;
            if_update_date_ready_ = 1;
        }
    }

    QMessageBox::information(this, "消息", "建图已开始，请耐心等待，不要在完成前关闭本程序！");
    set_status("准备文件中");
    mapping_result_ = QtConcurrent::run(this, &MappingUIMainWindow::run_mapping);

    // 启动轮询
    timer_->start(200);

    return true;
}

mapping::common::OriginPointInformation MappingUIMainWindow::GetMapOffsetFromConfig(const double &lon,
                                                                                    const double &lat,
                                                                                    const double &height) {
    mapping::common::OriginPointInformation map_origin;
    double antenna_x = 0.0, antenna_y = 0.0, antenna_angle = 0.0;
    GpsTransform origin_gps_transform(antenna_x, antenna_y, antenna_angle);

    if (lon < -180 || lon > 180 || lat < -90 || lat > 90) {
        map_origin.map_origin_zone = -1;
        return map_origin;
    }
    map_origin = origin_gps_transform.LatLonToOriginInfo(lon, lat, height);

    return map_origin;
}

void MappingUIMainWindow::update() {
    if (if_update_date_ready_.load() < 0) {
        if (if_update_date_ready_.load() == -1) {
            QMessageBox::warning(this, "Warning", "被扩展地图数据解压失败，请手动检测已下载数据是否可用！");
        } else if (if_update_date_ready_.load() == -2) {
            QMessageBox::warning(this, "Warning", "被扩展地图数据文件缺失，请手动检测已下载数据是否可用！");
        } else if (if_update_date_ready_.load() == -3) {
            QMessageBox::warning(this, "Warning", "原始数据下载失败，检查网络！");
        } else if (if_update_date_ready_.load() == -4) {
            QMessageBox::warning(this, "Warning", "原始数据下载失败，联系管理员检查原始数据！");
        }
        if_update_date_ready_ = 1;
        timer_->stop();
        return;
    }

    if (preparing_) return;
    if (running_ == false) {
        progress(100);
        ui->start_mapping->setEnabled(true);
        timer_->stop();
        return;
    }

    PipelineResult result;
    {
        std::unique_lock<std::mutex> lock(engine_mutex_);
        result = mapping_engine_->GetResult();
    }

    // 设置进度条
    // data fetching 0-5
    // data preparing 5-10
    // lidar odom 10-30
    // pose opti 30-50
    // data verification 50-100

    std::string step = result.current_working;
    bool finished = result.finished;
    if (step != current_step_) cnt_seconds_ = 0;  // reset cnt
    current_step_ = step;
    double last_predict_time = -0.1;

    if (finished) {
        auto task_info = mapping_engine_->GetTaskInfo();
        if (!task_info.checkin_passed) {
            QMessageBox::warning(this, "Warning", "建图数据存在问题，请检查数据及文件夹命名等是否符合规范！");
        }
        current_process_ = 95;
        uploading_result_ = QtConcurrent::run(this, &MappingUIMainWindow::finish_mapping);
        timer_->stop();
        timer_uploading_->start(1000);
    } else if (current_step_ == "Data_Fetching") {
        current_process_ = 0 + cnt_seconds_ / 5;
        current_process_ = current_process_ >= 5 ? 5 : current_process_;
        set_status("解压数据中");
    } else if (current_step_ == "CheckIn") {
        current_process_ = 5 + cnt_seconds_ / 5;
        current_process_ = current_process_ >= 8 ? 8 : current_process_;
        set_status("转换格式中");
    } else if (current_step_ == "DRFrontend") {
        current_process_ = 8 + cnt_seconds_ / 5;
        current_process_ = current_process_ >= 10 ? 10 : current_process_;
        set_status("DR计算中");
    } else if (current_step_ == "LidarFrontend") {
        current_process_ = 10 + cnt_seconds_ / 20;
        current_process_ = current_process_ >= 30 ? 30 : current_process_;
        set_status("计算激光里程计");
    } else if (current_step_ == "OptimizationStage1") {
        current_process_ = 30 + cnt_seconds_ / 10;
        current_process_ = current_process_ >= 32 ? 32 : current_process_;
        set_status("第一轮优化中");
    } else if (current_step_ == "LoopClosing") {
        current_process_ = 32 + cnt_seconds_ / 20;
        current_process_ = current_process_ >= 50 ? 50 : current_process_;
        set_status("闭环优化中");
    } else if (current_step_ == "OptimizationStage2") {
        current_process_ = 50 + cnt_seconds_ / 10;
        current_process_ = current_process_ >= 52 ? 52 : current_process_;
        set_status("第二轮优化中");
    } else if (current_step_ == "Validation") {
        // auto task_info = mapping_engine_->GetTaskInfo();
        current_process_ = 52 + cnt_seconds_ / 40;
        current_process_ = current_process_ >= 92 ? 92 : current_process_;
        set_status("验证数据中");
    } else if(current_step_ == "CheckOut"){
        // auto task_info = mapping_engine_->GetTaskInfo();
        current_process_ = 92 + cnt_seconds_ / 10;
        current_process_ = current_process_ >= 95 ? 95 : current_process_;
        set_status("成果数据准出中");   
    }

    // load detail report
    ui->detailed_message->setText(result.report.c_str());
    progress(current_process_);
    cnt_seconds_ += 0.2;
    return;
}

void MappingUIMainWindow::uploadind_status_update() {
    if (uploading_status_.load() <= 0) {
        return;
    } else if (uploading_status_.load() == 1) {
        QMessageBox::information(this, "Info", "文件上传成功，建图结束！");
        timer_uploading_->stop();
    } else if (uploading_status_.load() == 2) {
        QMessageBox::warning(this, "Warning", "文件上传失败，请检查联网情况，手动上传！");
        timer_uploading_->stop();
    }
    return;
}

void MappingUIMainWindow::finish_mapping() {
    QApplication::processEvents();
    LOG(INFO) << "finishing mapping";
    set_status("上传文件中");

    uploading_status_ = 0;

    mapping::io::YAML_IO config_yaml(config_file_name_.toUtf8().constData());
    mapping::io::UploadConfig upload_config;
    upload_config.file_upload_url = config_yaml.GetValue<std::string>("file_upload_url");
    upload_config.upload_data_type = "PreProcessingData";
    upload_config.version = config_yaml.GetValue<std::string>("version");
    std::string area_id = config_yaml.GetValue<std::string>("area_id");

    auto task_info = mapping_engine_->GetTaskInfo();
    task_info_ = task_info;

    mapping::io::OSS_IO oss(upload_config);
    std::string full_name =
        upload_config.upload_data_type + upload_config.file_upload_url + upload_config.version + "/mapping_result.zip";

    LOG(INFO) << "uploading results)";
    if (oss.upload("/home/idriver/results/" + area_id)) {
        // curl 传文件进度
        std::string cmd =
            std::string(
                R"(curl http://cluster2.avcp.idriverplus.com/frontend/web/index.php/mapCollection/process/file-upload-status)") +
            R"(\?access-token\=)" + account_token_ + R"(\&task_id\=)" + std::to_string(task_info.task_id) +
            R"(\&area_id\=)" + task_info.area_id + R"(\&version\=)" + task_info.version + R"(\&process\=100)" +
            R"(\&status\=2)" + R"(\&finish_time\=)" + std::to_string(100) + R"(\&file_type\=2)" + R"(\&file_url\=)" +
            full_name;
        LOG(INFO) << cmd;
        system(cmd.c_str());
        progress(100);
        set_status("就绪");
        uploading_status_ = 1;
        // QMessageBox::information(this, "Info", "建图结束！");
    } else {
        // 上传失败
        std::string cmd =
            std::string(
                R"(curl http://cluster2.avcp.idriverplus.com/frontend/web/index.php/mapCollection/process/file-upload-status)") +
            R"(\?access-token\=)" + account_token_ + R"(\&task_id\=)" + std::to_string(task_info.task_id) +
            R"(\&area_id\=)" + task_info.area_id + R"(\&version\=)" + task_info.version + R"(\&process\=100)" +
            R"(\&status\=1)" + R"(\&finish_time\=)" + std::to_string(100) + R"(\&file_type\=2)" + R"(\&file_url\=)" +
            full_name;
        LOG(INFO) << cmd;
        system(cmd.c_str());

        progress(100);
        set_status("就绪");
        uploading_status_ = 2;
        // QMessageBox::warning(this, "Warning", "上传失败，请查看联网情况并手动上传结果文件");
        ui->upload_result->setEnabled(true);
    }

    {
        std::unique_lock<std::mutex> lock(engine_mutex_);
        mapping_engine_->ClosePipeline();
        mapping_engine_ = nullptr;
    }
    ui->start_mapping->setEnabled(true);
    running_ = false;
}

void MappingUIMainWindow::closeEvent(QCloseEvent *event) {
    if (running_ && uploading_status_.load() <= 0) {
        if (QMessageBox::question(this, "提示", "建图程序正在运行中，关闭窗口将中断本次建图，确认吗？") ==
            QMessageBox::Yes) {
            event->accept();
        } else {
            event->ignore();
            return;
        }
    }

    event->accept();
}

bool MappingUIMainWindow::check_mapping_start_condition() {
    if (data_file_name_.isEmpty()) {
        QMessageBox::warning(this, "错误", "未找到建图数据");
        ;
        return false;
    }

    return true;
}

void MappingUIMainWindow::on_toolButton_clicked() {
    QFileDialog fd(this, "Select file", "", "config文件(*.yaml);;");
    if (fd.exec() == QDialog::Accepted) {
        auto filelist = fd.selectedFiles();  //返回文件列表的名称
        ui->map_data_path->setText(filelist[0]);
        auto index = filelist[0].lastIndexOf('/');

        data_path_ = filelist[0].mid(0, index);
        config_file_name_ = filelist[0];

        // 检查config.yaml同目录下是否有data.zip或data.rar
        QString zip_path = data_path_ + "/data.zip";
        QString rar_path = data_path_ + "/data.rar";
        if (QFileInfo(zip_path).exists()) {
            data_file_name_ = zip_path;
        } else if (QFileInfo(rar_path).exists()) {
            data_file_name_ = rar_path;
        } else {
            QMessageBox::warning(this, "错误", "未在 " + data_path_ + " 目录下找到data.zip或data.rar，无法建图");
            ;
            return;
        }

        // 推测预计时间
        std::string data_path = data_file_name_.toUtf8().constData();
        auto data_size = boost::filesystem::file_size(data_path);
        double data_size_in_GB = double(data_size) / 1024.0 / 1024.0 / 1024.0;
        double minute_predict = data_size_in_GB * 35;
        int hours = int(minute_predict / 60);
        int minutes = minute_predict - 60 * hours;
        QString predict_time;
        predict_time.sprintf("%d 小时 %d 分钟", hours, minutes);
        ui->mapping_wait_time->setPlainText(predict_time);
    } else {
        fd.close();
    }
}

void MappingUIMainWindow::on_detailed_message_textChanged() { ui->detailed_message->moveCursor(QTextCursor::End); }

void MappingUIMainWindow::on_upload_result_clicked() {
    ui->upload_result->setEnabled(false);
    set_status("上传文件中");
    uploading_result_ = QtConcurrent::run(this, &MappingUIMainWindow::reupload_mapping_result);
    timer_uploading_->start(1000);
}

void MappingUIMainWindow::reupload_mapping_result() {
    uploading_status_ = 0;

    mapping::io::YAML_IO config_yaml(config_file_name_.toUtf8().constData());
    mapping::io::UploadConfig upload_config;
    upload_config.file_upload_url = config_yaml.GetValue<std::string>("file_upload_url");
    upload_config.upload_data_type = "PreProcessingData";
    upload_config.version = config_yaml.GetValue<std::string>("version");
    std::string area_id = config_yaml.GetValue<std::string>("area_id");

    mapping::io::OSS_IO oss(upload_config);
    std::string full_name =
        upload_config.upload_data_type + upload_config.file_upload_url + upload_config.version + "/mapping_result.zip";

    if (oss.upload("/home/idriver/results/" + area_id)) {
        // curl 传文件进度
        std::string cmd =
            std::string(
                R"(curl http://cluster2.avcp.idriverplus.com/frontend/web/index.php/mapCollection/process/file-upload-status)") +
            R"(\?access-token\=)" + account_token_ + R"(\&task_id\=)" + std::to_string(task_info_.task_id) +
            R"(\&area_id\=)" + task_info_.area_id + R"(\&version\=)" + task_info_.version + R"(\&process\=100)" +
            R"(\&status\=2)" + R"(\&finish_time\=)" + std::to_string(100) + R"(\&file_type\=2)" + R"(\&file_url\=)" +
            full_name;
        LOG(INFO) << cmd;
        system(cmd.c_str());
        uploading_status_ = 1;
        // QMessageBox::information(this, "Info", "手动上传结果文件成功！");
        set_status("就绪");
    } else {
        // 上传失败
        std::string cmd =
            std::string(
                R"(curl http://cluster2.avcp.idriverplus.com/frontend/web/index.php/mapCollection/process/file-upload-status)") +
            R"(\?access-token\=)" + account_token_ + R"(\&task_id\=)" + std::to_string(task_info_.task_id) +
            R"(\&area_id\=)" + task_info_.area_id + R"(\&version\=)" + task_info_.version + R"(\&process\=100)" +
            R"(\&status\=1)" + R"(\&finish_time\=)" + std::to_string(100) + R"(\&file_type\=2)" + R"(\&file_url\=)" +
            full_name;
        LOG(INFO) << cmd;
        system(cmd.c_str());
        uploading_status_ = 2;

        // QMessageBox::warning(this, "Warning", "手动上传结果文件失败！");
        ui->upload_result->setEnabled(true);
    }
}
