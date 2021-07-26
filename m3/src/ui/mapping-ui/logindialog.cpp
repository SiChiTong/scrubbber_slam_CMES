#include <QMessageBox>
#include <glog/logging.h>
#include <json/json.h>

#include "logindialog.h"
#include "ui_logindialog.h"
#include <iostream>
#include <fstream>

LoginDialog::LoginDialog(QWidget *parent) :
        QDialog(parent), ui(new Ui::LoginDialog) {
    ui->setupUi(this);
}

LoginDialog::~LoginDialog() {
    delete ui;
}

void LoginDialog::on_login_btn_clicked() {
    // TODO 验证登录信息
    std::string user = ui->user_text->text().toUtf8().constData();
    std::string passwd = ui->password_text->text().toUtf8().constData();
    std::string cmd = R"(curl http://cluster2.avcp.idriverplus.com/frontend/web/index.php/mapCollection/login/index\?account\=)" + user +
                      R"(\&password\=)" + passwd + " > user.json";
    system(cmd.c_str());

    // load response from user.xml
    bool login_success = false;
    std::ifstream fin("./user.json");
    Json::CharReaderBuilder builder;
    Json::Value root;
    builder["collectComments"] = true;
    JSONCPP_STRING errs;

    if (!Json::parseFromStream(builder, fin, &root, &errs)) {
        LOG(INFO) << "parse failed";
        login_success = false;
    } else {
        int err_no = root["err_no"].asInt();
        LOG(INFO) << "err no = " << err_no;
        if (err_no == 200) {
            login_success = true;
            access_token_ = root["result"]["access-token"].asString();
            account_name_ = root["result"]["account"].asString();

            LOG(INFO) << "login name: " << account_name_ << ", token: " << access_token_;
        }
    }

    cmd = "rm user.json";
    system(cmd.c_str());

    if (login_success) {
        QMessageBox::information(this, "Info", QString("登录成功"));
        finish_status_ = 1;
        return QDialog::accept();
    } else {
        finish_status_ = 0;
        QMessageBox::information(this, "Info", QString("登录失败"));
        return QDialog::reject();
    }

}

void LoginDialog::on_exit_btn_clicked() {
    finish_status_ = -1;
    return QDialog::reject();
}
