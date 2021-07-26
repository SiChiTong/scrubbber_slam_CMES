#ifndef LOGINDIALOG_H
#define LOGINDIALOG_H

#include <QDialog>

namespace Ui {
class LoginDialog;
}

class LoginDialog : public QDialog {
Q_OBJECT

public:
    explicit LoginDialog(QWidget *parent = 0);

    ~LoginDialog();

    /// 获取帐号token和名称
    void GetAccountInfo(std::string &access_token, std::string &account_name) {
        access_token = access_token_;
        account_name = account_name_;
    }
 
    int GetFinishStatus(){
        return finish_status_;
    } 

private slots:

    void on_login_btn_clicked();

    void on_exit_btn_clicked();

private:
    Ui::LoginDialog *ui;

    std::string access_token_;
    std::string account_name_;
    int finish_status_ = -1;
};

#endif // LOGINDIALOG_H
