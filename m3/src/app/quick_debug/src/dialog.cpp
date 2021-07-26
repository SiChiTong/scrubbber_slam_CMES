#include "dialog.h"

#include <QString>

#include "ui_dialog.h"

Dialog::Dialog(QWidget *parent) : QDialog(parent), ui(new Ui::Dialog) {
    ui->setupUi(this);
    ui->lineEditFitThresh->setClearButtonEnabled(true);
    ui->lineEditTransEps->setClearButtonEnabled(true);
    ui->lineEditIter->setClearButtonEnabled(true);

    for (int i = 0; i < available_methods_.size(); ++i) {
        ui->comboBox->insertItem(i, QString(available_methods_[i].c_str()));
    }
}

Dialog::~Dialog() { delete ui; }

void Dialog::GetLoopCloseParams(LoopCloseParams &param) {
    if (ui->lineEditFitThresh->isModified()) {
        param.fit_thresh = ui->lineEditFitThresh->text().toDouble();
    }
    if (ui->lineEditIter->isModified()) {
        param.max_iter = ui->lineEditIter->text().toInt();
    }
    if (ui->lineEditTransEps->isModified()) {
        param.trans_eps = ui->lineEditTransEps->text().toDouble();
    }
    param.method = method_;
    if (ui->lineEditDisThresh->isModified()) {
        param.dis_thresh = ui->lineEditDisThresh->text().toDouble();
    }
}

void Dialog::on_comboBox_currentTextChanged(const QString &arg1) {
    method_ = arg1.toStdString();
    if (method_ == "ICP") {
        ui->lineEditFitThresh->setPlaceholderText(QString(std::to_string(10.0).c_str()));
        ui->lineEditTransEps->setPlaceholderText(QString(std::to_string(2.0).c_str()));
        ui->lineEditIter->setPlaceholderText(QString(std::to_string(64).c_str()));
    }
}
