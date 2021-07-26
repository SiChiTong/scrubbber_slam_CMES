#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>

namespace Ui {
class Dialog;
}

class Dialog : public QDialog {
    Q_OBJECT

   public:
    explicit Dialog(QWidget *parent = nullptr);
    ~Dialog();

    struct LoopCloseParams {
        double trans_eps = 0.01;
        uint8_t max_iter = 200;
        double fit_thresh = 1.8;
        std::string method = "NDT";
        double dis_thresh = 60;
    };

    void GetLoopCloseParams(LoopCloseParams &param);

   private slots:
    void on_comboBox_currentTextChanged(const QString &arg1);

   private:
    Ui::Dialog *ui;

    std::vector<std::string> available_methods_ = {"NDT", "MM", "ICP", "GICP"};

    std::string method_ = "NDT";
};

#endif  // DIALOG_H
