#ifndef OPTIMIZATIONDIALOG_H
#define OPTIMIZATIONDIALOG_H

#include <QDialog>

namespace Ui {
class OptimizationDialog;
}

class OptimizationDialog : public QDialog {
    Q_OBJECT

   public:
    struct OptimizationParams {
        uint iteration_num = 100;
        bool with_height = false;  // 使用层级和高度约束
    };

    explicit OptimizationDialog(QWidget *parent = nullptr);
    ~OptimizationDialog();

    void GetOptimizationaParams(OptimizationParams &params);

   private:
    Ui::OptimizationDialog *ui;
};

#endif  // OPTIMIZATIONDIALOG_H
