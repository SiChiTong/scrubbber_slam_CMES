#include "optimizationdialog.h"
#include "ui_optimizationdialog.h"

OptimizationDialog::OptimizationDialog(QWidget *parent) : QDialog(parent), ui(new Ui::OptimizationDialog) {
    ui->setupUi(this);
}

OptimizationDialog::~OptimizationDialog() { delete ui; }

void OptimizationDialog::GetOptimizationaParams(OptimizationParams &params) {
    if (ui->lineEditIterations->isModified()) {
        params.iteration_num = ui->lineEditIterations->text().toUInt();
    }
    if (ui->checkBoxHeightConstraint->isChecked()) {
        params.with_height = true;
    }
}
