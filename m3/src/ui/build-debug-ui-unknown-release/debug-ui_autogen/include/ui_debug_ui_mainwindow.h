/********************************************************************************
** Form generated from reading UI file 'debug_ui_mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DEBUG_UI_MAINWINDOW_H
#define UI_DEBUG_UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QWidget *widget;
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QHBoxLayout *horizontalLayout;
    QLineEdit *db_path_;
    QToolButton *db_path_btn_;
    QHBoxLayout *horizontalLayout_2;
    QLineEdit *keyframe_path_;
    QToolButton *kf_path_btn_;
    QPushButton *load_map_btn_;
    QVTKWidget *pcl_window_;
    QGroupBox *groupBox;
    QLabel *label_4;
    QSpinBox *loop_kf_1_;
    QSpinBox *loop_kf_2_;
    QLabel *label_13;
    QPushButton *add_loop_btn_;
    QLineEdit *num_loops_;
    QLabel *label_18;
    QGroupBox *groupBox_4;
    QTextBrowser *opti_report_;
    QPushButton *call_optimize_btn_;
    QPushButton *reset_optimize_btn_;
    QGroupBox *groupBox_2;
    QLabel *label_5;
    QSpinBox *current_kf_id_box_;
    QPushButton *focus_cur_kf_btn_;
    QLabel *label_6;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_12;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_2;
    QDoubleSpinBox *cur_kf_x_display_;
    QDoubleSpinBox *cur_kf_y_display_;
    QDoubleSpinBox *cur_kf_z_display_;
    QDoubleSpinBox *cur_kf_roll_display_;
    QDoubleSpinBox *cur_kf_pitch_display_;
    QDoubleSpinBox *cur_kf_yaw_display_;
    QPushButton *play_through_btn_;
    QGroupBox *groupBox_5;
    QCheckBox *use_internal_loop_closing_;
    QPushButton *call_loop_closing_;
    QPushButton *fix_current_btn_;
    QPushButton *clear_fixed_btn_1;
    QGroupBox *groupBox_3;
    QLabel *label_14;
    QCheckBox *show_optimization_check_;
    QCheckBox *show_pose_graph_check_;
    QComboBox *resolution_box_;
    QLabel *label_15;
    QDoubleSpinBox *camera_height_;
    QCheckBox *show_pose_graph_check_1;
    QCheckBox *highlight_current_;
    QCheckBox *highlight_loop_;
    QCheckBox *show_point_cloud_;
    QSlider *play_speed_;
    QLabel *label_16;
    QCheckBox *camera_follow_current_;
    QFrame *line;
    QLabel *label_2;
    QLineEdit *cur_kf_info_;
    QLabel *label_3;
    QLineEdit *cur_map_info_;
    QPushButton *save_map_btn_;
    QPushButton *reset_map_btn_;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1386, 717);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        widget = new QWidget(centralWidget);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(7, 0, 1361, 651));
        gridLayout = new QGridLayout(widget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        label = new QLabel(widget);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout->addWidget(label);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        db_path_ = new QLineEdit(widget);
        db_path_->setObjectName(QStringLiteral("db_path_"));
        db_path_->setReadOnly(true);

        horizontalLayout->addWidget(db_path_);

        db_path_btn_ = new QToolButton(widget);
        db_path_btn_->setObjectName(QStringLiteral("db_path_btn_"));

        horizontalLayout->addWidget(db_path_btn_);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        keyframe_path_ = new QLineEdit(widget);
        keyframe_path_->setObjectName(QStringLiteral("keyframe_path_"));
        keyframe_path_->setReadOnly(true);

        horizontalLayout_2->addWidget(keyframe_path_);

        kf_path_btn_ = new QToolButton(widget);
        kf_path_btn_->setObjectName(QStringLiteral("kf_path_btn_"));

        horizontalLayout_2->addWidget(kf_path_btn_);


        verticalLayout->addLayout(horizontalLayout_2);

        load_map_btn_ = new QPushButton(widget);
        load_map_btn_->setObjectName(QStringLiteral("load_map_btn_"));

        verticalLayout->addWidget(load_map_btn_);


        gridLayout->addLayout(verticalLayout, 0, 0, 1, 2);

        pcl_window_ = new QVTKWidget(widget);
        pcl_window_->setObjectName(QStringLiteral("pcl_window_"));

        gridLayout->addWidget(pcl_window_, 0, 2, 5, 3);

        groupBox = new QGroupBox(widget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(0, 30, 91, 36));
        loop_kf_1_ = new QSpinBox(groupBox);
        loop_kf_1_->setObjectName(QStringLiteral("loop_kf_1_"));
        loop_kf_1_->setGeometry(QRect(110, 30, 61, 33));
        loop_kf_2_ = new QSpinBox(groupBox);
        loop_kf_2_->setObjectName(QStringLiteral("loop_kf_2_"));
        loop_kf_2_->setGeometry(QRect(110, 70, 61, 33));
        label_13 = new QLabel(groupBox);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(0, 70, 81, 36));
        add_loop_btn_ = new QPushButton(groupBox);
        add_loop_btn_->setObjectName(QStringLiteral("add_loop_btn_"));
        add_loop_btn_->setGeometry(QRect(30, 110, 121, 32));
        num_loops_ = new QLineEdit(groupBox);
        num_loops_->setObjectName(QStringLiteral("num_loops_"));
        num_loops_->setGeometry(QRect(80, 160, 91, 32));
        num_loops_->setReadOnly(true);
        label_18 = new QLabel(groupBox);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(10, 160, 71, 36));

        gridLayout->addWidget(groupBox, 0, 5, 2, 2);

        groupBox_4 = new QGroupBox(widget);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        opti_report_ = new QTextBrowser(groupBox_4);
        opti_report_->setObjectName(QStringLiteral("opti_report_"));
        opti_report_->setGeometry(QRect(10, 70, 241, 301));
        call_optimize_btn_ = new QPushButton(groupBox_4);
        call_optimize_btn_->setObjectName(QStringLiteral("call_optimize_btn_"));
        call_optimize_btn_->setGeometry(QRect(10, 30, 97, 32));
        reset_optimize_btn_ = new QPushButton(groupBox_4);
        reset_optimize_btn_->setObjectName(QStringLiteral("reset_optimize_btn_"));
        reset_optimize_btn_->setGeometry(QRect(120, 30, 97, 32));

        gridLayout->addWidget(groupBox_4, 0, 7, 4, 3);

        groupBox_2 = new QGroupBox(widget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        label_5 = new QLabel(groupBox_2);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(10, 30, 61, 36));
        current_kf_id_box_ = new QSpinBox(groupBox_2);
        current_kf_id_box_->setObjectName(QStringLiteral("current_kf_id_box_"));
        current_kf_id_box_->setGeometry(QRect(70, 30, 111, 33));
        focus_cur_kf_btn_ = new QPushButton(groupBox_2);
        focus_cur_kf_btn_->setObjectName(QStringLiteral("focus_cur_kf_btn_"));
        focus_cur_kf_btn_->setGeometry(QRect(10, 70, 91, 32));
        label_6 = new QLabel(groupBox_2);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(10, 100, 61, 36));
        verticalLayoutWidget_3 = new QWidget(groupBox_2);
        verticalLayoutWidget_3->setObjectName(QStringLiteral("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(20, 130, 31, 231));
        verticalLayout_3 = new QVBoxLayout(verticalLayoutWidget_3);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_7 = new QLabel(verticalLayoutWidget_3);
        label_7->setObjectName(QStringLiteral("label_7"));

        verticalLayout_3->addWidget(label_7);

        label_8 = new QLabel(verticalLayoutWidget_3);
        label_8->setObjectName(QStringLiteral("label_8"));

        verticalLayout_3->addWidget(label_8);

        label_9 = new QLabel(verticalLayoutWidget_3);
        label_9->setObjectName(QStringLiteral("label_9"));

        verticalLayout_3->addWidget(label_9);

        label_10 = new QLabel(verticalLayoutWidget_3);
        label_10->setObjectName(QStringLiteral("label_10"));

        verticalLayout_3->addWidget(label_10);

        label_11 = new QLabel(verticalLayoutWidget_3);
        label_11->setObjectName(QStringLiteral("label_11"));

        verticalLayout_3->addWidget(label_11);

        label_12 = new QLabel(verticalLayoutWidget_3);
        label_12->setObjectName(QStringLiteral("label_12"));

        verticalLayout_3->addWidget(label_12);

        verticalLayoutWidget_2 = new QWidget(groupBox_2);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(60, 130, 115, 230));
        verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        cur_kf_x_display_ = new QDoubleSpinBox(verticalLayoutWidget_2);
        cur_kf_x_display_->setObjectName(QStringLiteral("cur_kf_x_display_"));
        cur_kf_x_display_->setDecimals(3);
        cur_kf_x_display_->setMinimum(-99999);
        cur_kf_x_display_->setMaximum(99999);
        cur_kf_x_display_->setSingleStep(0.05);

        verticalLayout_2->addWidget(cur_kf_x_display_);

        cur_kf_y_display_ = new QDoubleSpinBox(verticalLayoutWidget_2);
        cur_kf_y_display_->setObjectName(QStringLiteral("cur_kf_y_display_"));
        cur_kf_y_display_->setDecimals(3);
        cur_kf_y_display_->setMinimum(-99999);
        cur_kf_y_display_->setMaximum(99999);
        cur_kf_y_display_->setSingleStep(0.05);

        verticalLayout_2->addWidget(cur_kf_y_display_);

        cur_kf_z_display_ = new QDoubleSpinBox(verticalLayoutWidget_2);
        cur_kf_z_display_->setObjectName(QStringLiteral("cur_kf_z_display_"));
        cur_kf_z_display_->setDecimals(3);
        cur_kf_z_display_->setMinimum(-99999);
        cur_kf_z_display_->setMaximum(99999);
        cur_kf_z_display_->setSingleStep(0.05);

        verticalLayout_2->addWidget(cur_kf_z_display_);

        cur_kf_roll_display_ = new QDoubleSpinBox(verticalLayoutWidget_2);
        cur_kf_roll_display_->setObjectName(QStringLiteral("cur_kf_roll_display_"));
        cur_kf_roll_display_->setDecimals(3);
        cur_kf_roll_display_->setMinimum(-180);
        cur_kf_roll_display_->setMaximum(180);
        cur_kf_roll_display_->setSingleStep(0.1);

        verticalLayout_2->addWidget(cur_kf_roll_display_);

        cur_kf_pitch_display_ = new QDoubleSpinBox(verticalLayoutWidget_2);
        cur_kf_pitch_display_->setObjectName(QStringLiteral("cur_kf_pitch_display_"));
        cur_kf_pitch_display_->setDecimals(3);
        cur_kf_pitch_display_->setMinimum(-180);
        cur_kf_pitch_display_->setMaximum(180);
        cur_kf_pitch_display_->setSingleStep(0.1);

        verticalLayout_2->addWidget(cur_kf_pitch_display_);

        cur_kf_yaw_display_ = new QDoubleSpinBox(verticalLayoutWidget_2);
        cur_kf_yaw_display_->setObjectName(QStringLiteral("cur_kf_yaw_display_"));
        cur_kf_yaw_display_->setDecimals(3);
        cur_kf_yaw_display_->setMinimum(-180);
        cur_kf_yaw_display_->setMaximum(180);
        cur_kf_yaw_display_->setSingleStep(0.1);

        verticalLayout_2->addWidget(cur_kf_yaw_display_);

        play_through_btn_ = new QPushButton(groupBox_2);
        play_through_btn_->setObjectName(QStringLiteral("play_through_btn_"));
        play_through_btn_->setGeometry(QRect(110, 70, 61, 32));

        gridLayout->addWidget(groupBox_2, 1, 0, 4, 2);

        groupBox_5 = new QGroupBox(widget);
        groupBox_5->setObjectName(QStringLiteral("groupBox_5"));
        use_internal_loop_closing_ = new QCheckBox(groupBox_5);
        use_internal_loop_closing_->setObjectName(QStringLiteral("use_internal_loop_closing_"));
        use_internal_loop_closing_->setGeometry(QRect(10, 30, 151, 30));
        use_internal_loop_closing_->setChecked(true);
        call_loop_closing_ = new QPushButton(groupBox_5);
        call_loop_closing_->setObjectName(QStringLiteral("call_loop_closing_"));
        call_loop_closing_->setGeometry(QRect(20, 70, 141, 32));

        gridLayout->addWidget(groupBox_5, 2, 5, 1, 2);

        fix_current_btn_ = new QPushButton(widget);
        fix_current_btn_->setObjectName(QStringLiteral("fix_current_btn_"));

        gridLayout->addWidget(fix_current_btn_, 3, 5, 1, 1);

        clear_fixed_btn_1 = new QPushButton(widget);
        clear_fixed_btn_1->setObjectName(QStringLiteral("clear_fixed_btn_1"));

        gridLayout->addWidget(clear_fixed_btn_1, 3, 6, 1, 1);

        groupBox_3 = new QGroupBox(widget);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        label_14 = new QLabel(groupBox_3);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(12, 40, 131, 33));
        show_optimization_check_ = new QCheckBox(groupBox_3);
        show_optimization_check_->setObjectName(QStringLiteral("show_optimization_check_"));
        show_optimization_check_->setGeometry(QRect(10, 110, 151, 30));
        show_optimization_check_->setChecked(true);
        show_pose_graph_check_ = new QCheckBox(groupBox_3);
        show_pose_graph_check_->setObjectName(QStringLiteral("show_pose_graph_check_"));
        show_pose_graph_check_->setGeometry(QRect(10, 80, 151, 30));
        show_pose_graph_check_->setChecked(true);
        resolution_box_ = new QComboBox(groupBox_3);
        resolution_box_->setObjectName(QStringLiteral("resolution_box_"));
        resolution_box_->setGeometry(QRect(100, 40, 86, 32));
        resolution_box_->setMaxVisibleItems(4);
        label_15 = new QLabel(groupBox_3);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(240, 40, 71, 33));
        camera_height_ = new QDoubleSpinBox(groupBox_3);
        camera_height_->setObjectName(QStringLiteral("camera_height_"));
        camera_height_->setGeometry(QRect(310, 40, 81, 33));
        camera_height_->setMinimum(10);
        camera_height_->setMaximum(200);
        camera_height_->setSingleStep(10);
        camera_height_->setValue(100);
        show_pose_graph_check_1 = new QCheckBox(groupBox_3);
        show_pose_graph_check_1->setObjectName(QStringLiteral("show_pose_graph_check_1"));
        show_pose_graph_check_1->setGeometry(QRect(160, 80, 91, 30));
        show_pose_graph_check_1->setChecked(true);
        highlight_current_ = new QCheckBox(groupBox_3);
        highlight_current_->setObjectName(QStringLiteral("highlight_current_"));
        highlight_current_->setGeometry(QRect(160, 110, 111, 30));
        highlight_current_->setChecked(true);
        highlight_loop_ = new QCheckBox(groupBox_3);
        highlight_loop_->setObjectName(QStringLiteral("highlight_loop_"));
        highlight_loop_->setEnabled(true);
        highlight_loop_->setGeometry(QRect(280, 80, 111, 30));
        highlight_loop_->setChecked(false);
        show_point_cloud_ = new QCheckBox(groupBox_3);
        show_point_cloud_->setObjectName(QStringLiteral("show_point_cloud_"));
        show_point_cloud_->setGeometry(QRect(280, 110, 151, 30));
        show_point_cloud_->setChecked(true);
        play_speed_ = new QSlider(groupBox_3);
        play_speed_->setObjectName(QStringLiteral("play_speed_"));
        play_speed_->setGeometry(QRect(120, 160, 160, 16));
        play_speed_->setMinimum(1);
        play_speed_->setMaximum(20);
        play_speed_->setOrientation(Qt::Horizontal);
        play_speed_->setTickPosition(QSlider::TicksBelow);
        label_16 = new QLabel(groupBox_3);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(10, 150, 111, 24));
        camera_follow_current_ = new QCheckBox(groupBox_3);
        camera_follow_current_->setObjectName(QStringLiteral("camera_follow_current_"));
        camera_follow_current_->setGeometry(QRect(290, 150, 141, 30));
        camera_follow_current_->setChecked(true);

        gridLayout->addWidget(groupBox_3, 4, 5, 1, 5);

        line = new QFrame(widget);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        gridLayout->addWidget(line, 5, 0, 1, 10);

        label_2 = new QLabel(widget);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 6, 0, 1, 1);

        cur_kf_info_ = new QLineEdit(widget);
        cur_kf_info_->setObjectName(QStringLiteral("cur_kf_info_"));
        cur_kf_info_->setReadOnly(true);

        gridLayout->addWidget(cur_kf_info_, 6, 1, 1, 2);

        label_3 = new QLabel(widget);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 6, 3, 1, 1);

        cur_map_info_ = new QLineEdit(widget);
        cur_map_info_->setObjectName(QStringLiteral("cur_map_info_"));
        cur_map_info_->setReadOnly(true);

        gridLayout->addWidget(cur_map_info_, 6, 4, 1, 4);

        save_map_btn_ = new QPushButton(widget);
        save_map_btn_->setObjectName(QStringLiteral("save_map_btn_"));

        gridLayout->addWidget(save_map_btn_, 6, 8, 1, 1);

        reset_map_btn_ = new QPushButton(widget);
        reset_map_btn_->setObjectName(QStringLiteral("reset_map_btn_"));

        gridLayout->addWidget(reset_map_btn_, 6, 9, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1386, 29));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "\347\276\216\345\233\276\344\277\256\344\277\256 V1.0", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "\351\200\211\346\213\251DB\345\222\214Keyframe", Q_NULLPTR));
        db_path_btn_->setText(QApplication::translate("MainWindow", "...", Q_NULLPTR));
        kf_path_btn_->setText(QApplication::translate("MainWindow", "...", Q_NULLPTR));
        load_map_btn_->setText(QApplication::translate("MainWindow", "\350\257\273\345\217\226\345\234\260\345\233\276", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("MainWindow", "\346\240\207\350\256\260\345\233\236\347\216\257", Q_NULLPTR));
        label_4->setText(QApplication::translate("MainWindow", "\345\205\263\351\224\256\345\270\2471 \350\223\235", Q_NULLPTR));
        label_13->setText(QApplication::translate("MainWindow", "\345\205\263\351\224\256\345\270\2472 \347\273\277", Q_NULLPTR));
#ifndef QT_NO_WHATSTHIS
        add_loop_btn_->setWhatsThis(QApplication::translate("MainWindow", "<html><head/><body><p>\345\220\214\344\275\215\347\275\256\346\230\257\346\214\207\357\274\232\346\243\200\346\237\245\345\221\230\350\256\244\344\270\272\346\255\244\345\244\204\344\270\244\344\270\252\345\205\263\351\224\256\345\270\247\345\272\224\350\257\245\344\275\215\344\272\216\345\220\214\344\270\200\347\202\271\344\270\212\357\274\214\344\275\206\345\256\236\351\231\205\350\256\241\347\256\227\350\275\250\350\277\271\344\270\215\345\234\250\345\220\214\344\270\200\347\202\271\357\274\214\345\257\274\350\207\264\345\234\260\345\233\276\345\207\272\347\216\260\351\207\215\345\275\261\343\200\202\346\240\207\350\256\260\345\220\216\357\274\214\347\256\227\346\263\225\344\274\232\345\260\235\350\257\225\346\212\212\346\255\244\345\244\204\344\270\244\345\270\247\345\220\210\345\271\266\345\210\260\345\220\214\344\270\200\347\202\271\343\200\202</p></body></html>", Q_NULLPTR));
#endif // QT_NO_WHATSTHIS
        add_loop_btn_->setText(QApplication::translate("MainWindow", "\346\240\207\350\256\260\345\233\236\347\216\257", Q_NULLPTR));
        label_18->setText(QApplication::translate("MainWindow", "\346\240\207\350\256\260\346\225\260\351\207\217", Q_NULLPTR));
        groupBox_4->setTitle(QApplication::translate("MainWindow", "\344\274\230\345\214\226\344\277\241\346\201\257", Q_NULLPTR));
        call_optimize_btn_->setText(QApplication::translate("MainWindow", "\346\211\247\350\241\214\344\274\230\345\214\226", Q_NULLPTR));
        reset_optimize_btn_->setText(QApplication::translate("MainWindow", "\351\207\215\347\275\256\344\274\230\345\214\226", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "\347\274\226\350\276\221\345\205\263\351\224\256\345\270\247", Q_NULLPTR));
        label_5->setText(QApplication::translate("MainWindow", "\345\205\263\351\224\256\345\270\247", Q_NULLPTR));
        focus_cur_kf_btn_->setText(QApplication::translate("MainWindow", "\345\210\207\346\215\242\350\207\263\346\255\244\345\270\247", Q_NULLPTR));
        label_6->setText(QApplication::translate("MainWindow", "\345\276\256\350\260\203", Q_NULLPTR));
        label_7->setText(QApplication::translate("MainWindow", "X", Q_NULLPTR));
        label_8->setText(QApplication::translate("MainWindow", "Y", Q_NULLPTR));
        label_9->setText(QApplication::translate("MainWindow", "Z", Q_NULLPTR));
        label_10->setText(QApplication::translate("MainWindow", "R", Q_NULLPTR));
        label_11->setText(QApplication::translate("MainWindow", "P", Q_NULLPTR));
        label_12->setText(QApplication::translate("MainWindow", "Y", Q_NULLPTR));
        play_through_btn_->setText(QApplication::translate("MainWindow", "Play!", Q_NULLPTR));
        groupBox_5->setTitle(QApplication::translate("MainWindow", "\350\207\252\345\212\250\345\233\236\347\216\257\346\243\200\346\265\213", Q_NULLPTR));
        use_internal_loop_closing_->setText(QApplication::translate("MainWindow", "\344\275\277\347\224\250\350\207\252\345\212\250\345\233\236\347\216\257\347\273\223\346\236\234", Q_NULLPTR));
        call_loop_closing_->setText(QApplication::translate("MainWindow", "\346\211\247\350\241\214\350\207\252\345\212\250\345\233\236\347\216\257\346\243\200\346\265\213", Q_NULLPTR));
        fix_current_btn_->setText(QApplication::translate("MainWindow", "\345\233\272\345\256\232\345\275\223\345\211\215\345\270\247", Q_NULLPTR));
        clear_fixed_btn_1->setText(QApplication::translate("MainWindow", "\346\270\205\347\251\272\345\233\272\345\256\232\345\270\247", Q_NULLPTR));
        groupBox_3->setTitle(QApplication::translate("MainWindow", "\346\230\276\347\244\272\345\217\202\346\225\260", Q_NULLPTR));
        label_14->setText(QApplication::translate("MainWindow", "\347\202\271\344\272\221\345\210\206\350\276\250\347\216\207", Q_NULLPTR));
        show_optimization_check_->setText(QApplication::translate("MainWindow", "\346\230\276\347\244\272\344\274\230\345\214\226\344\277\241\346\201\257", Q_NULLPTR));
        show_pose_graph_check_->setText(QApplication::translate("MainWindow", "\346\230\276\347\244\272Pose Graph", Q_NULLPTR));
        label_15->setText(QApplication::translate("MainWindow", "\344\277\257\350\247\206\351\253\230\345\272\246", Q_NULLPTR));
        show_pose_graph_check_1->setText(QApplication::translate("MainWindow", "\346\230\276\347\244\272\350\275\250\350\277\271", Q_NULLPTR));
        highlight_current_->setText(QApplication::translate("MainWindow", "\351\253\230\344\272\256\345\275\223\345\211\215\345\270\247", Q_NULLPTR));
        highlight_loop_->setText(QApplication::translate("MainWindow", "\351\253\230\344\272\256\345\233\236\347\216\257\345\270\247", Q_NULLPTR));
        show_point_cloud_->setText(QApplication::translate("MainWindow", "\346\230\276\347\244\272\345\205\250\345\261\200\347\202\271\344\272\221", Q_NULLPTR));
        label_16->setText(QApplication::translate("MainWindow", "\350\207\252\345\212\250\346\222\255\346\224\276\351\200\237\345\272\246", Q_NULLPTR));
        camera_follow_current_->setText(QApplication::translate("MainWindow", "\351\225\234\345\244\264\350\267\237\351\232\217\345\275\223\345\211\215\345\270\247", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "\345\205\263\351\224\256\345\270\247\344\277\241\346\201\257", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "\350\277\220\350\241\214\347\212\266\346\200\201", Q_NULLPTR));
        save_map_btn_->setText(QApplication::translate("MainWindow", "\344\277\235\345\255\230\345\234\260\345\233\276", Q_NULLPTR));
        reset_map_btn_->setText(QApplication::translate("MainWindow", "\351\207\215\347\275\256\345\234\260\345\233\276", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DEBUG_UI_MAINWINDOW_H
