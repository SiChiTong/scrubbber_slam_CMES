/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *action;
    QAction *action_2;
    QAction *action_3;
    QWidget *centralWidget;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QLineEdit *map_data_path;
    QToolButton *toolButton;
    QPushButton *start_mapping;
    QLabel *label_3;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QLabel *label_5;
    QPlainTextEdit *mapping_wait_time;
    QCheckBox *auto_open_report;
    QCheckBox *auto_send_email;
    QLabel *label_4;
    QProgressBar *mapping_progress;
    QTextBrowser *detailed_message;
    QLCDNumber *mapping_process_number;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QComboBox *task_type_box;
    QLabel *label_6;
    QPushButton *upload_result;
    QLabel *label_12;
    QLineEdit *account_name_edt;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(806, 627);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        QPalette palette;
        QBrush brush(QColor(0, 0, 0, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        QBrush brush1(QColor(255, 255, 255, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Button, brush1);
        palette.setBrush(QPalette::Active, QPalette::Light, brush1);
        palette.setBrush(QPalette::Active, QPalette::Midlight, brush1);
        QBrush brush2(QColor(127, 127, 127, 255));
        brush2.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Dark, brush2);
        QBrush brush3(QColor(170, 170, 170, 255));
        brush3.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Mid, brush3);
        palette.setBrush(QPalette::Active, QPalette::Text, brush);
        palette.setBrush(QPalette::Active, QPalette::BrightText, brush1);
        palette.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Active, QPalette::Base, brush1);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Active, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Active, QPalette::AlternateBase, brush1);
        QBrush brush4(QColor(255, 255, 220, 255));
        brush4.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::ToolTipBase, brush4);
        palette.setBrush(QPalette::Active, QPalette::ToolTipText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Light, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Midlight, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Dark, brush2);
        palette.setBrush(QPalette::Inactive, QPalette::Mid, brush3);
        palette.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette.setBrush(QPalette::Inactive, QPalette::BrightText, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Inactive, QPalette::AlternateBase, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipBase, brush4);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipText, brush);
        palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Light, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Midlight, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Dark, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::Mid, brush3);
        palette.setBrush(QPalette::Disabled, QPalette::Text, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::BrightText, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Disabled, QPalette::AlternateBase, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipBase, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipText, brush);
        MainWindow->setPalette(palette);
        MainWindow->setStyleSheet(QStringLiteral(""));
        MainWindow->setToolButtonStyle(Qt::ToolButtonFollowStyle);
        MainWindow->setTabShape(QTabWidget::Triangular);
        MainWindow->setDockNestingEnabled(true);
        action = new QAction(MainWindow);
        action->setObjectName(QStringLiteral("action"));
        action_2 = new QAction(MainWindow);
        action_2->setObjectName(QStringLiteral("action_2"));
        action_3 = new QAction(MainWindow);
        action_3->setObjectName(QStringLiteral("action_3"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayoutWidget_2 = new QWidget(centralWidget);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(10, 20, 581, 36));
        verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_2 = new QLabel(verticalLayoutWidget_2);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_2->addWidget(label_2);

        map_data_path = new QLineEdit(verticalLayoutWidget_2);
        map_data_path->setObjectName(QStringLiteral("map_data_path"));

        horizontalLayout_2->addWidget(map_data_path);

        toolButton = new QToolButton(verticalLayoutWidget_2);
        toolButton->setObjectName(QStringLiteral("toolButton"));

        horizontalLayout_2->addWidget(toolButton);


        verticalLayout_2->addLayout(horizontalLayout_2);

        start_mapping = new QPushButton(centralWidget);
        start_mapping->setObjectName(QStringLiteral("start_mapping"));
        start_mapping->setGeometry(QRect(610, 440, 181, 61));
        start_mapping->setStyleSheet(QStringLiteral(""));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(10, 180, 72, 24));
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(610, 190, 181, 102));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        label_5 = new QLabel(verticalLayoutWidget);
        label_5->setObjectName(QStringLiteral("label_5"));

        verticalLayout->addWidget(label_5);

        mapping_wait_time = new QPlainTextEdit(verticalLayoutWidget);
        mapping_wait_time->setObjectName(QStringLiteral("mapping_wait_time"));
        mapping_wait_time->setReadOnly(true);

        verticalLayout->addWidget(mapping_wait_time);

        auto_open_report = new QCheckBox(centralWidget);
        auto_open_report->setObjectName(QStringLiteral("auto_open_report"));
        auto_open_report->setGeometry(QRect(610, 20, 151, 30));
        auto_open_report->setChecked(true);
        auto_send_email = new QCheckBox(centralWidget);
        auto_send_email->setObjectName(QStringLiteral("auto_send_email"));
        auto_send_email->setGeometry(QRect(610, 60, 151, 30));
        auto_send_email->setChecked(true);
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(10, 120, 91, 24));
        mapping_progress = new QProgressBar(centralWidget);
        mapping_progress->setObjectName(QStringLiteral("mapping_progress"));
        mapping_progress->setGeometry(QRect(120, 121, 661, 20));
        mapping_progress->setValue(0);
        mapping_progress->setTextVisible(false);
        mapping_progress->setInvertedAppearance(false);
        detailed_message = new QTextBrowser(centralWidget);
        detailed_message->setObjectName(QStringLiteral("detailed_message"));
        detailed_message->setGeometry(QRect(10, 220, 571, 291));
        mapping_process_number = new QLCDNumber(centralWidget);
        mapping_process_number->setObjectName(QStringLiteral("mapping_process_number"));
        mapping_process_number->setGeometry(QRect(610, 340, 181, 81));
        mapping_process_number->setStyleSheet(QStringLiteral(""));
        mapping_process_number->setFrameShape(QFrame::WinPanel);
        mapping_process_number->setFrameShadow(QFrame::Sunken);
        mapping_process_number->setSmallDecimalPoint(false);
        mapping_process_number->setDigitCount(5);
        mapping_process_number->setSegmentStyle(QLCDNumber::Outline);
        mapping_process_number->setProperty("value", QVariant(0));
        mapping_process_number->setProperty("intValue", QVariant(0));
        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(120, 150, 72, 24));
        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(190, 150, 72, 24));
        label_9 = new QLabel(centralWidget);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(270, 150, 72, 24));
        label_10 = new QLabel(centralWidget);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(350, 150, 431, 24));
        label_10->setAlignment(Qt::AlignCenter);
        label_11 = new QLabel(centralWidget);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(610, 310, 181, 24));
        task_type_box = new QComboBox(centralWidget);
        task_type_box->setObjectName(QStringLiteral("task_type_box"));
        task_type_box->setEnabled(false);
        task_type_box->setGeometry(QRect(120, 70, 161, 32));
        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(10, 70, 96, 32));
        upload_result = new QPushButton(centralWidget);
        upload_result->setObjectName(QStringLiteral("upload_result"));
        upload_result->setGeometry(QRect(320, 70, 151, 32));
        label_12 = new QLabel(centralWidget);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(530, 520, 51, 32));
        account_name_edt = new QLineEdit(centralWidget);
        account_name_edt->setObjectName(QStringLiteral("account_name_edt"));
        account_name_edt->setGeometry(QRect(610, 520, 181, 32));
        account_name_edt->setReadOnly(true);
        MainWindow->setCentralWidget(centralWidget);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "\346\231\272\350\241\214\350\200\205 \350\266\205\347\272\247\345\273\272\345\233\2763.0 \351\273\221\347\217\215\347\217\240\347\211\210", Q_NULLPTR));
        action->setText(QApplication::translate("MainWindow", "\350\275\257\344\273\266\347\211\210\346\234\254", Q_NULLPTR));
        action_2->setText(QApplication::translate("MainWindow", "\345\270\256\345\212\251", Q_NULLPTR));
        action_3->setText(QApplication::translate("MainWindow", "\351\200\211\346\213\251\345\234\260\345\233\276\346\225\260\346\215\256", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "\345\234\260\345\233\276\351\205\215\347\275\256\346\226\207\344\273\266", Q_NULLPTR));
        toolButton->setText(QApplication::translate("MainWindow", "...", Q_NULLPTR));
        start_mapping->setText(QApplication::translate("MainWindow", "\345\274\200\345\247\213\345\273\272\345\233\276", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "\346\227\245\345\277\227", Q_NULLPTR));
        label_5->setText(QApplication::translate("MainWindow", "\351\242\204\350\256\241\347\255\211\345\276\205\346\227\266\351\227\264", Q_NULLPTR));
        mapping_wait_time->setPlainText(QApplication::translate("MainWindow", "0 \345\260\217\346\227\266", Q_NULLPTR));
        auto_open_report->setText(QApplication::translate("MainWindow", "\350\207\252\345\212\250\346\211\223\345\274\200\346\212\245\345\221\212", Q_NULLPTR));
        auto_send_email->setText(QApplication::translate("MainWindow", "\350\207\252\345\212\250\345\217\221\351\200\201\351\202\256\344\273\266", Q_NULLPTR));
        label_4->setText(QApplication::translate("MainWindow", "\345\273\272\345\233\276\350\277\233\345\272\246", Q_NULLPTR));
        label_7->setText(QApplication::translate("MainWindow", "\346\225\260\346\215\256\345\207\206\345\244\207", Q_NULLPTR));
        label_8->setText(QApplication::translate("MainWindow", "\346\277\200\345\205\211\345\214\271\351\205\215", Q_NULLPTR));
        label_9->setText(QApplication::translate("MainWindow", "\350\275\250\350\277\271\344\274\230\345\214\226", Q_NULLPTR));
        label_10->setText(QApplication::translate("MainWindow", "\344\273\277\347\234\237\351\252\214\350\257\201", Q_NULLPTR));
        label_11->setText(QApplication::translate("MainWindow", "\345\273\272\345\233\276\350\277\233\345\272\246\347\231\276\345\210\206\346\257\224(100%)", Q_NULLPTR));
        task_type_box->setCurrentText(QString());
        label_6->setText(QApplication::translate("MainWindow", "\344\273\273\345\212\241\347\261\273\345\236\213", Q_NULLPTR));
        upload_result->setText(QApplication::translate("MainWindow", "\351\207\215\346\226\260\344\270\212\344\274\240\345\273\272\345\233\276\347\273\223\346\236\234", Q_NULLPTR));
        label_12->setText(QApplication::translate("MainWindow", "\345\273\272\345\233\276\345\221\230", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
