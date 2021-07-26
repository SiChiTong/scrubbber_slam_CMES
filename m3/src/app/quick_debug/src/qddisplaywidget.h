//
// Created by gaoxiang on 2021/1/22.
//

#ifndef QUICK_DEBUG_QDDISPLAYWIDGET_H
#define QUICK_DEBUG_QDDISPLAYWIDGET_H

#include <Eigen/Core>
#include <QMouseEvent>
#include <QOpenGLWidget>
#include <QRubberBand>
#include <memory>

class QuickDebugMainWindow;

namespace mapping {
class MappingInterface;
}

class QDDisplayWidget : public QOpenGLWidget {
    Q_OBJECT;

   public:
    explicit QDDisplayWidget(QWidget *parent);
    ~QDDisplayWidget();

    void SetMappingInterface(std::shared_ptr<mapping::MappingInterface> mapping_interface);

   public:
    // 和主窗口交互
    // 打开yaml文件
    bool OnOpenYAML();

    // =====================================================================
    /// 鼠标
    void OnLButtonDown(QPoint point, Qt::KeyboardModifiers modify);

    void OnRButtonDown(QPoint point, Qt::KeyboardModifiers modify);

    void OnMButtonDown(QPoint point, Qt::KeyboardModifiers modify);

    void OnLButtonUp(QPoint point, Qt::KeyboardModifiers modify);

    void OnMButtonUp(QPoint point, Qt::KeyboardModifiers modify);

    void OnRButtonUp(QPoint point, Qt::KeyboardModifiers modify);

    /// 处理事件
    virtual void mousePressEvent(QMouseEvent *event);

    virtual void mouseDoubleClickEvent(QMouseEvent *event);

    virtual void mouseReleaseEvent(QMouseEvent *event);

    virtual void mouseMoveEvent(QMouseEvent *event);

    virtual void keyPressEvent(QKeyEvent *event);

    virtual void keyReleaseEvent(QKeyEvent *event);

    virtual void wheelEvent(QWheelEvent *event);

    // =====================================================================
    /// QOpenGL Widget 接口
    virtual void initializeGL() override;

    virtual void resizeGL(int w, int h) override;

    virtual void paintGL() override;
    // =====================================================================

    void clearBttuonState() {
        mouse_left_pressed_ = false;
        mouse_middle_pressed_ = false;
    }

   signals:
    void ShowCameraPos(double x, double y);  // 发送相机位置信息

    void valueChanged(bool found, unsigned long newValue, Eigen::Vector3f pos);

   private:
    QuickDebugMainWindow *main_window_ = nullptr;
    std::shared_ptr<mapping::MappingInterface> map_interface_ = nullptr;

    bool gl_initialized_ = false;        // OpenGL 初始化
    bool mouse_left_pressed_ = false;    // 鼠标左键按下
    bool mouse_middle_pressed_ = false;  // 中键按下
    bool mouse_right_pressed_ = false;   // 右键按下
    QPoint down_point_;                  // 鼠标按下的点
    QPoint last_global_pos_;
    double theta_ = 0;
    double phi_ = 0;
    QRubberBand *mRubberBand;
    QPoint origin_;
};

#endif  // QUICK_DEBUG_QDDISPLAYWIDGET_H
